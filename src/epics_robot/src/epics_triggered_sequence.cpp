// EPICS PV Triggered Multi-Holder Sample Load Sequence
// Sequential movement through multiple waypoints with EPICS CA triggering
//
// Flow:
//   1. Wait for EPICS PV trigger (non-zero value)
//   2. Reset PV to 0
//   3. Execute entire sequence continuously
//   4. Return to step 1 (wait for next trigger)
//
// Parameters:
//   - epics_trigger_pv: PV name to monitor for trigger (default: "ROBOT:TRIGGER")
//   - start_from_step: Skip steps before this number (default: 0)
//
// Intended for UR3e + Hand-E:
//   arm_group: ur_arm
//   hand_group: hand (SRDF group states "open"/"close" must exist)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/executors.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include <sstream>
#include <iostream>
#include <future>
#include <functional>
#include <set>
#include <ctime>
#include <iomanip>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/cartesian_interpolator.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/action/execute_trajectory.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

// EPICS Channel Access
#include <cadef.h>
#include <db_access.h>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>

// YAML parsing
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace mtc = moveit::task_constructor;
using mtc::Stage;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("epics_triggered_sequence");

// Get current timestamp in human-readable format
static std::string get_timestamp()
{
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch()) % 1000;
  
  std::tm local_tm;
  localtime_r(&time_t_now, &local_tm);
  
  std::ostringstream oss;
  oss << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
      << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return oss.str();
}

// Global variables
static std::atomic<bool> shutdown_requested{false};
static rclcpp::executors::SingleThreadedExecutor* g_executor = nullptr;

// EPICS CA variables
static chid epics_trigger_chid = nullptr;
static chid epics_trigger_write_chid = nullptr;
static chid epics_start_step_chid = nullptr;
static chid epics_wait_chid = nullptr;
static chid epics_wait_write_chid = nullptr;
static chid epics_holder_chid = nullptr;
static chid epics_stop_chid = nullptr;
static chid epics_current_step_chid = nullptr;
static chid epics_gripper_rbv_chid = nullptr;
static chid epics_gripper_cmd_chid = nullptr;
static chid epics_pause_step_chid = nullptr;
static chid epics_calib_mode_chid = nullptr;
static std::string epics_trigger_pv_name;
static std::string epics_start_step_pv_name;
static std::string epics_wait_pv_name;
static std::string epics_holder_pv_name;
static std::string epics_stop_pv_name;
static std::string epics_current_step_pv_name;
static std::string epics_gripper_rbv_pv_name;
static std::string epics_gripper_cmd_pv_name;
static std::string epics_pause_step_pv_name;
static std::string epics_calib_mode_pv_name;
static bool epics_initialized = false;
static struct ca_client_context* epics_ca_context = nullptr;  // For multi-thread CA access

// Calibration mode values
enum class CalibMode {
  NORMAL = 0,        // Normal full sequence
  HOLDER = 1,        // Holder calibration: 0-5, wait, 20-23
  SAMPLE_HOLDER = 2  // Sample holder calibration: 0-8, wait, 16-23
};

// Gripper state tracking
static std::atomic<int> last_gripper_state{-1};  // -1: unknown, 0: close, 1: open

// Wait PV values
enum class WaitStatus {
  WAITING = 0,    // Keep waiting for measurement
  CONTINUE = 1,   // Continue with remaining steps
  SKIP = 2        // Skip remaining steps, wait for next trigger
};

// Joint names for UR3e + Hand-E
static const std::vector<std::string> ALL_JOINT_NAMES = {
    "robotiq_hande_left_finger_joint",
    "shoulder_pan_joint",
    "wrist_3_joint",
    "wrist_2_joint",
    "wrist_1_joint",
    "elbow_joint",
    "shoulder_lift_joint"
};

static const std::vector<std::string> ARM_JOINT_NAMES = {
    "shoulder_pan_joint",
    "wrist_3_joint",
    "wrist_2_joint",
    "wrist_1_joint",
    "elbow_joint",
    "shoulder_lift_joint"
};

// Waypoint data structure (loaded from YAML)
struct WaypointData {
  std::vector<double> holder1_standby;
  std::vector<double> holder1_on_position;
  std::vector<double> sample_holder_standby;
  std::vector<double> sample_holder_on_position;
  double above_y_offset;
  double retreat_z_offset;
  double holder1_on_x_offset;
  double holder1_on_y_offset;
  double holder1_on_z_offset;
  double sample_holder_on_x_offset;
  double sample_holder_on_y_offset;
  double sample_holder_on_z_offset;
  std::vector<double> holder_multi_x_offsets;
  std::vector<double> holder_multi_z_offsets;
  double wrist3_rotation_offset;  // wrist_3_joint rotation offset (radians), applied to all holder positions
};

// Load waypoints from YAML file
static bool load_waypoints_from_yaml(const std::string& yaml_path, WaypointData& data)
{
  try {
    RCLCPP_INFO(LOGGER, "Loading waypoints from YAML: %s", yaml_path.c_str());
    
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // Navigate to ros__parameters section
    YAML::Node params;
    if (config["/**"] && config["/**"]["ros__parameters"]) {
      params = config["/**"]["ros__parameters"];
    } else if (config["ros__parameters"]) {
      params = config["ros__parameters"];
    } else {
      params = config;  // Try root level
    }
    
    // Load joint positions
    if (params["holder1_standby"]) {
      data.holder1_standby = params["holder1_standby"].as<std::vector<double>>();
    }
    if (params["holder1_on_position"]) {
      data.holder1_on_position = params["holder1_on_position"].as<std::vector<double>>();
    }
    if (params["sample_holder_standby"]) {
      data.sample_holder_standby = params["sample_holder_standby"].as<std::vector<double>>();
    }
    if (params["sample_holder_on_position"]) {
      data.sample_holder_on_position = params["sample_holder_on_position"].as<std::vector<double>>();
    }
    
    // Load offset parameters with defaults
    data.above_y_offset = params["above_y_offset"] ? params["above_y_offset"].as<double>() : -0.005;
    data.retreat_z_offset = params["retreat_z_offset"] ? params["retreat_z_offset"].as<double>() : -0.05;
    
    data.holder1_on_x_offset = params["holder1_on_position_x_offset"] ? params["holder1_on_position_x_offset"].as<double>() : 0.0;
    data.holder1_on_y_offset = params["holder1_on_position_y_offset"] ? params["holder1_on_position_y_offset"].as<double>() : 0.0;
    data.holder1_on_z_offset = params["holder1_on_position_z_offset"] ? params["holder1_on_position_z_offset"].as<double>() : 0.0;
    
    data.sample_holder_on_x_offset = params["sample_holder_on_position_x_offset"] ? params["sample_holder_on_position_x_offset"].as<double>() : 0.0;
    data.sample_holder_on_y_offset = params["sample_holder_on_position_y_offset"] ? params["sample_holder_on_position_y_offset"].as<double>() : 0.0;
    data.sample_holder_on_z_offset = params["sample_holder_on_position_z_offset"] ? params["sample_holder_on_position_z_offset"].as<double>() : 0.0;
    
    // Load multi-holder offsets
    if (params["holder_multi_x_offsets"]) {
      data.holder_multi_x_offsets = params["holder_multi_x_offsets"].as<std::vector<double>>();
    } else {
      data.holder_multi_x_offsets = std::vector<double>(9, 0.0);
    }
    if (params["holder_multi_z_offsets"]) {
      data.holder_multi_z_offsets = params["holder_multi_z_offsets"].as<std::vector<double>>();
    } else {
      data.holder_multi_z_offsets = std::vector<double>(9, 0.0);
    }
    
    // Single wrist_3_joint rotation offset applied to all holder positions
    data.wrist3_rotation_offset = params["wrist3_rotation_offset"] ? params["wrist3_rotation_offset"].as<double>() : 0.0;
    
    RCLCPP_INFO(LOGGER, "✅ Waypoints loaded successfully from YAML");
    RCLCPP_INFO(LOGGER, "  holder1_standby: %zu joints", data.holder1_standby.size());
    RCLCPP_INFO(LOGGER, "  holder1_on_position: %zu joints", data.holder1_on_position.size());
    RCLCPP_INFO(LOGGER, "  sample_holder_standby: %zu joints", data.sample_holder_standby.size());
    RCLCPP_INFO(LOGGER, "  sample_holder_on_position: %zu joints", data.sample_holder_on_position.size());
    RCLCPP_INFO(LOGGER, "  above_y_offset: %.4f", data.above_y_offset);
    RCLCPP_INFO(LOGGER, "  retreat_z_offset: %.4f", data.retreat_z_offset);
    
    return true;
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(LOGGER, "Failed to parse YAML file: %s", e.what());
    return false;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Error loading YAML file: %s", e.what());
    return false;
  }
}

// EPICS CA helper functions
static bool epics_init()
{
  if (epics_initialized) {
    return true;
  }

  int status = ca_context_create(ca_enable_preemptive_callback);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create EPICS CA context: %s", ca_message(status));
    return false;
  }

  // Save context for multi-thread access
  epics_ca_context = ca_current_context();

  epics_initialized = true;
  RCLCPP_INFO(LOGGER, "EPICS CA context created successfully (preemptive callbacks enabled)");
  return true;
}

// Attach current thread to EPICS CA context (must be called from new threads)
static bool epics_attach_context()
{
  if (!epics_ca_context) {
    RCLCPP_ERROR(LOGGER, "EPICS CA context not initialized");
    return false;
  }

  int status = ca_attach_context(epics_ca_context);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to attach to EPICS CA context: %s", ca_message(status));
    return false;
  }
  return true;
}

static void epics_cleanup()
{
  if (epics_trigger_chid) {
    ca_clear_channel(epics_trigger_chid);
    epics_trigger_chid = nullptr;
  }
  if (epics_trigger_write_chid) {
    ca_clear_channel(epics_trigger_write_chid);
    epics_trigger_write_chid = nullptr;
  }
  if (epics_start_step_chid) {
    ca_clear_channel(epics_start_step_chid);
    epics_start_step_chid = nullptr;
  }
  if (epics_wait_chid) {
    ca_clear_channel(epics_wait_chid);
    epics_wait_chid = nullptr;
  }
  if (epics_wait_write_chid) {
    ca_clear_channel(epics_wait_write_chid);
    epics_wait_write_chid = nullptr;
  }
  if (epics_holder_chid) {
    ca_clear_channel(epics_holder_chid);
    epics_holder_chid = nullptr;
  }
  if (epics_stop_chid) {
    ca_clear_channel(epics_stop_chid);
    epics_stop_chid = nullptr;
  }
  if (epics_current_step_chid) {
    ca_clear_channel(epics_current_step_chid);
    epics_current_step_chid = nullptr;
  }
  if (epics_gripper_rbv_chid) {
    ca_clear_channel(epics_gripper_rbv_chid);
    epics_gripper_rbv_chid = nullptr;
  }
  if (epics_gripper_cmd_chid) {
    ca_clear_channel(epics_gripper_cmd_chid);
    epics_gripper_cmd_chid = nullptr;
  }
  if (epics_pause_step_chid) {
    ca_clear_channel(epics_pause_step_chid);
    epics_pause_step_chid = nullptr;
  }
  if (epics_calib_mode_chid) {
    ca_clear_channel(epics_calib_mode_chid);
    epics_calib_mode_chid = nullptr;
  }
  if (epics_initialized) {
    ca_context_destroy();
    epics_initialized = false;
  }
}

static bool epics_connect_pvs(const std::string& trigger_pv, const std::string& start_step_pv,
                               const std::string& wait_pv, const std::string& holder_pv,
                               const std::string& stop_pv, const std::string& current_step_pv,
                               const std::string& gripper_rbv_pv, const std::string& gripper_cmd_pv,
                               const std::string& pause_step_pv, const std::string& calib_mode_pv)
{
  epics_trigger_pv_name = trigger_pv;
  epics_start_step_pv_name = start_step_pv;
  epics_wait_pv_name = wait_pv;
  epics_holder_pv_name = holder_pv;
  epics_stop_pv_name = stop_pv;
  epics_current_step_pv_name = current_step_pv;
  epics_gripper_rbv_pv_name = gripper_rbv_pv;
  epics_gripper_cmd_pv_name = gripper_cmd_pv;
  epics_pause_step_pv_name = pause_step_pv;
  epics_calib_mode_pv_name = calib_mode_pv;

  // Create channel for trigger PV (read)
  int status = ca_create_channel(trigger_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_trigger_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", trigger_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for trigger PV (write)
  status = ca_create_channel(trigger_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_trigger_write_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create write channel for PV '%s': %s", trigger_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for start_step PV (read only)
  status = ca_create_channel(start_step_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_start_step_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", start_step_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for wait PV (read)
  status = ca_create_channel(wait_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_wait_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", wait_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for wait PV (write)
  status = ca_create_channel(wait_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_wait_write_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create write channel for PV '%s': %s", wait_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for holder PV (read only)
  status = ca_create_channel(holder_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_holder_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", holder_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for stop PV (read only)
  status = ca_create_channel(stop_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_stop_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", stop_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for current_step PV (write only)
  status = ca_create_channel(current_step_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_current_step_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", current_step_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for gripper RBV PV (write only - status display)
  status = ca_create_channel(gripper_rbv_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_gripper_rbv_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", gripper_rbv_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for gripper command PV (read - to receive commands)
  status = ca_create_channel(gripper_cmd_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_gripper_cmd_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", gripper_cmd_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for pause_step PV (read only)
  status = ca_create_channel(pause_step_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_pause_step_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", pause_step_pv.c_str(), ca_message(status));
    return false;
  }

  // Create channel for calib_mode PV (read only)
  status = ca_create_channel(calib_mode_pv.c_str(), nullptr, nullptr, CA_PRIORITY_DEFAULT, &epics_calib_mode_chid);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to create channel for PV '%s': %s", calib_mode_pv.c_str(), ca_message(status));
    return false;
  }

  // Wait for all connections
  status = ca_pend_io(5.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to connect to PVs: %s", ca_message(status));
    return false;
  }

  if (ca_state(epics_trigger_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", trigger_pv.c_str());
    return false;
  }

  if (ca_state(epics_start_step_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", start_step_pv.c_str());
    return false;
  }

  if (ca_state(epics_wait_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", wait_pv.c_str());
    return false;
  }

  if (ca_state(epics_holder_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", holder_pv.c_str());
    return false;
  }

  if (ca_state(epics_stop_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", stop_pv.c_str());
    return false;
  }

  if (ca_state(epics_current_step_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", current_step_pv.c_str());
    return false;
  }

  if (ca_state(epics_gripper_rbv_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", gripper_rbv_pv.c_str());
    return false;
  }

  if (ca_state(epics_gripper_cmd_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", gripper_cmd_pv.c_str());
    return false;
  }

  if (ca_state(epics_pause_step_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", pause_step_pv.c_str());
    return false;
  }

  if (ca_state(epics_calib_mode_chid) != cs_conn) {
    RCLCPP_ERROR(LOGGER, "PV '%s' is not connected", calib_mode_pv.c_str());
    return false;
  }

  RCLCPP_INFO(LOGGER, "Connected to EPICS PVs:");
  RCLCPP_INFO(LOGGER, "  Trigger: %s", trigger_pv.c_str());
  RCLCPP_INFO(LOGGER, "  StartStep: %s", start_step_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Wait: %s", wait_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Holder: %s", holder_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Stop: %s", stop_pv.c_str());
  RCLCPP_INFO(LOGGER, "  CurrentStep: %s", current_step_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Gripper_RBV: %s (status)", gripper_rbv_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Gripper: %s (command)", gripper_cmd_pv.c_str());
  RCLCPP_INFO(LOGGER, "  PauseStep: %s", pause_step_pv.c_str());
  RCLCPP_INFO(LOGGER, "  CalibMode: %s", calib_mode_pv.c_str());
  return true;
}

static int epics_read_trigger_pv()
{
  if (!epics_trigger_chid) {
    return -1;
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_trigger_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get trigger PV value: %s", ca_message(status));
    return -1;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return -1;
  }

  return static_cast<int>(value);
}

static int epics_read_start_step_pv()
{
  if (!epics_start_step_chid) {
    return 0;
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_start_step_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get start_step PV value: %s", ca_message(status));
    return 0;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return 0;
  }

  return static_cast<int>(value);
}

// Read Wait PV: 0=waiting, 1=continue, 2=skip remaining steps
static WaitStatus epics_read_wait_pv()
{
  if (!epics_wait_chid) {
    return WaitStatus::CONTINUE;  // Default: continue if PV not connected
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_wait_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get wait PV value: %s", ca_message(status));
    return WaitStatus::CONTINUE;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return WaitStatus::CONTINUE;
  }

  if (value == 0) return WaitStatus::WAITING;
  if (value == 2) return WaitStatus::SKIP;
  return WaitStatus::CONTINUE;  // value == 1 or any other value
}

// Read Holder PV: returns holder number (1-10)
static int epics_read_holder_pv()
{
  if (!epics_holder_chid) {
    return 1;  // Default: holder 1 if PV not connected
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_holder_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get holder PV value: %s", ca_message(status));
    return 1;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return 1;
  }

  int holder = static_cast<int>(value);
  if (holder < 1 || holder > 10) {
    RCLCPP_WARN(LOGGER, "Invalid holder number %d from PV, using 1", holder);
    return 1;
  }
  return holder;
}

// Read Stop PV: returns 1 if stop requested, 0 otherwise
static int epics_read_stop_pv()
{
  if (!epics_stop_chid) {
    return 0;  // Default: no stop if PV not connected
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_stop_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get stop PV value: %s", ca_message(status));
    return 0;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return 0;
  }

  return static_cast<int>(value);
}

// Wait for Stop PV to become 0 (called before each step)
// Returns: true if stop cleared (or not set), false if shutdown requested
static bool wait_for_stop_clear()
{
  int stop_value = epics_read_stop_pv();
  
  if (stop_value == 0) {
    return true;  // Not stopped, proceed
  }

  RCLCPP_INFO(LOGGER, " ");
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "[%s] ⏸️  STOPPED - Waiting for Stop PV to become 0...", get_timestamp().c_str());
  RCLCPP_INFO(LOGGER, "  Stop PV: %s = %d", epics_stop_pv_name.c_str(), stop_value);
  RCLCPP_INFO(LOGGER, "========================================");

  while (rclcpp::ok() && !shutdown_requested) {
    stop_value = epics_read_stop_pv();

    if (stop_value == 0) {
      RCLCPP_INFO(LOGGER, "[%s] ▶️  Stop cleared, resuming execution...", get_timestamp().c_str());
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;  // Shutdown requested
}

// Read PauseStep PV value
static int epics_read_pause_step_pv()
{
  if (!epics_pause_step_chid) {
    return 0;  // Default: no pause if PV not connected
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_pause_step_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get pause_step PV value: %s", ca_message(status));
    return 0;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return 0;
  }

  return static_cast<int>(value);
}

// Read CalibMode PV: 0=normal, 1=holder, 2=sample_holder
static CalibMode epics_read_calib_mode_pv()
{
  if (!epics_calib_mode_chid) {
    return CalibMode::NORMAL;  // Default: normal mode if PV not connected
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_calib_mode_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get calib_mode PV value: %s", ca_message(status));
    return CalibMode::NORMAL;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return CalibMode::NORMAL;
  }

  if (value == 1) return CalibMode::HOLDER;
  if (value == 2) return CalibMode::SAMPLE_HOLDER;
  return CalibMode::NORMAL;
}

// Wait for PauseStep PV to change from current step (called after each step)
// If PauseStep == current_step (and PauseStep != 0), pause until PauseStep changes to a different value
// Returns: true if pause cleared or not matching, false if shutdown requested
static bool wait_for_pause_step_change(int current_step)
{
  int pause_step = epics_read_pause_step_pv();
  
  // PauseStep 0 means no pause
  if (pause_step == 0 || pause_step != current_step) {
    return true;  // Not pausing at this step, proceed
  }

  RCLCPP_INFO(LOGGER, " ");
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "[%s] ⏸️  PAUSED at step %d - Waiting for PauseStep to change...", 
              get_timestamp().c_str(), current_step);
  RCLCPP_INFO(LOGGER, "  PauseStep PV: %s = %d", epics_pause_step_pv_name.c_str(), pause_step);
  RCLCPP_INFO(LOGGER, "  Set PauseStep to a different value to resume");
  RCLCPP_INFO(LOGGER, "========================================");

  while (rclcpp::ok() && !shutdown_requested) {
    pause_step = epics_read_pause_step_pv();

    if (pause_step != current_step) {
      RCLCPP_INFO(LOGGER, "[%s] ▶️  PauseStep changed to %d, resuming execution...", 
                  get_timestamp().c_str(), pause_step);
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return false;  // Shutdown requested
}

// Wait for measurement completion (called after step 12)
// Returns: WaitStatus::CONTINUE to proceed, WaitStatus::SKIP to skip remaining steps
static WaitStatus wait_for_measurement()
{
  RCLCPP_INFO(LOGGER, " ");
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "[%s] Waiting for measurement to complete...", get_timestamp().c_str());
  RCLCPP_INFO(LOGGER, "  Wait PV: %s", epics_wait_pv_name.c_str());
  RCLCPP_INFO(LOGGER, "    0 = Keep waiting");
  RCLCPP_INFO(LOGGER, "    1 = Continue to next steps");
  RCLCPP_INFO(LOGGER, "    2 = Skip remaining steps");
  RCLCPP_INFO(LOGGER, "========================================");

  while (rclcpp::ok() && !shutdown_requested) {
    WaitStatus status = epics_read_wait_pv();

    if (status == WaitStatus::CONTINUE) {
      RCLCPP_INFO(LOGGER, "[%s] Measurement complete, continuing...", get_timestamp().c_str());
      return WaitStatus::CONTINUE;
    } else if (status == WaitStatus::SKIP) {
      RCLCPP_INFO(LOGGER, "Skip requested, aborting remaining steps...");
      return WaitStatus::SKIP;
    }

    // status == WaitStatus::WAITING, keep polling
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return WaitStatus::SKIP;  // Shutdown requested
}

static bool epics_write_pv(int value)
{
  if (!epics_trigger_write_chid) {
    return false;
  }

  dbr_long_t val = static_cast<dbr_long_t>(value);
  int status = ca_put(DBR_LONG, epics_trigger_write_chid, &val);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to put PV value: %s", ca_message(status));
    return false;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete put: %s", ca_message(status));
    return false;
  }

  RCLCPP_INFO(LOGGER, "Reset PV '%s' to %d", epics_trigger_pv_name.c_str(), value);
  return true;
}

// Write to Wait PV
static bool epics_write_wait_pv(int value)
{
  if (!epics_wait_write_chid) {
    return false;
  }

  dbr_long_t val = static_cast<dbr_long_t>(value);
  int status = ca_put(DBR_LONG, epics_wait_write_chid, &val);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to put Wait PV value: %s", ca_message(status));
    return false;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete put: %s", ca_message(status));
    return false;
  }

  RCLCPP_INFO(LOGGER, "Set Wait PV '%s' to %d", epics_wait_pv_name.c_str(), value);
  return true;
}

// Write to CurrentStep PV
static bool epics_write_current_step_pv(int value)
{
  if (!epics_current_step_chid) {
    return false;
  }

  dbr_long_t val = static_cast<dbr_long_t>(value);
  int status = ca_put(DBR_LONG, epics_current_step_chid, &val);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to put CurrentStep PV value: %s", ca_message(status));
    return false;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete put: %s", ca_message(status));
    return false;
  }

  return true;
}

// Write to Gripper RBV PV (0=close, 1=open) - status display
static bool epics_write_gripper_rbv_pv(int value)
{
  if (!epics_gripper_rbv_chid) {
    return false;
  }

  dbr_long_t val = static_cast<dbr_long_t>(value);
  int status = ca_put(DBR_LONG, epics_gripper_rbv_chid, &val);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to put Gripper_RBV PV value: %s", ca_message(status));
    return false;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete put: %s", ca_message(status));
    return false;
  }

  return true;
}

// Read Gripper command PV (0=close, 1=open)
static int epics_read_gripper_cmd_pv()
{
  if (!epics_gripper_cmd_chid) {
    return -1;
  }

  dbr_long_t value = 0;
  int status = ca_get(DBR_LONG, epics_gripper_cmd_chid, &value);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to get Gripper command PV value: %s", ca_message(status));
    return -1;
  }

  status = ca_pend_io(1.0);
  if (status != ECA_NORMAL) {
    RCLCPP_ERROR(LOGGER, "Failed to complete get: %s", ca_message(status));
    return -1;
  }

  return static_cast<int>(value);
}

// Wait for EPICS PV trigger (non-zero value), then reset to 0
// Returns: start_from_step value (>= 0), or -1 if shutdown
// Optional callback is called during wait loop (e.g., for gripper command processing)
static int wait_for_epics_trigger(std::function<void()> idle_callback = nullptr)
{
  RCLCPP_INFO(LOGGER, " ");
  RCLCPP_INFO(LOGGER, "========================================");
  RCLCPP_INFO(LOGGER, "[%s] Waiting for EPICS trigger...", get_timestamp().c_str());
  RCLCPP_INFO(LOGGER, "  Trigger PV: %s (set to non-zero to start)", epics_trigger_pv_name.c_str());
  RCLCPP_INFO(LOGGER, "  StartStep PV: %s (step to start from)", epics_start_step_pv_name.c_str());
  RCLCPP_INFO(LOGGER, "========================================");

  while (rclcpp::ok() && !shutdown_requested) {
    int value = epics_read_trigger_pv();

    if (value > 0) {
      // Read start_step PV
      int start_step = epics_read_start_step_pv();
      RCLCPP_INFO(LOGGER, "[%s] Trigger received! Trigger=%d, StartStep=%d", get_timestamp().c_str(), value, start_step);

      // Reset trigger PV to 0
      if (!epics_write_pv(0)) {
        RCLCPP_WARN(LOGGER, "Failed to reset trigger PV to 0, continuing anyway...");
      }

      return start_step;
    } else if (value < 0) {
      RCLCPP_WARN(LOGGER, "Error reading trigger PV, retrying...");
    }

    // Execute idle callback (e.g., gripper command processing)
    if (idle_callback) {
      idle_callback();
    }

    // Poll every 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return -1;  // Shutdown requested
}

static std::map<std::string, double> joints_from_values(const std::vector<double>& values,
                                                         const std::string& label,
                                                         bool arm_only = true)
{
  const std::vector<std::string>& joint_names = arm_only ? ARM_JOINT_NAMES : ALL_JOINT_NAMES;
  size_t start_idx = arm_only ? 1 : 0;
  size_t expected_size = arm_only ? (ALL_JOINT_NAMES.size()) : ALL_JOINT_NAMES.size();

  if (values.size() != expected_size) {
    throw std::runtime_error(label + ": joint values size mismatch. Expected " +
                             std::to_string(expected_size) + " (all joints), got " +
                             std::to_string(values.size()));
  }

  std::map<std::string, double> joints;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joints[joint_names[i]] = values[start_idx + i];
  }
  return joints;
}

static std::map<std::string, double> apply_cartesian_offset_to_joints(
    const std::map<std::string, double>& original_joints,
    double x_offset,
    double y_offset,
    double z_offset,
    const moveit::core::RobotModelConstPtr& robot_model,
    const std::string& group_name,
    const std::string& ee_link,
    const std::string& label,
    bool z_global = false)
{
  if (std::abs(x_offset) < 1e-6 && std::abs(y_offset) < 1e-6 && std::abs(z_offset) < 1e-6) {
    return original_joints;
  }

  moveit::core::RobotState robot_state(robot_model);
  robot_state.setToDefaultValues();

  for (const auto& joint_pair : original_joints) {
    robot_state.setJointPositions(joint_pair.first, &joint_pair.second);
  }
  robot_state.update();

  const Eigen::Isometry3d& current_pose = robot_state.getGlobalLinkTransform(ee_link);

  Eigen::Isometry3d target_pose;

  if (z_global) {
    Eigen::Isometry3d offset_transform = Eigen::Isometry3d::Identity();
    offset_transform.translation() = Eigen::Vector3d(x_offset, y_offset, 0.0);
    target_pose = current_pose * offset_transform;
    target_pose.translation().z() += z_offset;
  } else {
    Eigen::Isometry3d offset_transform = Eigen::Isometry3d::Identity();
    offset_transform.translation() = Eigen::Vector3d(x_offset, y_offset, z_offset);
    target_pose = current_pose * offset_transform;
  }

  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
  if (!jmg) {
    throw std::runtime_error("Joint model group '" + group_name + "' not found");
  }

  bool ik_success = robot_state.setFromIK(jmg, target_pose, ee_link, 2.0);

  if (!ik_success) {
    RCLCPP_WARN(LOGGER, "%s: IK failed for Cartesian offset, using original joints", label.c_str());
    return original_joints;
  }

  std::map<std::string, double> new_joints;
  for (const auto& joint_pair : original_joints) {
    const auto* joint_value = robot_state.getJointPositions(joint_pair.first);
    if (joint_value) {
      new_joints[joint_pair.first] = *joint_value;
    } else {
      new_joints[joint_pair.first] = joint_pair.second;
    }
  }

  return new_joints;
}


static bool call_gripper_action(rclcpp::Node::SharedPtr node,
                                 const std::string& action_name,
                                 double position,
                                 double max_effort = 100.0)
{
  if (!node || !rclcpp::ok()) {
    RCLCPP_ERROR(LOGGER, "Node or ROS2 context is invalid");
    return false;
  }

  using GripperAction = control_msgs::action::GripperCommand;

  try {
    auto action_client = rclcpp_action::create_client<GripperAction>(node, action_name);

    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(LOGGER, "Gripper action server '%s' not available", action_name.c_str());
      return false;
    }

    auto goal_msg = GripperAction::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = max_effort;

    RCLCPP_INFO(LOGGER, "Sending gripper command: position=%.3f", position);

    auto goal_handle_future = action_client->async_send_goal(goal_msg);

    auto start = std::chrono::steady_clock::now();
    while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (!rclcpp::ok() || shutdown_requested) return false;
      if (std::chrono::steady_clock::now() - start > std::chrono::seconds(5)) return false;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) return false;

    auto result_future = action_client->async_get_result(goal_handle);
    start = std::chrono::steady_clock::now();
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (!rclcpp::ok() || shutdown_requested) return false;
      if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) return false;
    }

    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Exception in call_gripper_action: %s", e.what());
    return false;
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto node = std::make_shared<rclcpp::Node>("epics_triggered_sequence", options);

  // EPICS configuration
  const auto epics_trigger_pv = node->declare_parameter<std::string>("epics_trigger_pv", "Robot:Trigger");
  const auto epics_start_step_pv = node->declare_parameter<std::string>("epics_start_step_pv", "Robot:StartStep");
  const auto epics_wait_pv = node->declare_parameter<std::string>("epics_wait_pv", "Robot:Wait");
  const auto epics_holder_pv = node->declare_parameter<std::string>("epics_holder_pv", "Robot:Holder");
  const auto epics_stop_pv = node->declare_parameter<std::string>("epics_stop_pv", "Robot:Stop");
  const auto epics_current_step_pv = node->declare_parameter<std::string>("epics_current_step_pv", "Robot:CurrentStep");
  const auto epics_gripper_rbv_pv = node->declare_parameter<std::string>("epics_gripper_rbv_pv", "Robot:Gripper_RBV");
  const auto epics_gripper_pv = node->declare_parameter<std::string>("epics_gripper_pv", "Robot:Gripper");
  const auto epics_pause_step_pv = node->declare_parameter<std::string>("epics_pause_step_pv", "Robot:PauseStep");
  const auto epics_calib_mode_pv = node->declare_parameter<std::string>("epics_calib_mode_pv", "Robot:CalibMode");

  RCLCPP_INFO(LOGGER, "EPICS Configuration:");
  RCLCPP_INFO(LOGGER, "  Trigger PV: %s", epics_trigger_pv.c_str());
  RCLCPP_INFO(LOGGER, "  StartStep PV: %s", epics_start_step_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Wait PV: %s (0=wait, 1=continue, 2=skip)", epics_wait_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Holder PV: %s (1-10)", epics_holder_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Stop PV: %s (1=pause, 0=resume)", epics_stop_pv.c_str());
  RCLCPP_INFO(LOGGER, "  CurrentStep PV: %s", epics_current_step_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Gripper_RBV PV: %s (status readback)", epics_gripper_rbv_pv.c_str());
  RCLCPP_INFO(LOGGER, "  Gripper PV: %s (0=close, 1=open - command)", epics_gripper_pv.c_str());
  RCLCPP_INFO(LOGGER, "  PauseStep PV: %s (pause at specific step)", epics_pause_step_pv.c_str());
  RCLCPP_INFO(LOGGER, "  CalibMode PV: %s (0=normal, 1=holder, 2=sample_holder)", epics_calib_mode_pv.c_str());

  // Initialize EPICS CA
  if (!epics_init()) {
    RCLCPP_ERROR(LOGGER, "Failed to initialize EPICS CA");
    rclcpp::shutdown();
    return 1;
  }

  if (!epics_connect_pvs(epics_trigger_pv, epics_start_step_pv, epics_wait_pv, epics_holder_pv, epics_stop_pv, epics_current_step_pv, epics_gripper_rbv_pv, epics_gripper_pv, epics_pause_step_pv, epics_calib_mode_pv)) {
    RCLCPP_ERROR(LOGGER, "Failed to connect to EPICS PVs");
    epics_cleanup();
    rclcpp::shutdown();
    return 1;
  }

  // Get robot description from move_group
  RCLCPP_INFO(LOGGER, "Waiting for move_group parameter service...");
  auto param_client_node = rclcpp::Node::make_shared("epics_triggered_sequence_param_client");
  auto params_client = std::make_shared<rclcpp::SyncParametersClient>(param_client_node, "move_group");
  while (!params_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for move_group param service");
      epics_cleanup();
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(LOGGER, "move_group param service not available, waiting...");
  }

  auto params = params_client->get_parameters({"robot_description", "robot_description_semantic", "robot_description_kinematics"});
  if (params.size() != 3) {
    RCLCPP_ERROR(LOGGER, "Failed to fetch robot_description from move_group");
    epics_cleanup();
    rclcpp::shutdown();
    return 1;
  }
  node->set_parameter(params[0]);
  node->set_parameter(params[1]);

  const auto arm_group = node->declare_parameter<std::string>("arm_group", "ur_arm");

  if (params[2].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    node->set_parameter(params[2]);
  } else {
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver",
                                          "kdl_kinematics_plugin/KDLKinematicsPlugin"));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver_search_resolution", 0.005));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver_timeout", 0.005));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver_attempts", 3));
  }

  // Configuration parameters
  const auto hand_group = node->declare_parameter<std::string>("hand_group", "hand");
  const auto ik_frame = node->declare_parameter<std::string>("ik_frame", "robotiq_hande_end");
  const auto hand_open = node->declare_parameter<std::string>("hand_open", "open");
  const auto hand_close = node->declare_parameter<std::string>("hand_close", "close");

  // Gripper action configuration
  const auto gripper_action_name = node->declare_parameter<std::string>(
      "gripper_action_name", "/gripper_action_controller/gripper_cmd");
  const auto use_gripper_action = node->declare_parameter<bool>("use_gripper_action", false);
  const auto gripper_open_position = node->declare_parameter<double>("gripper_open_position", 0.025);
  const auto gripper_close_position = node->declare_parameter<double>("gripper_close_position", 0.01);
  const auto gripper_max_effort = node->declare_parameter<double>("gripper_max_effort", 100.0);

  // Holder offset parameter (holder number is read from EPICS PV)
  const auto holder_offset = node->declare_parameter<double>("holder_offset", 0.03);

  // MoveGroup action parameters
  const auto movegroup_action_name = node->declare_parameter<std::string>("movegroup_action_name", "/move_action");
  const auto movegroup_tolerance = node->declare_parameter<double>("movegroup_tolerance", 0.0005);
  const auto movegroup_planning_time = node->declare_parameter<double>("movegroup_planning_time", 5.0);
  const auto movegroup_velocity_scale = node->declare_parameter<double>("movegroup_velocity_scale", 1.0);
  const auto movegroup_acceleration_scale = node->declare_parameter<double>("movegroup_acceleration_scale", 1.0);

  // Signal handler
  auto signal_handler = [](int) {
    RCLCPP_WARN(LOGGER, "Shutdown signal received");
    shutdown_requested = true;
    if (g_executor) g_executor->cancel();
    rclcpp::shutdown();
  };
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  // Gripper threshold for open/close detection
  const double gripper_open_threshold = node->declare_parameter<double>("gripper_open_threshold", 0.02);
  
  // Joint state subscription for Cartesian path computation and gripper monitoring
  std::mutex joint_state_mutex;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state;
  auto joint_state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [&latest_joint_state, &joint_state_mutex, gripper_open_threshold](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex);
        latest_joint_state = msg;
        
        // Monitor gripper position and update EPICS PV
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (msg->name[i] == "robotiq_hande_left_finger_joint") {
            double gripper_pos = msg->position[i];
            // Open if position >= threshold, Close otherwise
            int gripper_state = (gripper_pos >= gripper_open_threshold) ? 1 : 0;
            
            // Only update PV if state changed
            int prev_state = last_gripper_state.load();
            if (prev_state != gripper_state) {
              last_gripper_state.store(gripper_state);
              epics_write_gripper_rbv_pv(gripper_state);
              RCLCPP_DEBUG(LOGGER, "Gripper state changed: %s (pos=%.4f)", 
                          gripper_state ? "OPEN" : "CLOSE", gripper_pos);
            }
            break;
          }
        }
      });

  // Start executor thread
  rclcpp::executors::SingleThreadedExecutor executor;
  g_executor = &executor;
  executor.add_node(node);
  std::atomic<bool> executor_running{true};
  std::thread executor_thread([&executor, &executor_running]() {
    while (executor_running && rclcpp::ok() && !shutdown_requested) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // Gripper command monitoring variables
  std::atomic<int> last_gripper_cmd{-1};  // -1: unknown
  std::atomic<int> pending_gripper_cmd{-1};  // -1: no pending command

  // Gripper command monitoring thread (only reads PV and sets pending command)
  std::atomic<bool> gripper_cmd_thread_running{true};
  std::thread gripper_cmd_thread([&gripper_cmd_thread_running, &last_gripper_cmd, &pending_gripper_cmd]() {
    // Attach this thread to EPICS CA context
    if (!epics_attach_context()) {
      RCLCPP_ERROR(LOGGER, "Gripper command thread failed to attach to EPICS CA context");
      return;
    }
    RCLCPP_INFO(LOGGER, "Gripper command monitoring thread attached to EPICS CA context");

    // Wait for system to be ready
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Initialize last command value
    int initial_cmd = epics_read_gripper_cmd_pv();
    if (initial_cmd >= 0) {
      last_gripper_cmd.store(initial_cmd);
      RCLCPP_INFO(LOGGER, "Gripper command PV initialized: %d (%s)",
                  initial_cmd, initial_cmd ? "OPEN" : "CLOSE");
    }

    while (gripper_cmd_thread_running && rclcpp::ok() && !shutdown_requested) {
      int cmd = epics_read_gripper_cmd_pv();
      if (cmd >= 0) {
        int prev_cmd = last_gripper_cmd.load();
        if (prev_cmd >= 0 && cmd != prev_cmd) {
          RCLCPP_INFO(LOGGER, "Gripper command PV changed: %d -> %d (%s)",
                      prev_cmd, cmd, cmd ? "OPEN" : "CLOSE");
          last_gripper_cmd.store(cmd);
          pending_gripper_cmd.store(cmd);  // Set pending command for main thread
        } else if (prev_cmd < 0) {
          last_gripper_cmd.store(cmd);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(LOGGER, "Gripper command monitoring thread stopped");
  });

  // Helper function to execute pending gripper command
  auto execute_pending_gripper_cmd = [&]() -> bool {
    int cmd = pending_gripper_cmd.exchange(-1);  // Get and clear pending command
    if (cmd < 0) return false;  // No pending command

    RCLCPP_INFO(LOGGER, "Executing gripper command: %s", cmd ? "OPEN" : "CLOSE");

    if (use_gripper_action) {
      double position = (cmd == 1) ? gripper_open_position : gripper_close_position;
      if (call_gripper_action(node, gripper_action_name, position, gripper_max_effort)) {
        RCLCPP_INFO(LOGGER, "Gripper %s via action", cmd ? "OPENED" : "CLOSED");
        return true;
      } else {
        RCLCPP_WARN(LOGGER, "Gripper action failed");
        return false;
      }
    } else {
      try {
        moveit::planning_interface::MoveGroupInterface hand_move_group(node, hand_group);
        const std::string& target = (cmd == 1) ? hand_open : hand_close;
        hand_move_group.setNamedTarget(target);
        auto result = hand_move_group.move();
        if (result == moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_INFO(LOGGER, "Gripper %s via MoveIt named target '%s'",
                      cmd ? "OPENED" : "CLOSED", target.c_str());
          return true;
        } else {
          RCLCPP_WARN(LOGGER, "Gripper MoveIt command failed");
          return false;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception in gripper command: %s", e.what());
        return false;
      }
    }
  };

  // YAML file path for waypoints (will be reloaded on each trigger)
  const auto waypoints_yaml_path = node->declare_parameter<std::string>(
      "waypoints_yaml_path", "");
  
  if (waypoints_yaml_path.empty()) {
    RCLCPP_ERROR(LOGGER, "waypoints_yaml_path parameter is required!");
    RCLCPP_ERROR(LOGGER, "Set it via launch file or command line: --ros-args -p waypoints_yaml_path:=/path/to/taught_waypoints.yaml");
    executor_running = false;
    gripper_cmd_thread_running = false;
    executor.cancel();
    if (executor_thread.joinable()) executor_thread.join();
    if (gripper_cmd_thread.joinable()) gripper_cmd_thread.join();
    epics_cleanup();
    rclcpp::shutdown();
    return 1;
  }
  
  RCLCPP_INFO(LOGGER, "Waypoints YAML path: %s", waypoints_yaml_path.c_str());

  try {
    // Load robot model
    RCLCPP_INFO(LOGGER, "Loading robot model...");
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    moveit::core::RobotModelConstPtr robot_model = robot_model_loader.getModel();
    if (!robot_model) {
      RCLCPP_ERROR(LOGGER, "Failed to load robot model");
      epics_cleanup();
      rclcpp::shutdown();
      return 1;
    }

    // Waypoint data (will be loaded from YAML on each trigger)
    WaypointData waypoint_data;
    
    // Computed waypoints (will be recalculated on each trigger)
    std::map<std::string, double> j_holder1_standby_base;
    std::map<std::string, double> j_holder1_on_position_base;
    std::map<std::string, double> j_sample_holder_standby_base;
    std::map<std::string, double> j_sample_holder_on_position_base;
    std::map<std::string, double> j_holder1_above_base;
    std::map<std::string, double> j_holder1_retreat_base;
    std::map<std::string, double> j_sample_holder_above_base;
    std::map<std::string, double> j_sample_holder_retreat_base;
    
    // Lambda to load and calculate waypoints from YAML
    auto reload_waypoints = [&]() -> bool {
      if (!load_waypoints_from_yaml(waypoints_yaml_path, waypoint_data)) {
        RCLCPP_ERROR(LOGGER, "Failed to load waypoints from YAML");
        return false;
      }
      
      // Convert waypoints
      auto j_holder1_standby_taught = joints_from_values(waypoint_data.holder1_standby, "holder1_standby");
      auto j_holder1_on_position_taught = joints_from_values(waypoint_data.holder1_on_position, "holder1_on_position");
      j_sample_holder_standby_base = joints_from_values(waypoint_data.sample_holder_standby, "sample_holder_standby");
      auto j_sample_holder_on_position_taught = joints_from_values(waypoint_data.sample_holder_on_position, "sample_holder_on_position");

      j_holder1_standby_base = apply_cartesian_offset_to_joints(
          j_holder1_standby_taught, waypoint_data.holder1_on_x_offset, waypoint_data.holder1_on_y_offset, waypoint_data.holder1_on_z_offset,
          robot_model, arm_group, ik_frame, "holder1_standby", false);

      j_holder1_on_position_base = apply_cartesian_offset_to_joints(
          j_holder1_on_position_taught, waypoint_data.holder1_on_x_offset, waypoint_data.holder1_on_y_offset, waypoint_data.holder1_on_z_offset,
          robot_model, arm_group, ik_frame, "holder1_on_position", false);

      j_sample_holder_on_position_base = apply_cartesian_offset_to_joints(
          j_sample_holder_on_position_taught, waypoint_data.sample_holder_on_x_offset, waypoint_data.sample_holder_on_y_offset, waypoint_data.sample_holder_on_z_offset,
          robot_model, arm_group, ik_frame, "sample_holder_on_position");

      j_holder1_above_base = apply_cartesian_offset_to_joints(
          j_holder1_on_position_base, 0.0, waypoint_data.above_y_offset, 0.0,
          robot_model, arm_group, ik_frame, "holder1_above", false);

      j_holder1_retreat_base = apply_cartesian_offset_to_joints(
          j_holder1_above_base, 0.0, 0.0, waypoint_data.retreat_z_offset,
          robot_model, arm_group, ik_frame, "holder1_retreat", false);

      j_sample_holder_above_base = apply_cartesian_offset_to_joints(
          j_sample_holder_on_position_base, 0.0, waypoint_data.above_y_offset, 0.0,
          robot_model, arm_group, ik_frame, "sample_holder_above");

      j_sample_holder_retreat_base = apply_cartesian_offset_to_joints(
          j_sample_holder_above_base, 0.0, 0.0, waypoint_data.retreat_z_offset,
          robot_model, arm_group, ik_frame, "sample_holder_retreat");

      RCLCPP_INFO(LOGGER, "✅ Waypoints calculated successfully");
      return true;
    };
    
    // Initial load
    if (!reload_waypoints()) {
      RCLCPP_ERROR(LOGGER, "Failed to load initial waypoints");
      executor_running = false;
      gripper_cmd_thread_running = false;
      executor.cancel();
      if (executor_thread.joinable()) executor_thread.join();
      if (gripper_cmd_thread.joinable()) gripper_cmd_thread.join();
      epics_cleanup();
      rclcpp::shutdown();
      return 1;
    }

    // Create planners
    auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    auto hand_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

    // Helper: Execute MoveGroup action
    auto execute_movegroup_action = [&](int step_number, const std::string& step_name,
                                        const std::map<std::string, double>& goal_joints,
                                        int start_from_step) -> bool {
      if (step_number < start_from_step) {
        RCLCPP_INFO(LOGGER, "Skipping step %d (%s)", step_number, step_name.c_str());
        return true;
      }

      // Check Stop PV before executing step
      if (!wait_for_stop_clear()) {
        return false;  // Shutdown requested
      }

      RCLCPP_INFO(LOGGER, "Step %d: %s", step_number, step_name.c_str());

      if (shutdown_requested) return false;

      try {
        auto movegroup_node = rclcpp::Node::make_shared("movegroup_" + step_name);
        auto movegroup_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
            movegroup_node, movegroup_action_name);

        if (!movegroup_client->wait_for_action_server(std::chrono::seconds(5))) {
          RCLCPP_ERROR(LOGGER, "MoveGroup action server not available");
          return false;
        }

        moveit_msgs::action::MoveGroup::Goal goal;
        goal.request.group_name = arm_group;
        goal.request.num_planning_attempts = 1;
        goal.request.allowed_planning_time = movegroup_planning_time;
        goal.request.max_velocity_scaling_factor = movegroup_velocity_scale;
        goal.request.max_acceleration_scaling_factor = movegroup_acceleration_scale;

        moveit_msgs::msg::Constraints constraints;
        for (const auto& jp : goal_joints) {
          moveit_msgs::msg::JointConstraint jc;
          jc.joint_name = jp.first;
          jc.position = jp.second;
          jc.tolerance_above = movegroup_tolerance;
          jc.tolerance_below = movegroup_tolerance;
          jc.weight = 1.0;
          constraints.joint_constraints.push_back(jc);
        }
        goal.request.goal_constraints.push_back(constraints);
        goal.planning_options.plan_only = false;

        auto goal_handle_future = movegroup_client->async_send_goal(goal);

        auto start = std::chrono::steady_clock::now();
        while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
          if (!rclcpp::ok() || shutdown_requested) return false;
          if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) return false;
          rclcpp::spin_some(movegroup_node);
        }

        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) return false;

        auto result_future = movegroup_client->async_get_result(goal_handle);
        start = std::chrono::steady_clock::now();
        while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
          if (!rclcpp::ok() || shutdown_requested) return false;
          if (std::chrono::steady_clock::now() - start > std::chrono::seconds(120)) return false;
          rclcpp::spin_some(movegroup_node);
        }

        auto wrapped_result = result_future.get();
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            wrapped_result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_INFO(LOGGER, "  -> Completed");
          epics_write_current_step_pv(step_number);  // Update CurrentStep PV
          // Check PauseStep PV - if matches this step, wait until it changes
          if (!wait_for_pause_step_change(step_number)) {
            return false;  // Shutdown requested
          }
          return true;
        }
        RCLCPP_ERROR(LOGGER, "  -> Failed");
        return false;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception: %s", e.what());
        return false;
      }
    };

    // Helper: Execute Cartesian (line) path between two joint states
    auto execute_cartesian_action = [&](int step_number, const std::string& step_name,
                                        const std::map<std::string, double>& goal_joints,
                                        int start_from_step) -> bool {
      if (step_number < start_from_step) {
        RCLCPP_INFO(LOGGER, "Skipping step %d (%s)", step_number, step_name.c_str());
        return true;
      }

      // Check Stop PV before executing step
      if (!wait_for_stop_clear()) {
        return false;  // Shutdown requested
      }

      RCLCPP_INFO(LOGGER, "Step %d: %s (Cartesian)", step_number, step_name.c_str());

      if (shutdown_requested) return false;

      try {
        // Get current robot state from pre-subscribed joint_states
        sensor_msgs::msg::JointState::SharedPtr current_joint_state;
        {
          std::lock_guard<std::mutex> lock(joint_state_mutex);
          current_joint_state = latest_joint_state;
        }
        
        if (!current_joint_state) {
          RCLCPP_ERROR(LOGGER, "No joint state available");
          return false;
        }
        
        moveit::core::RobotState current_state(robot_model);
        current_state.setToDefaultValues();
        
        // Set current state from joint_states
        for (size_t i = 0; i < current_joint_state->name.size(); ++i) {
          const moveit::core::JointModel* jm = robot_model->getJointModel(current_joint_state->name[i]);
          if (jm) {
            current_state.setJointPositions(current_joint_state->name[i], &current_joint_state->position[i]);
          }
        }
        current_state.update();
        
        // Create goal state
        moveit::core::RobotState goal_state(robot_model);
        goal_state.setToDefaultValues();
        goal_state = current_state;  // Copy current state first
        for (const auto& jp : goal_joints) {
          goal_state.setJointPositions(jp.first, &jp.second);
        }
        goal_state.update();
        
        // Get joint model group
        const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(arm_group);
        if (!jmg) {
          RCLCPP_ERROR(LOGGER, "Joint model group '%s' not found", arm_group.c_str());
          return false;
        }
        
        // Get end-effector link
        const moveit::core::LinkModel* ee_link = robot_model->getLinkModel(ik_frame);
        if (!ee_link) {
          RCLCPP_ERROR(LOGGER, "End-effector link '%s' not found", ik_frame.c_str());
          return false;
        }
        
        // Get target pose from goal state
        const Eigen::Isometry3d& target_pose = goal_state.getGlobalLinkTransform(ik_frame);
        
        // Compute Cartesian path
        std::vector<moveit::core::RobotStatePtr> trajectory;
        moveit::core::MaxEEFStep max_step(0.005, 0.05);  // 5mm linear, 0.05 rad rotation
        moveit::core::JumpThreshold jump_threshold(0.0);  // Disable jump threshold
        
        double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
            &current_state, jmg, trajectory, ee_link, target_pose, true, max_step, jump_threshold);
        
        RCLCPP_INFO(LOGGER, "  Cartesian path: %.1f%% computed (%zu waypoints)", fraction * 100.0, trajectory.size());
        
        if (fraction < 0.95) {
          RCLCPP_WARN(LOGGER, "  Cartesian path incomplete (%.1f%%), falling back to joint space", fraction * 100.0);
          // Fall back to regular MoveGroup action
          return execute_movegroup_action(step_number, step_name, goal_joints, start_from_step);
        }
        
        if (trajectory.empty()) {
          RCLCPP_WARN(LOGGER, "  Empty trajectory, skipping");
          return true;
        }
        
        // Create robot trajectory and add time parameterization
        robot_trajectory::RobotTrajectory rt(robot_model, arm_group);
        for (const auto& state : trajectory) {
          rt.addSuffixWayPoint(state, 0.0);
        }
        
        // Time parameterization
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        if (!totg.computeTimeStamps(rt, movegroup_velocity_scale, movegroup_acceleration_scale)) {
          RCLCPP_ERROR(LOGGER, "Failed to compute time stamps");
          return false;
        }
        
        // Convert to message
        moveit_msgs::msg::RobotTrajectory rt_msg;
        rt.getRobotTrajectoryMsg(rt_msg);
        
        // Execute using ExecuteTrajectory action
        auto execute_node = rclcpp::Node::make_shared("execute_cartesian_" + step_name);
        auto execute_client = rclcpp_action::create_client<moveit_msgs::action::ExecuteTrajectory>(
            execute_node, "/execute_trajectory");
        
        if (!execute_client->wait_for_action_server(std::chrono::seconds(5))) {
          RCLCPP_ERROR(LOGGER, "ExecuteTrajectory action server not available");
          return false;
        }
        
        moveit_msgs::action::ExecuteTrajectory::Goal execute_goal;
        execute_goal.trajectory = rt_msg;
        
        auto goal_handle_future = execute_client->async_send_goal(execute_goal);
        
        auto start = std::chrono::steady_clock::now();
        while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
          if (!rclcpp::ok() || shutdown_requested) return false;
          if (std::chrono::steady_clock::now() - start > std::chrono::seconds(10)) return false;
          rclcpp::spin_some(execute_node);
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(LOGGER, "ExecuteTrajectory goal rejected");
          return false;
        }
        
        auto result_future = execute_client->async_get_result(goal_handle);
        start = std::chrono::steady_clock::now();
        while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
          if (!rclcpp::ok() || shutdown_requested) return false;
          if (std::chrono::steady_clock::now() - start > std::chrono::seconds(120)) return false;
          rclcpp::spin_some(execute_node);
        }
        
        auto wrapped_result = result_future.get();
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            wrapped_result.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_INFO(LOGGER, "  -> Completed (Cartesian)");
          epics_write_current_step_pv(step_number);  // Update CurrentStep PV
          // Check PauseStep PV - if matches this step, wait until it changes
          if (!wait_for_pause_step_change(step_number)) {
            return false;  // Shutdown requested
          }
          return true;
        }
        RCLCPP_ERROR(LOGGER, "  -> Failed (error code: %d)", wrapped_result.result->error_code.val);
        return false;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception in Cartesian execution: %s", e.what());
        return false;
      }
    };

    // Helper: Execute hand stage
    auto execute_hand_stage = [&](int step_number, const std::string& step_name,
                                  const std::string& hand_state,
                                  int start_from_step) -> bool {
      if (step_number < start_from_step) {
        RCLCPP_INFO(LOGGER, "Skipping step %d (%s)", step_number, step_name.c_str());
        return true;
      }

      // Check Stop PV before executing step
      if (!wait_for_stop_clear()) {
        return false;  // Shutdown requested
      }

      RCLCPP_INFO(LOGGER, "Step %d: %s", step_number, step_name.c_str());

      if (shutdown_requested) return false;

      if (use_gripper_action) {
        double position = (hand_state == hand_open) ? gripper_open_position : gripper_close_position;
        if (!call_gripper_action(node, gripper_action_name, position, gripper_max_effort)) {
          RCLCPP_WARN(LOGGER, "Gripper action failed, continuing...");
        }
        epics_write_current_step_pv(step_number);  // Update CurrentStep PV
        // Check PauseStep PV - if matches this step, wait until it changes
        if (!wait_for_pause_step_change(step_number)) {
          return false;  // Shutdown requested
        }
        return true;
      } else {
        mtc::Task task;
        task.stages()->setName(step_name);
        task.loadRobotModel(node);
        task.setProperty("group", arm_group);
        task.setProperty("eef", hand_group);
        task.setProperty("ik_frame", ik_frame);
        task.add(std::make_unique<mtc::stages::CurrentState>("current"));

        auto hand_stage = std::make_unique<mtc::stages::MoveTo>(step_name, hand_planner);
        hand_stage->setGroup(hand_group);
        hand_stage->setGoal(hand_state);
        hand_stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
        task.add(std::move(hand_stage));

        try {
          task.init();
          if (!task.plan(5)) return false;
          auto result = task.execute(*task.solutions().front());
          if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) return false;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(LOGGER, "Hand stage error: %s", e.what());
          return false;
        }
        RCLCPP_INFO(LOGGER, "  -> Completed");
        epics_write_current_step_pv(step_number);  // Update CurrentStep PV
        // Check PauseStep PV - if matches this step, wait until it changes
        if (!wait_for_pause_step_change(step_number)) {
          return false;  // Shutdown requested
        }
        return true;
      }
    };

    // Cleanup helper
    auto cleanup_and_exit = [&](int code) -> int {
      executor_running = false;
      gripper_cmd_thread_running = false;
      executor.cancel();
      if (executor_thread.joinable()) executor_thread.join();
      if (gripper_cmd_thread.joinable()) gripper_cmd_thread.join();
      g_executor = nullptr;
      epics_cleanup();
      rclcpp::shutdown();
      return code;
    };

    RCLCPP_INFO(LOGGER, " ");
    RCLCPP_INFO(LOGGER, "========================================");
    RCLCPP_INFO(LOGGER, "EPICS Triggered Sequence Ready");
    RCLCPP_INFO(LOGGER, "========================================");
    RCLCPP_INFO(LOGGER, "  Trigger PV: %s", epics_trigger_pv.c_str());
    RCLCPP_INFO(LOGGER, "  StartStep PV: %s", epics_start_step_pv.c_str());
    RCLCPP_INFO(LOGGER, "  Wait PV: %s (0=wait, 1=continue, 2=skip)", epics_wait_pv.c_str());
    RCLCPP_INFO(LOGGER, "  Holder PV: %s (1-10)", epics_holder_pv.c_str());
    RCLCPP_INFO(LOGGER, "  Stop PV: %s (1=pause, 0=resume)", epics_stop_pv.c_str());
    RCLCPP_INFO(LOGGER, "  CurrentStep PV: %s (updated after each step)", epics_current_step_pv.c_str());
    RCLCPP_INFO(LOGGER, "  Gripper_RBV PV: %s (status, threshold=%.3f)", epics_gripper_rbv_pv.c_str(), gripper_open_threshold);
    RCLCPP_INFO(LOGGER, "  Gripper PV: %s (command: 0=close, 1=open)", epics_gripper_pv.c_str());
    RCLCPP_INFO(LOGGER, "  PauseStep PV: %s (N=pause after step N until changed)", epics_pause_step_pv.c_str());
    RCLCPP_INFO(LOGGER, "  CalibMode PV: %s", epics_calib_mode_pv.c_str());
    RCLCPP_INFO(LOGGER, "    0=Normal (full sequence)");
    RCLCPP_INFO(LOGGER, "    1=Holder calibration (0-5, wait, 20-23)");
    RCLCPP_INFO(LOGGER, "    2=SampleHolder calibration (0-8, wait, 16-23)");
    RCLCPP_INFO(LOGGER, "========================================");

    // Main loop: Wait for trigger -> Execute sequence -> Repeat
    int sequence_count = 0;
    while (rclcpp::ok() && !shutdown_requested) {
      // Wait for EPICS trigger and get start_from_step
      int start_from_step = wait_for_epics_trigger(execute_pending_gripper_cmd);
      if (start_from_step < 0) {
        break;  // Shutdown requested
      }

      sequence_count++;

      // Read holder number from EPICS PV
      int holder_number = epics_read_holder_pv();

      // Read calibration mode from EPICS PV
      CalibMode calib_mode = epics_read_calib_mode_pv();
      const char* calib_mode_str = (calib_mode == CalibMode::HOLDER) ? "Holder" :
                                   (calib_mode == CalibMode::SAMPLE_HOLDER) ? "SampleHolder" : "Normal";

      RCLCPP_INFO(LOGGER, " ");
      RCLCPP_INFO(LOGGER, "========================================");
      RCLCPP_INFO(LOGGER, "[%s] Starting sequence #%d (from step %d, holder %d, mode=%s)",
                  get_timestamp().c_str(), sequence_count, start_from_step, holder_number, calib_mode_str);
      RCLCPP_INFO(LOGGER, "========================================");

      // Reset Wait PV to 0 before starting sequence
      epics_write_wait_pv(0);

      // Reload waypoints from YAML before each sequence
      RCLCPP_INFO(LOGGER, "Reloading waypoints from YAML...");
      if (!reload_waypoints()) {
        RCLCPP_ERROR(LOGGER, "Failed to reload waypoints, skipping sequence");
        continue;
      }

      bool sequence_success = true;
      bool skip_remaining = false;  // Set by Wait PV

      // Apply holder offsets (using values from reloaded YAML)
      double y_offset = (holder_number - 1) * holder_offset;
      double x_offset = 0.0, z_offset = 0.0;
      if (holder_number >= 2 && holder_number <= 10) {
        size_t idx = holder_number - 2;
        if (idx < waypoint_data.holder_multi_x_offsets.size()) x_offset = waypoint_data.holder_multi_x_offsets[idx];
        if (idx < waypoint_data.holder_multi_z_offsets.size()) z_offset = waypoint_data.holder_multi_z_offsets[idx];
      }

      // Lambda to apply wrist_3_joint rotation offset to all holder positions
      double wrist3_offset = waypoint_data.wrist3_rotation_offset;
      auto apply_wrist3_offset = [&](std::map<std::string, double> joints) -> std::map<std::string, double> {
        if (std::abs(wrist3_offset) > 1e-6) {
          auto it = joints.find("wrist_3_joint");
          if (it != joints.end()) {
            it->second += wrist3_offset;
          }
        }
        return joints;
      };
      
      if (std::abs(wrist3_offset) > 1e-6) {
        RCLCPP_INFO(LOGGER, "  Applying wrist_3_joint offset: %.4f rad (%.2f deg)", wrist3_offset, wrist3_offset * 180.0 / M_PI);
      }

      auto temp_standby = j_holder1_standby_base;
      if (holder_number == 10) {
        temp_standby = apply_cartesian_offset_to_joints(
            j_holder1_standby_base, 0.0, -0.005, 0.0,
            robot_model, arm_group, ik_frame, "holder10_standby", false);
      }

      auto j_standby = apply_wrist3_offset(apply_cartesian_offset_to_joints(temp_standby, x_offset, y_offset, z_offset,
          robot_model, arm_group, ik_frame, "standby", false));
      auto j_on_pos = apply_wrist3_offset(apply_cartesian_offset_to_joints(j_holder1_on_position_base, x_offset, y_offset, z_offset,
          robot_model, arm_group, ik_frame, "on_position", false));
      auto j_above = apply_wrist3_offset(apply_cartesian_offset_to_joints(j_on_pos, 0.0, waypoint_data.above_y_offset, 0.0,
          robot_model, arm_group, ik_frame, "above", false));
      auto j_retreat = apply_wrist3_offset(apply_cartesian_offset_to_joints(j_above, 0.0, 0.0, waypoint_data.retreat_z_offset,
          robot_model, arm_group, ik_frame, "retreat", false));

      auto j_sh_standby = j_sample_holder_standby_base;
      auto j_sh_above = j_sample_holder_above_base;
      auto j_sh_on_pos = j_sample_holder_on_position_base;

      // Execute sequence steps
      #define EXEC_ARM(n, name, joints) \
        if (!skip_remaining && !(sequence_success = execute_movegroup_action(n, name, joints, start_from_step))) break
      #define EXEC_CARTESIAN(n, name, joints) \
        if (!skip_remaining && !(sequence_success = execute_cartesian_action(n, name, joints, start_from_step))) break
      #define EXEC_HAND(n, name, state) \
        if (!skip_remaining && !(sequence_success = execute_hand_stage(n, name, state, start_from_step))) break

      // ========================================
      // CALIBRATION MODE: HOLDER (steps 0-5, wait for trigger, 20-23)
      // ========================================
      if (calib_mode == CalibMode::HOLDER) {
        RCLCPP_INFO(LOGGER, ">>> HOLDER CALIBRATION MODE: Steps 0-5, wait, 20-23 <<<");

        // Phase 1: Pick sample from holder and hold at above position (steps 0-5)
        EXEC_HAND(0, "open_hand", hand_open);
        EXEC_ARM(1, "holder_standby", j_standby);
        EXEC_CARTESIAN(2, "holder_above", j_above);
        EXEC_CARTESIAN(3, "holder_on_position", j_on_pos);
        EXEC_HAND(4, "close_gripper", hand_close);
        EXEC_CARTESIAN(5, "holder_above_return", j_above);

        // Wait for next trigger to return sample
        if (sequence_success) {
          RCLCPP_INFO(LOGGER, " ");
          RCLCPP_INFO(LOGGER, "========================================");
          RCLCPP_INFO(LOGGER, "[%s] HOLDER CALIBRATION: Holding at above position", get_timestamp().c_str());
          RCLCPP_INFO(LOGGER, "  Check alignment, then set Trigger=1 to return sample");
          RCLCPP_INFO(LOGGER, "========================================");

          // Wait for next trigger
          int next_trigger = wait_for_epics_trigger(execute_pending_gripper_cmd);
          if (next_trigger < 0) {
            break;  // Shutdown requested
          }

          // Phase 2: Return sample to holder (steps 20-23)
          RCLCPP_INFO(LOGGER, ">>> Returning sample to holder (steps 20-23) <<<");
          EXEC_CARTESIAN(20, "holder_on_position_final", j_on_pos);
          EXEC_HAND(21, "open_gripper_final", hand_open);
          EXEC_CARTESIAN(22, "holder_above_final_return", j_above);
          EXEC_CARTESIAN(23, "holder_standby_final", j_standby);
        }
      }
      // ========================================
      // CALIBRATION MODE: SAMPLE_HOLDER (steps 0-8, wait for trigger, 16-23)
      // ========================================
      else if (calib_mode == CalibMode::SAMPLE_HOLDER) {
        RCLCPP_INFO(LOGGER, ">>> SAMPLE HOLDER CALIBRATION MODE: Steps 0-8, wait, 16-23 <<<");

        // Phase 1: Pick from holder and move to sample holder above (steps 0-8)
        EXEC_HAND(0, "open_hand", hand_open);
        EXEC_ARM(1, "holder_standby", j_standby);
        EXEC_CARTESIAN(2, "holder_above", j_above);
        EXEC_CARTESIAN(3, "holder_on_position", j_on_pos);
        EXEC_HAND(4, "close_gripper", hand_close);
        EXEC_CARTESIAN(5, "holder_above_return", j_above);
        EXEC_CARTESIAN(6, "holder_retreat", j_retreat);
        EXEC_ARM(7, "sample_holder_standby", j_sh_standby);
        EXEC_CARTESIAN(8, "sample_holder_above", j_sh_above);

        // Wait for next trigger to continue
        if (sequence_success) {
          RCLCPP_INFO(LOGGER, " ");
          RCLCPP_INFO(LOGGER, "========================================");
          RCLCPP_INFO(LOGGER, "[%s] SAMPLE HOLDER CALIBRATION: Holding at sample holder above", get_timestamp().c_str());
          RCLCPP_INFO(LOGGER, "  Check alignment, then set Trigger=1 to return sample");
          RCLCPP_INFO(LOGGER, "========================================");

          // Wait for next trigger
          int next_trigger = wait_for_epics_trigger(execute_pending_gripper_cmd);
          if (next_trigger < 0) {
            break;  // Shutdown requested
          }

          // Phase 2: Return sample to holder (steps 16-23)
          RCLCPP_INFO(LOGGER, ">>> Returning sample to holder (steps 16-23) <<<");
          EXEC_CARTESIAN(16, "sample_holder_above_2nd_return", j_sh_above);
          EXEC_CARTESIAN(17, "sample_holder_standby_2nd", j_sh_standby);
          EXEC_ARM(18, "holder_standby_return", j_standby);
          EXEC_CARTESIAN(19, "holder_above_final", j_above);
          EXEC_CARTESIAN(20, "holder_on_position_final", j_on_pos);
          EXEC_HAND(21, "open_gripper_final", hand_open);
          EXEC_CARTESIAN(22, "holder_above_final_return", j_above);
          EXEC_CARTESIAN(23, "holder_standby_final", j_standby);
        }
      }
      // ========================================
      // NORMAL MODE: Full sequence (steps 0-23)
      // ========================================
      else {
        // First sample: pick from holder, place to sample holder
        // Steps 2, 6, 8, 12 use Cartesian (line) path to avoid collision
        EXEC_HAND(0, "open_hand", hand_open);
        EXEC_ARM(1, "holder_standby", j_standby);
        EXEC_CARTESIAN(2, "holder_above", j_above);              // Line: standby -> above
        EXEC_CARTESIAN(3, "holder_on_position", j_on_pos);
        EXEC_HAND(4, "close_gripper", hand_close);
        EXEC_CARTESIAN(5, "holder_above_return", j_above);
        EXEC_CARTESIAN(6, "holder_retreat", j_retreat);          // Line: above -> retreat
        EXEC_ARM(7, "sample_holder_standby", j_sh_standby);
        EXEC_CARTESIAN(8, "sample_holder_above", j_sh_above);    // Line: standby -> above
        EXEC_CARTESIAN(9, "sample_holder_on_position", j_sh_on_pos);
        EXEC_HAND(10, "open_gripper", hand_open);
        EXEC_CARTESIAN(11, "sample_holder_above_return", j_sh_above);
        EXEC_CARTESIAN(12, "sample_holder_standby_return", j_sh_standby);  // Line: above -> standby

        // Wait for measurement after step 12 (before picking up measured sample)
        if (sequence_success && !skip_remaining && start_from_step <= 12) {
          WaitStatus wait_result = wait_for_measurement();
          if (wait_result == WaitStatus::SKIP) {
            RCLCPP_INFO(LOGGER, "Skip requested - skipping remaining steps (13-23)");
            skip_remaining = true;
          }
          // WaitStatus::CONTINUE -> proceed normally
        }

        // Second sample: pick from sample holder, place to holder
        if (!skip_remaining) {
          EXEC_CARTESIAN(13, "sample_holder_above_2nd", j_sh_above);
          EXEC_CARTESIAN(14, "sample_holder_on_position_2nd", j_sh_on_pos);
          EXEC_HAND(15, "close_gripper_2nd", hand_close);
          EXEC_CARTESIAN(16, "sample_holder_above_2nd_return", j_sh_above);
          EXEC_CARTESIAN(17, "sample_holder_standby_2nd", j_sh_standby);
          EXEC_ARM(18, "holder_standby_return", j_standby);
          EXEC_CARTESIAN(19, "holder_above_final", j_above);
          EXEC_CARTESIAN(20, "holder_on_position_final", j_on_pos);
          EXEC_HAND(21, "open_gripper_final", hand_open);
          EXEC_CARTESIAN(22, "holder_above_final_return", j_above);
          EXEC_CARTESIAN(23, "holder_standby_final", j_standby);
        }
      }

      #undef EXEC_ARM
      #undef EXEC_CARTESIAN
      #undef EXEC_HAND

      // Completion messages
      if (calib_mode != CalibMode::NORMAL) {
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "========================================");
        RCLCPP_INFO(LOGGER, "[%s] Calibration sequence #%d completed (%s mode)",
                    get_timestamp().c_str(), sequence_count, calib_mode_str);
        RCLCPP_INFO(LOGGER, "========================================");
      } else if (skip_remaining) {
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "========================================");
        RCLCPP_INFO(LOGGER, "[%s] Sequence #%d: Steps 13-23 skipped (Wait PV = 2)",
                    get_timestamp().c_str(), sequence_count);
        RCLCPP_INFO(LOGGER, "========================================");
      } else if (sequence_success) {
        RCLCPP_INFO(LOGGER, " ");
        RCLCPP_INFO(LOGGER, "========================================");
        RCLCPP_INFO(LOGGER, "[%s] Sequence #%d completed successfully!",
                    get_timestamp().c_str(), sequence_count);
        RCLCPP_INFO(LOGGER, "========================================");
      } else {
        RCLCPP_ERROR(LOGGER, "[%s] Sequence #%d failed!", get_timestamp().c_str(), sequence_count);
      }

      // Reset CurrentStep PV to 0 after sequence completion
      epics_write_current_step_pv(0);
    }

    return cleanup_and_exit(0);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Error: %s", e.what());
    executor_running = false;
    gripper_cmd_thread_running = false;
    executor.cancel();
    if (executor_thread.joinable()) executor_thread.join();
    if (gripper_cmd_thread.joinable()) gripper_cmd_thread.join();
    epics_cleanup();
    rclcpp::shutdown();
    return 1;
  }
}
