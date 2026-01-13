// Sample Load Sequence using MoveIt Task Constructor (MTC)
// Sequential movement through multiple waypoints for sample loading operation
//
// Intended for UR3e + Hand-E:
//   arm_group: ur_arm
//   hand_group: hand (SRDF group states "open"/"close" must exist)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/executors.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <csignal>
#include <sstream>
#include <iostream>
#include <future>
#include <set>

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stage.h>  // For InitStageException
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <control_msgs/srv/query_trajectory_state.hpp>
#include <moveit_task_constructor_msgs/action/execute_task_solution.hpp>

#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <atomic>

namespace mtc = moveit::task_constructor;
using mtc::Stage;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("sample_load_sequence");

// Global variables for step-by-step control
static std::mutex step_mutex;
static std::condition_variable step_cv;
static std::atomic<bool> step_ready{false};
static std::atomic<bool> step_enabled{false};
static std::atomic<bool> shutdown_requested{false};
static rclcpp::executors::SingleThreadedExecutor* g_executor = nullptr;
static rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr g_step_service = nullptr;  // Keep service alive

// Joint names for UR3e + Hand-E (in order from joint_states)
// Note: First joint is gripper, rest are arm joints
static const std::vector<std::string> ALL_JOINT_NAMES = {
    "robotiq_hande_left_finger_joint",  // hand group
    "shoulder_pan_joint",                // ur_arm group
    "wrist_3_joint",                     // ur_arm group
    "wrist_2_joint",                     // ur_arm group
    "wrist_1_joint",                     // ur_arm group
    "elbow_joint",                       // ur_arm group
    "shoulder_lift_joint"                // ur_arm group
};

// Arm-only joint names (for ur_arm group, excluding gripper)
static const std::vector<std::string> ARM_JOINT_NAMES = {
    "shoulder_pan_joint",
    "wrist_3_joint",
    "wrist_2_joint",
    "wrist_1_joint",
    "elbow_joint",
    "shoulder_lift_joint"
};

static std::map<std::string, double> joints_from_values(const std::vector<double>& values,
                                                         const std::string& label,
                                                         bool arm_only = true)
{
  const std::vector<std::string>& joint_names = arm_only ? ARM_JOINT_NAMES : ALL_JOINT_NAMES;
  
  // If arm_only, skip first value (gripper) and use remaining 6 values for arm joints
  size_t start_idx = arm_only ? 1 : 0;
  size_t expected_size = arm_only ? (ALL_JOINT_NAMES.size()) : ALL_JOINT_NAMES.size();
  
  if (values.size() != expected_size) {
    throw std::runtime_error(label + ": joint values size mismatch. Expected " + 
                             std::to_string(expected_size) + " (all joints), got " + 
                             std::to_string(values.size()));
  }
  
  if (arm_only && values.size() < joint_names.size() + 1) {
    throw std::runtime_error(label + ": insufficient values for arm_only mode. Need at least " + 
                             std::to_string(joint_names.size() + 1) + " values, got " + 
                             std::to_string(values.size()));
  }

  std::map<std::string, double> joints;
  for (size_t i = 0; i < joint_names.size(); ++i) {
    joints[joint_names[i]] = values[start_idx + i];
  }
  return joints;
}

static std::unique_ptr<mtc::stages::MoveTo> make_joint_move(const std::string& name,
                                                            const mtc::solvers::PlannerInterfacePtr& planner,
                                                            const std::string& arm_group,
                                                            const std::map<std::string, double>& joints)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(arm_group);
  stage->setGoal(joints);
  stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
  return stage;
}

static std::unique_ptr<mtc::stages::MoveTo> make_hand_named(const std::string& name,
                                                            const mtc::solvers::PlannerInterfacePtr& planner,
                                                            const std::string& hand_group,
                                                            const std::string& named_target)
{
  auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
  stage->setGroup(hand_group);
  stage->setGoal(named_target);  // SRDF group_state e.g. "open", "close"
  stage->restrictDirection(mtc::stages::MoveTo::FORWARD);
  return stage;
}

// Gripper action client helper function
static bool call_gripper_action(rclcpp::Node::SharedPtr node, 
                                 const std::string& action_name,
                                 double position, 
                                 double max_effort = 100.0)
{
  // Check if node and ROS2 are still valid
  if (!node || !rclcpp::ok()) {
    RCLCPP_ERROR(LOGGER, "Node or ROS2 context is invalid, cannot call gripper action");
    return false;
  }
  
  // Check if node's context is valid
  if (!node->get_node_base_interface()) {
    RCLCPP_ERROR(LOGGER, "Node base interface is invalid, cannot call gripper action");
    return false;
  }
  
  using GripperAction = control_msgs::action::GripperCommand;
  
  try {
    auto action_client = rclcpp_action::create_client<GripperAction>(node, action_name);
    
    if (!action_client) {
      RCLCPP_ERROR(LOGGER, "Failed to create gripper action client");
      return false;
    }

    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(LOGGER, "Gripper action server '%s' not available", action_name.c_str());
      return false;
    }

    auto goal_msg = GripperAction::Goal();
    goal_msg.command.position = position;
    goal_msg.command.max_effort = max_effort;

    RCLCPP_INFO(LOGGER, "Sending gripper command: position=%.3f, max_effort=%.1f", position, max_effort);
    
    auto send_goal_options = rclcpp_action::Client<GripperAction>::SendGoalOptions();
    send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<GripperAction>::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(LOGGER, "Gripper action completed successfully");
      } else {
        RCLCPP_WARN(LOGGER, "Gripper action failed with code: %d", static_cast<int>(result.code));
      }
    };

    auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
    
    // Wait for goal to be accepted (using a simple wait loop since executor is already running)
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5);
    while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (!rclcpp::ok() || shutdown_requested) {
        RCLCPP_ERROR(LOGGER, "ROS2 shutdown or interrupt during gripper goal send");
        return false;
      }
      if (std::chrono::steady_clock::now() - start > timeout) {
        RCLCPP_ERROR(LOGGER, "Timeout waiting for gripper goal to be accepted");
        return false;
      }
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER, "Gripper goal was rejected");
      return false;
    }

    // Wait for result
    auto result_future = action_client->async_get_result(goal_handle);
    start = std::chrono::steady_clock::now();
    timeout = std::chrono::seconds(10);
    while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (!rclcpp::ok() || shutdown_requested) {
        RCLCPP_ERROR(LOGGER, "ROS2 shutdown or interrupt during gripper result wait");
        return false;
      }
      if (std::chrono::steady_clock::now() - start > timeout) {
        RCLCPP_ERROR(LOGGER, "Timeout waiting for gripper result");
        return false;
      }
    }

    auto result = result_future.get();
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(LOGGER, "Gripper action succeeded");
      return true;
    } else {
      RCLCPP_WARN(LOGGER, "Gripper action failed with code: %d", static_cast<int>(result.code));
      return false;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Exception in call_gripper_action: %s", e.what());
    return false;
  }
}

// Service callback for step-by-step control
static void step_control_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused
  std::lock_guard<std::mutex> lock(step_mutex);
  step_ready = true;
  step_cv.notify_one();
  response->success = true;
  response->message = "Step proceed signal received";
  RCLCPP_INFO(LOGGER, "✅ Step proceed signal received, continuing...");
}

// Wait for user signal to proceed to next step
// Helper function to get current joint states from /joint_states topic
static std::map<std::string, double> get_current_joint_states(rclcpp::Node::SharedPtr node, 
                                                               const std::vector<std::string>& joint_names,
                                                               std::chrono::seconds timeout = std::chrono::seconds(5))
{
  std::map<std::string, double> current_joints;
  std::promise<sensor_msgs::msg::JointState> promise;
  std::future<sensor_msgs::msg::JointState> future = promise.get_future();
  
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 
    rclcpp::QoS(1).transient_local().reliable(),
    [&promise, &joint_names](const sensor_msgs::msg::JointState::SharedPtr msg) {
      // Check if we have all required joints
      std::set<std::string> required_joints(joint_names.begin(), joint_names.end());
      std::set<std::string> received_joints(msg->name.begin(), msg->name.end());
      
      bool has_all = true;
      for (const auto& joint : required_joints) {
        if (received_joints.find(joint) == received_joints.end()) {
          has_all = false;
          break;
        }
      }
      
      if (has_all && msg->position.size() == msg->name.size()) {
        promise.set_value(*msg);
      }
    });
  
  // Wait for joint states
  auto start = std::chrono::steady_clock::now();
  while (future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (std::chrono::steady_clock::now() - start > timeout) {
      throw std::runtime_error("Timeout waiting for joint_states");
    }
    rclcpp::spin_some(node);
  }
  
  auto joint_state = future.get();
  for (size_t i = 0; i < joint_state.name.size(); ++i) {
    if (std::find(joint_names.begin(), joint_names.end(), joint_state.name[i]) != joint_names.end()) {
      current_joints[joint_state.name[i]] = joint_state.position[i];
    }
  }
  
  return current_joints;
}

static void wait_for_step_proceed(rclcpp::Node::SharedPtr node, const std::string& step_name)
{
  if (!step_enabled) {
    return;  // Step-by-step mode is disabled
  }

  RCLCPP_WARN(LOGGER, "⏸️  Waiting for step proceed signal for: %s", step_name.c_str());
  RCLCPP_WARN(LOGGER, "   Call: ros2 service call /sample_load_sequence/step_proceed std_srvs/srv/Trigger");
  RCLCPP_INFO(LOGGER, "   (Service should be available now)");
  
  std::unique_lock<std::mutex> lock(step_mutex);
  step_ready = false;
  
  // Wait with timeout (300 seconds) - executor thread will process service calls
  auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(300);
  while (!step_ready && rclcpp::ok() && !shutdown_requested) {
    auto now = std::chrono::steady_clock::now();
    if (now >= timeout) {
      RCLCPP_WARN(LOGGER, "Timeout waiting for step proceed signal, continuing anyway...");
      break;
    }
    
    // Wait for condition variable (executor thread will notify when service is called)
    if (step_cv.wait_until(lock, timeout) == std::cv_status::timeout) {
      RCLCPP_WARN(LOGGER, "Timeout waiting for step proceed signal, continuing anyway...");
      break;
    }
    
    if (step_ready || shutdown_requested) {
      break;
    }
  }
  lock.unlock();
  
  if (shutdown_requested) {
    RCLCPP_WARN(LOGGER, "Shutdown requested, aborting step: %s", step_name.c_str());
    return;
  }
  
  RCLCPP_INFO(LOGGER, "▶️  Proceeding to next step: %s", step_name.c_str());
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  auto node = std::make_shared<rclcpp::Node>("sample_load_sequence", options);

  // Get robot description from move_group
  RCLCPP_INFO(LOGGER, "Waiting for move_group parameter service...");
  auto param_client_node = rclcpp::Node::make_shared("sample_load_sequence_param_client");
  auto params_client = std::make_shared<rclcpp::SyncParametersClient>(param_client_node, "move_group");
  while (!params_client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(LOGGER, "Interrupted while waiting for move_group param service");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(LOGGER, "move_group param service not available, waiting again...");
  }

  auto params = params_client->get_parameters({"robot_description", "robot_description_semantic", "robot_description_kinematics"});
  if (params.size() != 3) {
    RCLCPP_ERROR(LOGGER, "Failed to fetch robot_description(_semantic/kinematics) from move_group");
    rclcpp::shutdown();
    return 1;
  }
  node->set_parameter(params[0]);
  node->set_parameter(params[1]);
  
  // Configuration (declare before using in kinematics injection)
  const auto arm_group = node->declare_parameter<std::string>("arm_group", "ur_arm");
  
  if (params[2].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
    node->set_parameter(params[2]);
    RCLCPP_INFO(LOGGER, "Loaded robot_description + robot_description_semantic + robot_description_kinematics from move_group");
  } else {
    RCLCPP_WARN(LOGGER, "move_group has no robot_description_kinematics. Injecting default KDL IK for group '%s'", arm_group.c_str());
    // Minimal IK config for ur_arm (matches ur3e_hande_moveit_config/config/kinematics.yaml)
    // Note: MoveIt reads hierarchical params as dot-separated names.
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver",
                                          "kdl_kinematics_plugin/KDLKinematicsPlugin"));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver_search_resolution", 0.005));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver_timeout", 0.005));
    node->set_parameter(rclcpp::Parameter("robot_description_kinematics." + arm_group + ".kinematics_solver_attempts", 3));
    RCLCPP_INFO(LOGGER, "Injected robot_description_kinematics for '%s'", arm_group.c_str());
  }
  
  // Try to get planning pipeline parameters from move_group
  // PipelinePlanner looks for "ompl.planning_plugin" or falls back to "move_group" namespace
  try {
    // Get all parameters from move_group that start with "ompl" or "planning"
    auto all_params = params_client->list_parameters({}, 0);
    for (const auto& param_name : all_params.names) {
      if (param_name.find("ompl") == 0 || param_name.find("planning_plugin") == 0 || 
          param_name.find("planning_pipelines") == 0) {
        try {
          auto param = params_client->get_parameters({param_name});
          if (!param.empty() && param[0].get_type() != rclcpp::ParameterType::PARAMETER_NOT_SET) {
            node->set_parameter(param[0]);
            RCLCPP_DEBUG(LOGGER, "Copied planning parameter: %s", param_name.c_str());
          }
        } catch (const std::exception& e) {
          RCLCPP_DEBUG(LOGGER, "Could not copy parameter %s: %s", param_name.c_str(), e.what());
        }
      }
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN(LOGGER, "Could not fetch planning pipeline parameters: %s", e.what());
    RCLCPP_WARN(LOGGER, "Will use default planning plugin if available");
  }
  
  // Set default OMPL planning plugin if not set
  if (!node->has_parameter("ompl.planning_plugin")) {
    RCLCPP_WARN(LOGGER, "ompl.planning_plugin not found, setting default");
    node->declare_parameter("ompl.planning_plugin", "ompl_interface/OMPLPlanner");
  }
  const auto hand_group = node->declare_parameter<std::string>("hand_group", "hand");
  const auto ik_frame = node->declare_parameter<std::string>("ik_frame", "robotiq_hande_end");
  const auto hand_open = node->declare_parameter<std::string>("hand_open", "open");
  const auto hand_close = node->declare_parameter<std::string>("hand_close", "close");
  
  // Gripper action configuration
  const auto gripper_action_name = node->declare_parameter<std::string>(
      "gripper_action_name", "/gripper_action_controller/gripper_cmd");
  const auto use_gripper_action = node->declare_parameter<bool>("use_gripper_action", false);
  const auto gripper_open_position = node->declare_parameter<double>("gripper_open_position", 0.025);
  const auto gripper_close_position = node->declare_parameter<double>("gripper_close_position", 0.0);
  const auto gripper_max_effort = node->declare_parameter<double>("gripper_max_effort", 100.0);
  
  // Step-by-step debug mode
  const auto step_by_step = node->declare_parameter<bool>("step_by_step", false);
  step_enabled = step_by_step;

  // Repeat parameters
  const auto num_cycles = node->declare_parameter<int>("num_cycles", 1);
  const auto cycle_delay_seconds = node->declare_parameter<double>("cycle_delay_seconds", 2.0);

  // Use MoveGroup action directly instead of MTC (simpler, more reliable for large motions)
  const auto use_movegroup_action = node->declare_parameter<bool>("use_movegroup_action", true);
  
  // MoveGroup action parameters
  const auto movegroup_action_name = node->declare_parameter<std::string>("movegroup_action_name", "/move_action");
  const auto movegroup_tolerance = node->declare_parameter<double>("movegroup_tolerance", 0.001);
  const auto movegroup_planning_time = node->declare_parameter<double>("movegroup_planning_time", 5.0);
  const auto movegroup_velocity_scale = node->declare_parameter<double>("movegroup_velocity_scale", 1.0);
  const auto movegroup_acceleration_scale = node->declare_parameter<double>("movegroup_acceleration_scale", 1.0);
  
  // Create service for step-by-step control
  // Use absolute service name to ensure it's accessible
  std::string service_name = std::string("/") + node->get_name() + std::string("/step_proceed");
  g_step_service = node->create_service<std_srvs::srv::Trigger>(
      service_name,
      step_control_service_callback);
  
  if (!g_step_service) {
    RCLCPP_ERROR(LOGGER, "Failed to create step_proceed service!");
    rclcpp::shutdown();
    return 1;
  }
  
  RCLCPP_INFO(LOGGER, "✅ Service '%s' created successfully", service_name.c_str());
  
  // Signal handler for graceful shutdown
  auto signal_handler = [](int) {
    RCLCPP_WARN(LOGGER, "Shutdown signal received, stopping...");
    shutdown_requested = true;
    step_ready = true;  // Unblock any waiting steps
    step_cv.notify_all();
    if (g_executor) {
      g_executor->cancel();
    }
    rclcpp::shutdown();
  };
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
  
  // Start executor in separate thread to handle service calls
  rclcpp::executors::SingleThreadedExecutor executor;
  g_executor = &executor;
  executor.add_node(node);
  std::atomic<bool> executor_running{true};
  std::thread executor_thread([&executor, &executor_running, service_name]() {
    RCLCPP_INFO(LOGGER, "Executor thread started, service '%s' should be available", service_name.c_str());
    int spin_count = 0;
    while (executor_running && rclcpp::ok() && !shutdown_requested) {
      executor.spin_once(std::chrono::milliseconds(100));
      spin_count++;
      // Log every 10 seconds (100ms * 100 = 10s) to confirm executor is running
      if (spin_count % 100 == 0) {
        RCLCPP_DEBUG(LOGGER, "Executor thread still running, service '%s' available", service_name.c_str());
      }
    }
    RCLCPP_INFO(LOGGER, "Executor thread stopping...");
  });
  
  // Give executor time to register service
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  if (step_by_step) {
    RCLCPP_WARN(LOGGER, "🔍 Step-by-step debug mode ENABLED");
    RCLCPP_WARN(LOGGER, "   Service name: %s", service_name.c_str());
    RCLCPP_WARN(LOGGER, "   Use: ros2 service call %s std_srvs/srv/Trigger", service_name.c_str());
    RCLCPP_INFO(LOGGER, "   Service is ready and waiting for calls...");
  }

  // Waypoint positions (7 joint values each: gripper + 6 arm joints)
  // Full 2-sample cycle matching bash script

  // 1. holder1 standby
  const auto pos1_holder1_standby = node->declare_parameter<std::vector<double>>(
      "pos1_holder1_standby",
      std::vector<double>{0.003333333333333334, -1.204714600239889, 0.006249474361538887,
                          -1.2075546423541468, -3.4199043713011683, -1.620782494544983, -1.2571996313384552});

  // 2. holder1 above stage
  const auto pos2_holder1_above = node->declare_parameter<std::vector<double>>(
      "pos2_holder1_above",
      std::vector<double>{0.003333333333333334, -1.3276198546039026, 0.004290068056434393,
                          -1.330451790486471, -3.358044763604635, -1.3280060291290283, -1.6114217243590296});

  // 3. holder1 on sample (gripper OPEN)
  const auto pos3_holder1_on_sample = node->declare_parameter<std::vector<double>>(
      "pos3_holder1_on_sample",
      std::vector<double>{0.024705882352941178, -1.3272793928729456, 0.004449322819709778,
                          -1.3298853079425257, -3.333250185052389, -1.3573991060256958, -1.608025690118307});

  // 4. holder1 above stage (return)
  const auto pos4_holder1_above_return = node->declare_parameter<std::vector<double>>(
      "pos4_holder1_above_return",
      std::vector<double>{0.003333333333333334, -1.3276198546039026, 0.004290068056434393,
                          -1.330451790486471, -3.358044763604635, -1.3280060291290283, -1.6114217243590296});

  // 5. holder1 standby (return)
  const auto pos5_holder1_standby_return = node->declare_parameter<std::vector<double>>(
      "pos5_holder1_standby_return",
      std::vector<double>{0.003333333333333334, -1.204714600239889, 0.006249474361538887,
                          -1.2075546423541468, -3.4199043713011683, -1.620782494544983, -1.2571996313384552});

  // 6. sample holder standby 2 (gripper CLOSED with sample)
  const auto pos6_sample_holder_standby2 = node->declare_parameter<std::vector<double>>(
      "pos6_sample_holder_standby2",
      std::vector<double>{0.024705882352941178, -0.17618304887880498, 0.0090628145262599,
                          -1.7623332182513636, -2.3054896793761195, -2.479313373565674, -1.4844883319786568});

  // 7. sample holder above (gripper CLOSED)
  const auto pos7_sample_holder_above = node->declare_parameter<std::vector<double>>(
      "pos7_sample_holder_above",
      std::vector<double>{0.003333333333333334, -0.09968024889101201, 0.007673098240047693,
                          -1.6882298628436487, -2.4419547520079554, -1.847308874130249, -1.9792944393553675});

  // 8. sample holder on position
  const auto pos8_sample_holder_on = node->declare_parameter<std::vector<double>>(
      "pos8_sample_holder_on",
      std::vector<double>{0.003333333333333334, -0.09952193895448858, 0.007664382923394442,
                          -1.6880744139300745, -2.419722696343893, -1.8556915521621704, -1.9931227169432582});

  // 9. sample holder above (return)
  const auto pos9_sample_holder_above_return = node->declare_parameter<std::vector<double>>(
      "pos9_sample_holder_above_return",
      std::vector<double>{0.003333333333333334, -0.09968024889101201, 0.007673098240047693,
                          -1.6882298628436487, -2.4419547520079554, -1.847308874130249, -1.9792944393553675});

  // 10. sample holder standby 2 (return, gripper OPEN)
  const auto pos10_sample_holder_standby2_return = node->declare_parameter<std::vector<double>>(
      "pos10_sample_holder_standby2_return",
      std::vector<double>{0.024705882352941178, -0.17618304887880498, 0.0090628145262599,
                          -1.7623332182513636, -2.3054896793761195, -2.479313373565674, -1.4844883319786568});

  // 11. sample holder above (2nd)
  const auto pos11_sample_holder_above_2nd = node->declare_parameter<std::vector<double>>(
      "pos11_sample_holder_above_2nd",
      std::vector<double>{0.003333333333333334, -0.09968024889101201, 0.007673098240047693,
                          -1.6882298628436487, -2.4419547520079554, -1.847308874130249, -1.9792944393553675});

  // 12. sample holder on position (2nd)
  const auto pos12_sample_holder_on_2nd = node->declare_parameter<std::vector<double>>(
      "pos12_sample_holder_on_2nd",
      std::vector<double>{0.003333333333333334, -0.09952193895448858, 0.007664382923394442,
                          -1.6880744139300745, -2.419722696343893, -1.8556915521621704, -1.9931227169432582});

  // 13. sample holder above (2nd return)
  const auto pos13_sample_holder_above_2nd_return = node->declare_parameter<std::vector<double>>(
      "pos13_sample_holder_above_2nd_return",
      std::vector<double>{0.003333333333333334, -0.09968024889101201, 0.007673098240047693,
                          -1.6882298628436487, -2.4419547520079554, -1.847308874130249, -1.9792944393553675});

  // 14. sample holder standby 2 (2nd return)
  const auto pos14_sample_holder_standby2_2nd_return = node->declare_parameter<std::vector<double>>(
      "pos14_sample_holder_standby2_2nd_return",
      std::vector<double>{0.024705882352941178, -0.17618304887880498, 0.0090628145262599,
                          -1.7623332182513636, -2.3054896793761195, -2.479313373565674, -1.4844883319786568});

  // 15. holder1 standby (go back)
  const auto pos15_holder1_standby_go_back = node->declare_parameter<std::vector<double>>(
      "pos15_holder1_standby_go_back",
      std::vector<double>{0.003333333333333334, -1.204714600239889, 0.006249474361538887,
                          -1.2075546423541468, -3.4199043713011683, -1.620782494544983, -1.2571996313384552});

  // 16. holder1 above stage (final)
  const auto pos16_holder1_above_final = node->declare_parameter<std::vector<double>>(
      "pos16_holder1_above_final",
      std::vector<double>{0.003333333333333334, -1.3276198546039026, 0.004290068056434393,
                          -1.330451790486471, -3.358044763604635, -1.3280060291290283, -1.6114217243590296});

  // 17. holder1 on sample (final, gripper CLOSED with sample)
  const auto pos17_holder1_on_final = node->declare_parameter<std::vector<double>>(
      "pos17_holder1_on_final",
      std::vector<double>{0.024705882352941178, -1.3272793928729456, 0.004449322819709778,
                          -1.3298853079425257, -3.333250185052389, -1.3573991060256958, -1.608025690118307});

  // 18. holder1 above stage (final return)
  const auto pos18_holder1_above_final_return = node->declare_parameter<std::vector<double>>(
      "pos18_holder1_above_final_return",
      std::vector<double>{0.003333333333333334, -1.3276198546039026, 0.004290068056434393,
                          -1.330451790486471, -3.358044763604635, -1.3280060291290283, -1.6114217243590296});

  // 19. holder1 standby (final return)
  const auto pos19_holder1_standby_final_return = node->declare_parameter<std::vector<double>>(
      "pos19_holder1_standby_final_return",
      std::vector<double>{0.003333333333333334, -1.204714600239889, 0.006249474361538887,
                          -1.2075546423541468, -3.4199043713011683, -1.620782494544983, -1.2571996313384552});

  try {
    // Convert to joint maps (arm joints only, skip gripper value)
    auto j1_holder1_standby = joints_from_values(pos1_holder1_standby, "pos1_holder1_standby");
    auto j2_holder1_above = joints_from_values(pos2_holder1_above, "pos2_holder1_above");
    auto j3_holder1_on_sample = joints_from_values(pos3_holder1_on_sample, "pos3_holder1_on_sample");
    auto j4_holder1_above_return = joints_from_values(pos4_holder1_above_return, "pos4_holder1_above_return");
    auto j5_holder1_standby_return = joints_from_values(pos5_holder1_standby_return, "pos5_holder1_standby_return");
    auto j6_sample_holder_standby2 = joints_from_values(pos6_sample_holder_standby2, "pos6_sample_holder_standby2");
    auto j7_sample_holder_above = joints_from_values(pos7_sample_holder_above, "pos7_sample_holder_above");
    auto j8_sample_holder_on = joints_from_values(pos8_sample_holder_on, "pos8_sample_holder_on");
    auto j9_sample_holder_above_return = joints_from_values(pos9_sample_holder_above_return, "pos9_sample_holder_above_return");
    auto j10_sample_holder_standby2_return = joints_from_values(pos10_sample_holder_standby2_return, "pos10_sample_holder_standby2_return");
    auto j11_sample_holder_above_2nd = joints_from_values(pos11_sample_holder_above_2nd, "pos11_sample_holder_above_2nd");
    auto j12_sample_holder_on_2nd = joints_from_values(pos12_sample_holder_on_2nd, "pos12_sample_holder_on_2nd");
    auto j13_sample_holder_above_2nd_return = joints_from_values(pos13_sample_holder_above_2nd_return, "pos13_sample_holder_above_2nd_return");
    auto j14_sample_holder_standby2_2nd_return = joints_from_values(pos14_sample_holder_standby2_2nd_return, "pos14_sample_holder_standby2_2nd_return");
    auto j15_holder1_standby_go_back = joints_from_values(pos15_holder1_standby_go_back, "pos15_holder1_standby_go_back");
    auto j16_holder1_above_final = joints_from_values(pos16_holder1_above_final, "pos16_holder1_above_final");
    auto j17_holder1_on_final = joints_from_values(pos17_holder1_on_final, "pos17_holder1_on_final");
    auto j18_holder1_above_final_return = joints_from_values(pos18_holder1_above_final_return, "pos18_holder1_above_final_return");
    auto j19_holder1_standby_final_return = joints_from_values(pos19_holder1_standby_final_return, "pos19_holder1_standby_final_return");

    // Note: We create a dummy task here for structure, but each execute_single_stage creates its own task
    // This task is not actually used, but we keep it for potential future use
    RCLCPP_INFO(LOGGER, "Creating planners...");
    
    std::shared_ptr<mtc::solvers::PipelinePlanner> sampling_planner;
    try {
      sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
      RCLCPP_INFO(LOGGER, "✅ PipelinePlanner created successfully");
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "Failed to create PipelinePlanner: %s", e.what());
      throw;
    }
    
    auto hand_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    RCLCPP_INFO(LOGGER, "✅ JointInterpolationPlanner created successfully");

    // Helper function to execute using MoveGroup action directly (same as bash script mvan)
    auto execute_movegroup_action = [&](const std::string& step_name, const std::map<std::string, double>& goal_joints) -> bool {
      RCLCPP_INFO(LOGGER, "🔧 Executing step '%s' using MoveGroup action", step_name.c_str());
      wait_for_step_proceed(node, step_name);
      
      try {
        // Use goal_joints directly (same as bash script - no need to read current states)
        RCLCPP_INFO(LOGGER, "Target joint positions:");
        for (const auto& joint_pair : goal_joints) {
          RCLCPP_INFO(LOGGER, "  %s: %.6f", joint_pair.first.c_str(), joint_pair.second);
        }
        
        // Create MoveGroup action client
        auto movegroup_node = rclcpp::Node::make_shared("movegroup_" + step_name);
        auto movegroup_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
            movegroup_node, movegroup_action_name);
        
        RCLCPP_INFO(LOGGER, "Waiting for MoveGroup action server '%s'...", movegroup_action_name.c_str());
        if (!movegroup_client->wait_for_action_server(std::chrono::seconds(5))) {
          RCLCPP_ERROR(LOGGER, "MoveGroup action server '%s' is not available!", movegroup_action_name.c_str());
          return false;
        }
        RCLCPP_INFO(LOGGER, "✅ MoveGroup action server is available");
        
        // Create goal (same as bash script mvan)
        moveit_msgs::action::MoveGroup::Goal goal;
        goal.request.group_name = arm_group;
        goal.request.num_planning_attempts = 1;  // Same as bash script
        goal.request.allowed_planning_time = movegroup_planning_time;
        goal.request.max_velocity_scaling_factor = movegroup_velocity_scale;
        goal.request.max_acceleration_scaling_factor = movegroup_acceleration_scale;
        
        // Set joint constraints (same as bash script)
        moveit_msgs::msg::Constraints constraints;
        for (const auto& joint_pair : goal_joints) {
          moveit_msgs::msg::JointConstraint jc;
          jc.joint_name = joint_pair.first;
          jc.position = joint_pair.second;
          jc.tolerance_above = movegroup_tolerance;
          jc.tolerance_below = movegroup_tolerance;
          jc.weight = 1.0;
          constraints.joint_constraints.push_back(jc);
        }
        goal.request.goal_constraints.push_back(constraints);
        goal.planning_options.plan_only = false;
        goal.planning_options.look_around = false;
        goal.planning_options.replan = false;
        goal.planning_options.replan_attempts = 0;
        goal.planning_options.replan_delay = 0.0;
        
        RCLCPP_INFO(LOGGER, "Sending MoveGroup goal...");
        
        // Send goal
        auto goal_handle_future = movegroup_client->async_send_goal(goal);
        
        // Wait for goal acceptance
        auto start = std::chrono::steady_clock::now();
        auto timeout = std::chrono::seconds(10);
        while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
          if (!rclcpp::ok() || shutdown_requested) {
            RCLCPP_ERROR(LOGGER, "Interrupted while waiting for goal acceptance");
            return false;
          }
          if (std::chrono::steady_clock::now() - start > timeout) {
            RCLCPP_ERROR(LOGGER, "Timeout waiting for goal acceptance");
            return false;
          }
          rclcpp::spin_some(movegroup_node);
        }
        
        auto goal_handle = goal_handle_future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(LOGGER, "Goal was rejected by MoveGroup action server");
          return false;
        }
        
        RCLCPP_INFO(LOGGER, "✅ Goal accepted, waiting for execution result...");
        
        // Wait for result
        auto result_future = movegroup_client->async_get_result(goal_handle);
        start = std::chrono::steady_clock::now();
        timeout = std::chrono::seconds(120);  // Increased timeout for slow motions
        while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
          if (!rclcpp::ok() || shutdown_requested) {
            RCLCPP_ERROR(LOGGER, "Interrupted while waiting for execution result");
            return false;
          }
          if (std::chrono::steady_clock::now() - start > timeout) {
            RCLCPP_ERROR(LOGGER, "Timeout waiting for execution result");
            return false;
          }
          rclcpp::spin_some(movegroup_node);

          // Log progress every 10 seconds
          auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
              std::chrono::steady_clock::now() - start).count();
          if (elapsed > 0 && elapsed % 10 == 0) {
            static int last_logged = 0;
            if (elapsed != last_logged) {
              RCLCPP_INFO(LOGGER, "  Still waiting for result... (%ld seconds elapsed)", elapsed);
              last_logged = elapsed;
            }
          }
        }
        
        auto wrapped_result = result_future.get();
        if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
          auto error_code = wrapped_result.result->error_code.val;
          if (error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
            RCLCPP_INFO(LOGGER, "✅ Execution successful for: %s", step_name.c_str());
            return true;
          } else {
            RCLCPP_ERROR(LOGGER, "Execution failed for step: %s (code: %d)", step_name.c_str(), error_code);
            return false;
          }
        } else {
          RCLCPP_ERROR(LOGGER, "MoveGroup action failed with code: %d", static_cast<int>(wrapped_result.code));
          if (wrapped_result.result) {
            RCLCPP_ERROR(LOGGER, "  MoveIt error code: %d", wrapped_result.result->error_code.val);
          }
          return false;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception during MoveGroup action execution for step '%s': %s", step_name.c_str(), e.what());
        return false;
      }
    };

    // Helper function to execute a single stage task (MTC or MoveGroup action)
    auto execute_single_stage = [&](const std::string& step_name, std::unique_ptr<mtc::stages::MoveTo> stage, const std::map<std::string, double>& goal_joints) -> bool {
      // Use MoveGroup action directly if enabled
      if (use_movegroup_action) {
        return execute_movegroup_action(step_name, goal_joints);
      }
      
      // Otherwise use MTC
      RCLCPP_INFO(LOGGER, "🔧 Setting up task for step: %s", step_name.c_str());
      wait_for_step_proceed(node, step_name);
      
      RCLCPP_INFO(LOGGER, "Creating MTC task for step: %s", step_name.c_str());
      mtc::Task task;
      task.stages()->setName(step_name);
      
      RCLCPP_DEBUG(LOGGER, "Loading robot model for step: %s", step_name.c_str());
      task.loadRobotModel(node);
      
      RCLCPP_DEBUG(LOGGER, "Setting task properties for step: %s", step_name.c_str());
      task.setProperty("group", arm_group);
      task.setProperty("eef", hand_group);
      task.setProperty("ik_frame", ik_frame);
      
      RCLCPP_DEBUG(LOGGER, "Adding stages for step: %s", step_name.c_str());
      task.add(std::make_unique<mtc::stages::CurrentState>("current"));
      RCLCPP_DEBUG(LOGGER, "Added CurrentState stage");
      
      if (!stage) {
        RCLCPP_ERROR(LOGGER, "Stage is null for step: %s", step_name.c_str());
        return false;
      }
      task.add(std::move(stage));
      RCLCPP_DEBUG(LOGGER, "Added MoveTo stage");
      
      RCLCPP_INFO(LOGGER, "Initializing task for step: %s", step_name.c_str());
      try {
        task.init();
        RCLCPP_INFO(LOGGER, "✅ Task initialized successfully for step: %s", step_name.c_str());
      } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "❌ Failed to initialize task for step '" << step_name << "':\n" << e);
        return false;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "❌ Exception during task init for step '%s': %s", step_name.c_str(), e.what());
        return false;
      }
      
      RCLCPP_INFO(LOGGER, "Planning task for step: %s", step_name.c_str());
      try {
        if (!task.plan(5)) {
          RCLCPP_ERROR(LOGGER, "Planning failed for step: %s", step_name.c_str());
          // Try to explain the failure
          std::ostringstream oss;
          if (task.explainFailure(oss)) {
            RCLCPP_ERROR(LOGGER, "Failure explanation: %s", oss.str().c_str());
          }
          return false;
        }
      } catch (const mtc::InitStageException& e) {
        RCLCPP_ERROR_STREAM(LOGGER, "❌ InitStageException during planning for step '" << step_name << "':\n" << e);
        return false;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "❌ Exception during planning for step '%s': %s", step_name.c_str(), e.what());
        return false;
      }
      
      RCLCPP_INFO(LOGGER, "✅ Planning successful for: %s", step_name.c_str());
      task.introspection().publishSolution(*task.solutions().front());
      
      wait_for_step_proceed(node, step_name + "_execute");
      
      // Check if action server is available before executing
      RCLCPP_INFO(LOGGER, "Checking action server availability before execution...");
      
      // Verify action server is available
      auto action_client_node = rclcpp::Node::make_shared("check_action_server_" + step_name);
      auto action_client = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
          action_client_node, "execute_task_solution");
      
      RCLCPP_INFO(LOGGER, "Waiting for execute_task_solution action server...");
      if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(LOGGER, "execute_task_solution action server is not available!");
        RCLCPP_ERROR(LOGGER, "Make sure move_group is running with ExecuteTaskSolutionCapability");
        return false;
      }
      RCLCPP_INFO(LOGGER, "✅ Action server is available");
      
      // Check robot controller status
      RCLCPP_INFO(LOGGER, "Checking robot controller status...");
      auto controller_info_client = node->create_client<control_msgs::srv::QueryTrajectoryState>(
          "/controller_manager/list_controllers");
      if (controller_info_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_INFO(LOGGER, "  Controller manager service available");
      } else {
        RCLCPP_WARN(LOGGER, "  Controller manager service not available (this may be normal)");
      }
      
      // Give a moment for everything to settle and allow planning scene to sync
      RCLCPP_INFO(LOGGER, "Waiting for planning scene to sync...");
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      // Additional wait to ensure robot is ready
      RCLCPP_INFO(LOGGER, "Ensuring robot is ready for execution...");
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      
      RCLCPP_INFO(LOGGER, "Executing task for step: %s", step_name.c_str());
      RCLCPP_INFO(LOGGER, "  Note: If execution fails, check move_group logs for details");
      RCLCPP_INFO(LOGGER, "  Common issues: robot controller not ready, planning scene mismatch, invalid motion plan");
      
      // Instead of using task.execute() directly, use action client to get more detailed error information
      RCLCPP_INFO(LOGGER, "  Preparing to execute solution via action client...");
      
      // Create action client with detailed callbacks
      auto execute_node = rclcpp::Node::make_shared("execute_" + step_name);
      auto execute_client = rclcpp_action::create_client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
          execute_node, "execute_task_solution");
      
      if (!execute_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(LOGGER, "Action server not available for execution");
        return false;
      }
      
      // Convert solution to goal message
      moveit_task_constructor_msgs::action::ExecuteTaskSolution::Goal goal;
      task.solutions().front()->toMsg(goal.solution, &task.introspection());
      
      RCLCPP_INFO(LOGGER, "  Sending goal to action server...");
      
      // Set up goal options with detailed callbacks
      auto send_goal_options = rclcpp_action::Client<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SendGoalOptions();
      
      send_goal_options.feedback_callback = [](const rclcpp_action::ClientGoalHandle<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::SharedPtr&,
                                                const std::shared_ptr<const moveit_task_constructor_msgs::action::ExecuteTaskSolution::Feedback> /*feedback*/) {
        RCLCPP_INFO(LOGGER, "  Execution feedback received");
      };
      
      send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<moveit_task_constructor_msgs::action::ExecuteTaskSolution>::WrappedResult& result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(LOGGER, "  Action succeeded with code: %d", result.result->error_code.val);
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(LOGGER, "  Action was ABORTED");
            RCLCPP_ERROR(LOGGER, "  Error code from result: %d", result.result->error_code.val);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(LOGGER, "  Action was CANCELED");
            RCLCPP_ERROR(LOGGER, "  Error code from result: %d", result.result->error_code.val);
            break;
          case rclcpp_action::ResultCode::UNKNOWN:
            RCLCPP_ERROR(LOGGER, "  Action result is UNKNOWN");
            break;
          default:
            RCLCPP_ERROR(LOGGER, "  Action result code: %d", static_cast<int>(result.code));
            break;
        }
      };
      
      // Send goal
      auto goal_handle_future = execute_client->async_send_goal(goal, send_goal_options);
      
      // Wait for goal to be accepted (using executor to spin)
      auto start = std::chrono::steady_clock::now();
      auto timeout = std::chrono::seconds(10);
      while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        if (!rclcpp::ok() || shutdown_requested) {
          RCLCPP_ERROR(LOGGER, "Interrupted while waiting for goal acceptance");
          return false;
        }
        if (std::chrono::steady_clock::now() - start > timeout) {
          RCLCPP_ERROR(LOGGER, "Timeout waiting for goal acceptance");
          return false;
        }
        // Spin the node to process callbacks
        rclcpp::spin_some(execute_node);
      }
      
      auto goal_handle = goal_handle_future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(LOGGER, "Goal was rejected by action server");
        return false;
      }
      
      RCLCPP_INFO(LOGGER, "  Goal accepted, waiting for result...");
      
      // Wait for result
      auto result_future = execute_client->async_get_result(goal_handle);
      start = std::chrono::steady_clock::now();
      timeout = std::chrono::seconds(30);  // Longer timeout for execution
      while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        if (!rclcpp::ok() || shutdown_requested) {
          RCLCPP_ERROR(LOGGER, "Interrupted while waiting for execution result");
          return false;
        }
        if (std::chrono::steady_clock::now() - start > timeout) {
          RCLCPP_ERROR(LOGGER, "Timeout waiting for execution result");
          return false;
        }
        // Spin the node to process callbacks
        rclcpp::spin_some(execute_node);
      }
      
      auto wrapped_result = result_future.get();
      moveit::core::MoveItErrorCode result;
      
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        result = moveit::core::MoveItErrorCode(wrapped_result.result->error_code);
        RCLCPP_INFO(LOGGER, "  Execution completed successfully with code: %d", result.val);
      } else {
        RCLCPP_ERROR(LOGGER, "  Execution failed with action result code: %d", static_cast<int>(wrapped_result.code));
        if (wrapped_result.result) {
          result = moveit::core::MoveItErrorCode(wrapped_result.result->error_code);
          RCLCPP_ERROR(LOGGER, "  MoveIt error code: %d", result.val);
        } else {
          result = moveit::core::MoveItErrorCode(moveit_msgs::msg::MoveItErrorCodes::FAILURE);
        }
      }
      
      try {
        // Use the result we got from action client
        auto result_code = result.val;
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(LOGGER, "Execution failed for step: %s (code: %d)", step_name.c_str(), result.val);
          // Print error code meaning
          switch (result_code) {
            case moveit_msgs::msg::MoveItErrorCodes::SUCCESS:
              RCLCPP_ERROR(LOGGER, "  Error code: SUCCESS (this shouldn't happen)");
              break;
            case moveit_msgs::msg::MoveItErrorCodes::FAILURE:
              RCLCPP_ERROR(LOGGER, "  Error code: FAILURE - General failure");
              break;
            case moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED:
              RCLCPP_ERROR(LOGGER, "  Error code: PLANNING_FAILED");
              break;
            case moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN:
              RCLCPP_ERROR(LOGGER, "  Error code: INVALID_MOTION_PLAN");
              break;
            case moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED:
              RCLCPP_ERROR(LOGGER, "  Error code: CONTROL_FAILED");
              RCLCPP_ERROR(LOGGER, "  This usually means:");
              RCLCPP_ERROR(LOGGER, "    - Robot controller is not running or not ready");
              RCLCPP_ERROR(LOGGER, "    - Trajectory execution failed");
              RCLCPP_ERROR(LOGGER, "    - Planning scene mismatch with actual robot state");
              RCLCPP_ERROR(LOGGER, "  Check:");
              RCLCPP_ERROR(LOGGER, "    - ros2 control list_controllers");
              RCLCPP_ERROR(LOGGER, "    - move_group logs for detailed error");
              RCLCPP_ERROR(LOGGER, "    - Robot state via: ros2 topic echo /joint_states");
              break;
            case moveit_msgs::msg::MoveItErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA:
              RCLCPP_ERROR(LOGGER, "  Error code: UNABLE_TO_AQUIRE_SENSOR_DATA");
              break;
            case moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT:
              RCLCPP_ERROR(LOGGER, "  Error code: TIMED_OUT");
              break;
            case moveit_msgs::msg::MoveItErrorCodes::PREEMPTED:
              RCLCPP_ERROR(LOGGER, "  Error code: PREEMPTED");
              break;
            default:
              RCLCPP_ERROR(LOGGER, "  Error code: %d (unknown)", result.val);
              break;
          }
          return false;
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(LOGGER, "Exception during task execution for step '%s': %s", step_name.c_str(), e.what());
        return false;
      }
      
      RCLCPP_INFO(LOGGER, "✅ Execution successful for: %s", step_name.c_str());
      return true;
    };

    // Helper function to execute hand stage
    auto execute_hand_stage = [&](const std::string& step_name, const std::string& hand_state) -> bool {
      if (shutdown_requested) {
        return false;
      }
      wait_for_step_proceed(node, step_name);
      if (shutdown_requested) {
        return false;
      }
      
      if (use_gripper_action) {
        double position = (hand_state == hand_open) ? gripper_open_position : gripper_close_position;
        RCLCPP_INFO(LOGGER, "%s gripper via action server...", (hand_state == hand_open) ? "Opening" : "Closing");
        if (!call_gripper_action(node, gripper_action_name, position, gripper_max_effort)) {
          RCLCPP_WARN(LOGGER, "Failed to control gripper via action, continuing anyway...");
        }
        return true;
      } else {
        RCLCPP_INFO(LOGGER, "Creating MTC task for hand stage: %s (state: %s)", step_name.c_str(), hand_state.c_str());
        mtc::Task task;
        task.stages()->setName(step_name);
        
        RCLCPP_DEBUG(LOGGER, "Loading robot model...");
        task.loadRobotModel(node);
        
        RCLCPP_DEBUG(LOGGER, "Setting task properties...");
        task.setProperty("group", arm_group);
        task.setProperty("eef", hand_group);
        task.setProperty("ik_frame", ik_frame);
        
        RCLCPP_DEBUG(LOGGER, "Adding stages...");
        task.add(std::make_unique<mtc::stages::CurrentState>("current"));
        auto hand_stage = make_hand_named(step_name, hand_planner, hand_group, hand_state);
        task.add(std::move(hand_stage));
        
        RCLCPP_DEBUG(LOGGER, "Initializing task...");
        try {
          task.init();
          RCLCPP_DEBUG(LOGGER, "Task initialized successfully");
        } catch (const mtc::InitStageException& e) {
          RCLCPP_ERROR_STREAM(LOGGER, "Failed to initialize task for step '" << step_name << "':\n" << e);
          return false;
        } catch (const std::exception& e) {
          RCLCPP_ERROR(LOGGER, "Exception during task init for step '%s': %s", step_name.c_str(), e.what());
          return false;
        }
        
        RCLCPP_DEBUG(LOGGER, "Planning...");
        if (!task.plan(5)) {
          RCLCPP_ERROR(LOGGER, "Planning failed for step: %s", step_name.c_str());
          return false;
        }
        
        if (shutdown_requested) {
          return false;
        }
        wait_for_step_proceed(node, step_name + "_execute");
        if (shutdown_requested) {
          return false;
        }
        
        RCLCPP_DEBUG(LOGGER, "Executing...");
        auto result = task.execute(*task.solutions().front());
        if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          RCLCPP_ERROR(LOGGER, "Execution failed for step: %s (code: %d)", step_name.c_str(), result.val);
          return false;
        }
        RCLCPP_INFO(LOGGER, "✅ Hand stage '%s' completed successfully", step_name.c_str());
        return true;
      }
    };

    // Full 2-sample cycle matching bash script - Execute each step individually
    RCLCPP_INFO(LOGGER, "Starting sample load sequence (2-sample cycle)...");
    RCLCPP_INFO(LOGGER, "  arm_group: %s", arm_group.c_str());
    RCLCPP_INFO(LOGGER, "  hand_group: %s", hand_group.c_str());
    RCLCPP_INFO(LOGGER, "  ik_frame: %s", ik_frame.c_str());
    RCLCPP_INFO(LOGGER, "  hand_open: %s", hand_open.c_str());
    RCLCPP_INFO(LOGGER, "  hand_close: %s", hand_close.c_str());
    RCLCPP_INFO(LOGGER, "  num_cycles: %d", num_cycles);
    if (num_cycles > 1) {
      RCLCPP_INFO(LOGGER, "  cycle_delay: %.1f seconds", cycle_delay_seconds);
    }

    // Helper lambda for cleanup
    auto cleanup_and_exit = [&](int code) -> int {
      if (shutdown_requested && code != 0) {
        RCLCPP_WARN(LOGGER, "Shutdown requested, exiting gracefully...");
        code = 0;
      }
      executor_running = false;
      executor.cancel();
      if (executor_thread.joinable()) {
        executor_thread.join();
      }
      g_executor = nullptr;
      g_step_service.reset();  // Explicitly reset service
      rclcpp::shutdown();
      return code;
    };

    // Execute cycles
    for (int cycle = 1; cycle <= num_cycles; ++cycle) {
      if (shutdown_requested) {
        RCLCPP_WARN(LOGGER, "Shutdown requested, stopping cycles");
        break;
      }

      if (num_cycles > 1) {
        RCLCPP_INFO(LOGGER, "========================================");
        RCLCPP_INFO(LOGGER, "Starting cycle %d of %d", cycle, num_cycles);
        RCLCPP_INFO(LOGGER, "========================================");
      }

      // === FIRST SAMPLE: Pick from holder1, Place to sample holder ===

      // 0. Open hand
      RCLCPP_INFO(LOGGER, "Step 0: Open hand");
      if (!execute_hand_stage("0_open_hand", hand_open)) {
        return cleanup_and_exit(1);
      }

    // 1. holder1 standby
    RCLCPP_INFO(LOGGER, "Step 1: holder1 standby");
    if (!execute_single_stage("1_holder1_standby", make_joint_move("1_holder1_standby", sampling_planner, arm_group, j1_holder1_standby), j1_holder1_standby)) {
      return cleanup_and_exit(1);
    }

    // 2. holder1 above stage
    RCLCPP_INFO(LOGGER, "Step 2: holder1 above stage");
    if (!execute_single_stage("2_holder1_above", make_joint_move("2_holder1_above", sampling_planner, arm_group, j2_holder1_above), j2_holder1_above)) {
      return cleanup_and_exit(1);
    }

    // 3. holder1 on sample (gripper open)
    RCLCPP_INFO(LOGGER, "Step 3: holder1 on sample");
    if (!execute_single_stage("3_holder1_on_sample", make_joint_move("3_holder1_on_sample", sampling_planner, arm_group, j3_holder1_on_sample), j3_holder1_on_sample)) {
      return cleanup_and_exit(1);
    }

    // 4. Close gripper (grab sample)
    RCLCPP_INFO(LOGGER, "Step 4: Close gripper (grab sample)");
    if (!execute_hand_stage("4_close_gripper", hand_close)) {
      return cleanup_and_exit(1);
    }

    // 5. holder1 above stage (return)
    RCLCPP_INFO(LOGGER, "Step 5: holder1 above stage (return)");
    if (!execute_single_stage("5_holder1_above_return", make_joint_move("5_holder1_above_return", sampling_planner, arm_group, j4_holder1_above_return), j4_holder1_above_return)) {
      return cleanup_and_exit(1);
    }

    // 6. holder1 standby (return)
    RCLCPP_INFO(LOGGER, "Step 6: holder1 standby (return)");
    if (!execute_single_stage("6_holder1_standby_return", make_joint_move("6_holder1_standby_return", sampling_planner, arm_group, j5_holder1_standby_return), j5_holder1_standby_return)) {
      return cleanup_and_exit(1);
    }

    // 7. sample holder standby 2
    RCLCPP_INFO(LOGGER, "Step 7: sample holder standby 2");
    if (!execute_single_stage("7_sample_holder_standby2", make_joint_move("7_sample_holder_standby2", sampling_planner, arm_group, j6_sample_holder_standby2), j6_sample_holder_standby2)) {
      return cleanup_and_exit(1);
    }

    // 8. sample holder above
    RCLCPP_INFO(LOGGER, "Step 8: sample holder above");
    if (!execute_single_stage("8_sample_holder_above", make_joint_move("8_sample_holder_above", sampling_planner, arm_group, j7_sample_holder_above), j7_sample_holder_above)) {
      return cleanup_and_exit(1);
    }

    // 9. sample holder on position
    RCLCPP_INFO(LOGGER, "Step 9: sample holder on position");
    if (!execute_single_stage("9_sample_holder_on", make_joint_move("9_sample_holder_on", sampling_planner, arm_group, j8_sample_holder_on), j8_sample_holder_on)) {
      return cleanup_and_exit(1);
    }

    // 10. Open gripper (release sample)
    RCLCPP_INFO(LOGGER, "Step 10: Open gripper (release sample)");
    if (!execute_hand_stage("10_open_gripper", hand_open)) {
      return cleanup_and_exit(1);
    }

    // 11. sample holder above (return)
    RCLCPP_INFO(LOGGER, "Step 11: sample holder above (return)");
    if (!execute_single_stage("11_sample_holder_above_return", make_joint_move("11_sample_holder_above_return", sampling_planner, arm_group, j9_sample_holder_above_return), j9_sample_holder_above_return)) {
      return cleanup_and_exit(1);
    }

    // 12. sample holder standby 2 (return)
    RCLCPP_INFO(LOGGER, "Step 12: sample holder standby 2 (return)");
    if (!execute_single_stage("12_sample_holder_standby2_return", make_joint_move("12_sample_holder_standby2_return", sampling_planner, arm_group, j10_sample_holder_standby2_return), j10_sample_holder_standby2_return)) {
      return cleanup_and_exit(1);
    }

    // === SECOND SAMPLE: Pick from sample holder, Place to holder1 ===

    // 13. sample holder above (2nd)
    RCLCPP_INFO(LOGGER, "Step 13: sample holder above (2nd)");
    if (!execute_single_stage("13_sample_holder_above_2nd", make_joint_move("13_sample_holder_above_2nd", sampling_planner, arm_group, j11_sample_holder_above_2nd), j11_sample_holder_above_2nd)) {
      return cleanup_and_exit(1);
    }

    // 14. sample holder on position (2nd)
    RCLCPP_INFO(LOGGER, "Step 14: sample holder on position (2nd)");
    if (!execute_single_stage("14_sample_holder_on_2nd", make_joint_move("14_sample_holder_on_2nd", sampling_planner, arm_group, j12_sample_holder_on_2nd), j12_sample_holder_on_2nd)) {
      return cleanup_and_exit(1);
    }

    // 15. Close gripper (grab 2nd sample)
    RCLCPP_INFO(LOGGER, "Step 15: Close gripper (grab 2nd sample)");
    if (!execute_hand_stage("15_close_gripper", hand_close)) {
      return cleanup_and_exit(1);
    }

    // 16. sample holder above (2nd return)
    RCLCPP_INFO(LOGGER, "Step 16: sample holder above (2nd return)");
    if (!execute_single_stage("16_sample_holder_above_2nd_return", make_joint_move("16_sample_holder_above_2nd_return", sampling_planner, arm_group, j13_sample_holder_above_2nd_return), j13_sample_holder_above_2nd_return)) {
      return cleanup_and_exit(1);
    }

    // 17. sample holder standby 2 (2nd return)
    RCLCPP_INFO(LOGGER, "Step 17: sample holder standby 2 (2nd return)");
    if (!execute_single_stage("17_sample_holder_standby2_2nd_return", make_joint_move("17_sample_holder_standby2_2nd_return", sampling_planner, arm_group, j14_sample_holder_standby2_2nd_return), j14_sample_holder_standby2_2nd_return)) {
      return cleanup_and_exit(1);
    }

    // 18. holder1 standby (go back)
    RCLCPP_INFO(LOGGER, "Step 18: holder1 standby (go back)");
    if (!execute_single_stage("18_holder1_standby_go_back", make_joint_move("18_holder1_standby_go_back", sampling_planner, arm_group, j15_holder1_standby_go_back), j15_holder1_standby_go_back)) {
      return cleanup_and_exit(1);
    }

    // 19. holder1 above stage (final)
    RCLCPP_INFO(LOGGER, "Step 19: holder1 above stage (final)");
    if (!execute_single_stage("19_holder1_above_final", make_joint_move("19_holder1_above_final", sampling_planner, arm_group, j16_holder1_above_final), j16_holder1_above_final)) {
      return cleanup_and_exit(1);
    }

    // 20. holder1 on sample (final)
    RCLCPP_INFO(LOGGER, "Step 20: holder1 on sample (final)");
    if (!execute_single_stage("20_holder1_on_final", make_joint_move("20_holder1_on_final", sampling_planner, arm_group, j17_holder1_on_final), j17_holder1_on_final)) {
      return cleanup_and_exit(1);
    }

    // 21. Open gripper (release 2nd sample)
    RCLCPP_INFO(LOGGER, "Step 21: Open gripper (release 2nd sample)");
    if (!execute_hand_stage("21_open_gripper", hand_open)) {
      return cleanup_and_exit(1);
    }

    // 22. holder1 above stage (final return)
    RCLCPP_INFO(LOGGER, "Step 22: holder1 above stage (final return)");
    if (!execute_single_stage("22_holder1_above_final_return", make_joint_move("22_holder1_above_final_return", sampling_planner, arm_group, j18_holder1_above_final_return), j18_holder1_above_final_return)) {
      return cleanup_and_exit(1);
    }

      // 23. holder1 standby (final return)
      RCLCPP_INFO(LOGGER, "Step 23: holder1 standby (final return)");
      if (!execute_single_stage("23_holder1_standby_final_return", make_joint_move("23_holder1_standby_final_return", sampling_planner, arm_group, j19_holder1_standby_final_return), j19_holder1_standby_final_return)) {
        return cleanup_and_exit(1);
      }

      if (num_cycles > 1) {
        RCLCPP_INFO(LOGGER, "✅ Cycle %d of %d completed successfully!", cycle, num_cycles);

        // Wait between cycles (except after last cycle)
        if (cycle < num_cycles) {
          RCLCPP_INFO(LOGGER, "⏸️  Waiting %.1f seconds before next cycle...", cycle_delay_seconds);
          std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(cycle_delay_seconds * 1000)));
        }
      } else {
        RCLCPP_INFO(LOGGER, "✅ All steps completed successfully!");
      }
    } // End of cycle loop

    if (num_cycles > 1) {
      RCLCPP_INFO(LOGGER, "========================================");
      RCLCPP_INFO(LOGGER, "✅ All %d cycles completed successfully!", num_cycles);
      RCLCPP_INFO(LOGGER, "========================================");
    }
    RCLCPP_INFO(LOGGER, "Done");

  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Error: %s", e.what());
    executor_running = false;
    executor.cancel();
    if (executor_thread.joinable()) {
      executor_thread.join();
    }
    g_executor = nullptr;
    g_step_service.reset();  // Explicitly reset service
    rclcpp::shutdown();
    return 1;
  }

  // Stop executor thread
  executor_running = false;
  executor.cancel();
  if (executor_thread.joinable()) {
    executor_thread.join();
  }
  g_executor = nullptr;
  g_step_service.reset();  // Explicitly reset service
  
  rclcpp::shutdown();
  return 0;
}

