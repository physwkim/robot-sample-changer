// Joint space planning test using taught waypoints from multi_holder_sequence
// Tests movement between two joint configurations with upright orientation constraint

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <future>
#include <set>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("upright_constraint_test");

// Joint names for UR3e (arm only, excluding gripper)
static const std::vector<std::string> ARM_JOINT_NAMES = {
    "shoulder_pan_joint",
    "wrist_3_joint",
    "wrist_2_joint",
    "wrist_1_joint",
    "elbow_joint",
    "shoulder_lift_joint"
};

class UprightConstraintTest : public rclcpp::Node
{
public:
  UprightConstraintTest(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("upright_constraint_test", options)
  {
    RCLCPP_INFO(LOGGER, "=== Joint Space Planning Test (with taught waypoints) ===");

    // Sync parameters from move_group (including kinematics)
    syncParametersFromMoveGroup();

    // Load taught waypoints from YAML
    loadTaughtWaypoints();
  }

  void syncParametersFromMoveGroup()
  {
    auto param_client_node = rclcpp::Node::make_shared("param_sync_client");
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(param_client_node, "move_group");

    if (!param_client->wait_for_service(5s)) {
      RCLCPP_ERROR(LOGGER, "move_group parameter service not available!");
      RCLCPP_ERROR(LOGGER, "Make sure move_group is running.");
      return;
    }

    // Get robot_description and robot_description_semantic
    auto base_params = param_client->get_parameters({"robot_description", "robot_description_semantic"});
    for (const auto& param : base_params) {
      declareParameterFromRemote(param);
    }

    // Get kinematics parameters
    auto kinematics_list = param_client->list_parameters({"robot_description_kinematics"}, 10);
    if (!kinematics_list.names.empty()) {
      auto kinematics_params = param_client->get_parameters(kinematics_list.names);
      for (const auto& param : kinematics_params) {
        declareParameterFromRemote(param);
      }
      RCLCPP_INFO(LOGGER, "✓ Synced %zu kinematics parameters from move_group", kinematics_params.size());
    } else {
      RCLCPP_WARN(LOGGER, "No kinematics parameters found in move_group");
    }
  }

  void declareParameterFromRemote(const rclcpp::Parameter& param)
  {
    const auto& name = param.get_name();
    if (has_parameter(name)) {
      return;
    }

    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_STRING:
        declare_parameter<std::string>(name, param.as_string());
        break;
      case rclcpp::ParameterType::PARAMETER_BOOL:
        declare_parameter<bool>(name, param.as_bool());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        declare_parameter<int64_t>(name, param.as_int());
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        declare_parameter<double>(name, param.as_double());
        break;
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        declare_parameter<std::vector<std::string>>(name, param.as_string_array());
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        declare_parameter<std::vector<int64_t>>(name, param.as_integer_array());
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        declare_parameter<std::vector<double>>(name, param.as_double_array());
        break;
      case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
        declare_parameter<std::vector<uint8_t>>(name, param.as_byte_array());
        break;
      default:
        break;
    }
  }

  void loadTaughtWaypoints()
  {
    // Load taught waypoints from YAML (same as multi_holder_sequence)
    // Format: [gripper, shoulder_pan, wrist_3, wrist_2, wrist_1, elbow, shoulder_lift]
    // Note: Parameters are already declared by automatically_declare_parameters_from_overrides
    auto holder1_standby_values = get_parameter("holder1_standby").as_double_array();
    auto sample_holder_standby_values = get_parameter("sample_holder_standby").as_double_array();

    // Convert to joint maps (arm only, skip gripper)
    holder1_standby_ = joints_from_values(holder1_standby_values, "holder1_standby");
    sample_holder_standby_ = joints_from_values(sample_holder_standby_values, "sample_holder_standby");

    RCLCPP_INFO(LOGGER, "Loaded taught waypoints:");
    RCLCPP_INFO(LOGGER, "  - holder1_standby");
    RCLCPP_INFO(LOGGER, "  - sample_holder_standby");
  }

  std::map<std::string, double> joints_from_values(const std::vector<double>& values, const std::string& label)
  {
    // Skip first value (gripper) and use remaining 6 values for arm joints
    if (values.size() != 7) {
      throw std::runtime_error(label + ": expected 7 joint values (gripper + 6 arm), got " + std::to_string(values.size()));
    }

    std::map<std::string, double> joints;
    for (size_t i = 0; i < ARM_JOINT_NAMES.size(); ++i) {
      joints[ARM_JOINT_NAMES[i]] = values[i + 1];  // Skip gripper (index 0)
    }
    return joints;
  }

  // Get current joint states from /joint_states topic (all joints)
  std::map<std::string, double> get_current_joint_states(bool arm_only = true)
  {
    RCLCPP_INFO(LOGGER, "Reading current joint states from /joint_states...");

    // Create a simple synchronous subscriber
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg;
    auto subscription = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states",
        rclcpp::QoS(10),
        [&joint_state_msg](const sensor_msgs::msg::JointState::SharedPtr msg) {
          joint_state_msg = msg;
        });

    // Spin until we receive a message
    auto start = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5);

    while (!joint_state_msg && rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(std::chrono::milliseconds(10));

      if (std::chrono::steady_clock::now() - start > timeout) {
        RCLCPP_ERROR(LOGGER, "Timeout waiting for /joint_states");
        RCLCPP_ERROR(LOGGER, "Check: ros2 topic echo /joint_states --once");
        throw std::runtime_error("Timeout waiting for joint_states");
      }
    }

    if (!joint_state_msg) {
      throw std::runtime_error("Failed to receive joint_states");
    }

    // Extract joint positions
    std::map<std::string, double> current_joints;
    if (arm_only) {
      // Only arm joints
      for (size_t i = 0; i < joint_state_msg->name.size(); ++i) {
        if (std::find(ARM_JOINT_NAMES.begin(), ARM_JOINT_NAMES.end(), joint_state_msg->name[i]) != ARM_JOINT_NAMES.end()) {
          current_joints[joint_state_msg->name[i]] = joint_state_msg->position[i];
        }
      }

      // Verify we have all required joints
      if (current_joints.size() != ARM_JOINT_NAMES.size()) {
        RCLCPP_ERROR(LOGGER, "Missing joints! Got %zu, expected %zu",
                     current_joints.size(), ARM_JOINT_NAMES.size());
        throw std::runtime_error("Incomplete joint state data");
      }
    } else {
      // All joints
      for (size_t i = 0; i < joint_state_msg->name.size(); ++i) {
        current_joints[joint_state_msg->name[i]] = joint_state_msg->position[i];
      }
    }

    RCLCPP_INFO(LOGGER, "✅ Successfully read %zu joint positions", current_joints.size());
    return current_joints;
  }

  // Apply Cartesian offset to joint positions using FK/IK
  // (copied from multi_holder_sequence)
  std::map<std::string, double> apply_cartesian_offset_to_joints(
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
    // Check if any offset is applied
    if (std::abs(x_offset) < 1e-6 && std::abs(y_offset) < 1e-6 && std::abs(z_offset) < 1e-6) {
      return original_joints;
    }

    // Create robot state
    moveit::core::RobotState robot_state(robot_model);
    robot_state.setToDefaultValues();

    // Set current joint values
    for (const auto& joint_pair : original_joints) {
      robot_state.setJointPositions(joint_pair.first, &joint_pair.second);
    }
    robot_state.update();

    // Get current end-effector pose using FK
    const Eigen::Isometry3d& current_pose = robot_state.getGlobalLinkTransform(ee_link);

    Eigen::Isometry3d target_pose;

    if (z_global) {
      // Apply Z offset in GLOBAL FRAME, X and Y in LOCAL FRAME
      Eigen::Isometry3d offset_transform = Eigen::Isometry3d::Identity();
      offset_transform.translation() = Eigen::Vector3d(x_offset, y_offset, 0.0);
      target_pose = current_pose * offset_transform;

      // Then apply global Z offset
      target_pose.translation().z() += z_offset;

      RCLCPP_INFO(LOGGER, "%s: Applying Cartesian offset [X:%.4f(local), Y:%.4f(local), Z:%.4f(GLOBAL)]m",
                   label.c_str(), x_offset, y_offset, z_offset);
    } else {
      // Apply Cartesian offset in END-EFFECTOR LOCAL FRAME
      Eigen::Isometry3d offset_transform = Eigen::Isometry3d::Identity();
      offset_transform.translation() = Eigen::Vector3d(x_offset, y_offset, z_offset);

      target_pose = current_pose * offset_transform;

      // Calculate global offset for debugging
      Eigen::Vector3d global_offset = target_pose.translation() - current_pose.translation();

      RCLCPP_INFO(LOGGER, "%s: Applying Cartesian offset [%.4f, %.4f, %.4f]m (in end-effector local frame)",
                   label.c_str(), x_offset, y_offset, z_offset);
      RCLCPP_INFO(LOGGER, "  Local offset [X:%.4f Y:%.4f Z:%.4f] -> Global offset [X:%.4f Y:%.4f Z:%.4f]",
                   x_offset, y_offset, z_offset,
                   global_offset.x(), global_offset.y(), global_offset.z());
    }

    RCLCPP_INFO(LOGGER, "  Original global pos: [%.4f, %.4f, %.4f] -> Target global pos: [%.4f, %.4f, %.4f]",
                 current_pose.translation().x(), current_pose.translation().y(), current_pose.translation().z(),
                 target_pose.translation().x(), target_pose.translation().y(), target_pose.translation().z());

    // Compute IK for new pose
    const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group_name);
    if (!jmg) {
      throw std::runtime_error("Joint model group '" + group_name + "' not found");
    }

    bool ik_success = robot_state.setFromIK(jmg, target_pose, ee_link, 2.0);

    if (!ik_success) {
      RCLCPP_WARN(LOGGER, "%s: IK failed for Cartesian offset, using original joints", label.c_str());
      return original_joints;
    }

    // Extract new joint values
    std::map<std::string, double> new_joints;
    for (const auto& joint_pair : original_joints) {
      const auto* joint_value = robot_state.getJointPositions(joint_pair.first);
      if (joint_value) {
        new_joints[joint_pair.first] = *joint_value;
      } else {
        new_joints[joint_pair.first] = joint_pair.second;
      }
    }

    RCLCPP_INFO(LOGGER, "✅ %s: Successfully applied Cartesian offset via IK", label.c_str());
    return new_joints;
  }

  // Helper function to execute using MoveGroup action
  // goal_joints should only contain arm joints, gripper joints will be read from current state
  bool execute_movegroup_action(const std::string& step_name, const std::map<std::string, double>& goal_joints)
  {
    RCLCPP_INFO(LOGGER, "🔧 Executing step '%s' using MoveGroup action", step_name.c_str());

    try {
      // Log target joint positions
      RCLCPP_INFO(LOGGER, "Target joint positions:");
      for (const auto& joint_pair : goal_joints) {
        RCLCPP_INFO(LOGGER, "  %s: %.6f", joint_pair.first.c_str(), joint_pair.second);
      }

      // Create MoveGroup action client
      auto movegroup_node = rclcpp::Node::make_shared("movegroup_" + step_name);
      auto movegroup_client = rclcpp_action::create_client<moveit_msgs::action::MoveGroup>(
          movegroup_node, "/move_action");

      RCLCPP_INFO(LOGGER, "Waiting for MoveGroup action server '/move_action'...");
      if (!movegroup_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(LOGGER, "MoveGroup action server '/move_action' is not available!");
        return false;
      }
      RCLCPP_INFO(LOGGER, "✅ MoveGroup action server is available");

      // Create goal
      moveit_msgs::action::MoveGroup::Goal goal;
      goal.request.group_name = "ur_manipulator";  // Changed from ur_arm to include gripper in collision checking
      goal.request.num_planning_attempts = 20;  // Increased for better obstacle avoidance
      goal.request.allowed_planning_time = 15.0;  // Increased planning time for complex paths
      goal.request.max_velocity_scaling_factor = 1.0;
      goal.request.max_acceleration_scaling_factor = 1.0;
      
      // Set planner ID for better obstacle avoidance (RRTConnect is good for avoiding obstacles)
      // If planner_id field exists, uncomment the following line:
      // goal.request.planner_id = "RRTConnectkConfigDefault";

      RCLCPP_INFO(LOGGER, "Planning with %d attempts, %.1f seconds timeout",
                  goal.request.num_planning_attempts, goal.request.allowed_planning_time);
      RCLCPP_INFO(LOGGER, "Using look_around=true and replan=true for obstacle avoidance");

      // Get current gripper joint positions (to keep gripper fixed)
      RCLCPP_INFO(LOGGER, "Reading current gripper joint positions...");
      auto all_joints = get_current_joint_states(false);  // Get ALL joints including gripper

      // Set joint constraints
      moveit_msgs::msg::Constraints constraints;

      // Add arm joint constraints
      for (const auto& joint_pair : goal_joints) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = joint_pair.first;
        jc.position = joint_pair.second;
        jc.tolerance_above = 0.174;  // 10 degrees for maximum planning flexibility
        jc.tolerance_below = 0.174;
        jc.weight = 1.0;
        constraints.joint_constraints.push_back(jc);
      }

      // Add gripper joint constraints (keep gripper fixed)
      // Gripper joint name for HandE: "robotiq_hande_left_finger_joint"
      if (all_joints.find("robotiq_hande_left_finger_joint") != all_joints.end()) {
        moveit_msgs::msg::JointConstraint jc;
        jc.joint_name = "robotiq_hande_left_finger_joint";
        jc.position = all_joints["robotiq_hande_left_finger_joint"];
        jc.tolerance_above = 0.001;  // Very tight tolerance to keep gripper fixed
        jc.tolerance_below = 0.001;
        jc.weight = 1.0;
        constraints.joint_constraints.push_back(jc);
        RCLCPP_INFO(LOGGER, "Added gripper constraint: robotiq_hande_left_finger_joint = %.4f", jc.position);
      } else {
        RCLCPP_WARN(LOGGER, "Gripper joint 'robotiq_hande_left_finger_joint' not found in current joint states");
        // List available joints for debugging
        RCLCPP_INFO(LOGGER, "Available joints:");
        for (const auto& j : all_joints) {
          RCLCPP_INFO(LOGGER, "  %s: %.4f", j.first.c_str(), j.second);
        }
      }

      goal.request.goal_constraints.push_back(constraints);

      // Add upright orientation constraint for entire path
      moveit_msgs::msg::OrientationConstraint oc;
      oc.link_name = "robotiq_hande_end";  // End effector link
      oc.header.frame_id = "base_link";

      // Target orientation: upright (Z-axis pointing up)
      // This is identity quaternion - end effector Z aligned with world Z
      oc.orientation.x = 0.0;
      oc.orientation.y = 0.0;
      oc.orientation.z = 0.0;
      oc.orientation.w = 1.0;

      // Tolerance: allow free rotation around Z-axis, but keep upright
      oc.absolute_x_axis_tolerance = 0.1;  // ±0.1 rad (~5.7°) tilt around X
      oc.absolute_y_axis_tolerance = 0.1;  // ±0.1 rad (~5.7°) tilt around Y
      oc.absolute_z_axis_tolerance = 3.14; // Free rotation around Z (upright axis)
      oc.weight = 1.0;

      // Add to PATH constraints (not goal constraints) to maintain throughout motion
      moveit_msgs::msg::Constraints path_constraints;
      path_constraints.orientation_constraints.push_back(oc);
      goal.request.path_constraints = path_constraints;

      RCLCPP_INFO(LOGGER, "Added upright orientation constraint:");
      RCLCPP_INFO(LOGGER, "  - X/Y tilt tolerance: ±0.1 rad (±5.7°)");
      RCLCPP_INFO(LOGGER, "  - Z rotation: free (±3.14 rad)");
      RCLCPP_INFO(LOGGER, "Goal tolerance: ±0.174 rad (±10°) for arm joints, ±0.001 rad for gripper");
      goal.planning_options.plan_only = false;
      goal.planning_options.look_around = true;  // Enable look around to find alternative paths
      goal.planning_options.replan = true;  // Enable replanning if collision detected
      goal.planning_options.replan_attempts = 5;  // Allow multiple replanning attempts
      goal.planning_options.replan_delay = 0.5;  // Small delay before replanning

      RCLCPP_INFO(LOGGER, "Sending MoveGroup goal...");

      // Send goal
      auto goal_handle_future = movegroup_client->async_send_goal(goal);

      // Wait for goal acceptance
      auto start = std::chrono::steady_clock::now();
      auto timeout = std::chrono::seconds(10);
      while (goal_handle_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
        if (!rclcpp::ok()) {
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
      timeout = std::chrono::seconds(120);
      while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(LOGGER, "Interrupted while waiting for execution result");
          return false;
        }
        if (std::chrono::steady_clock::now() - start > timeout) {
          RCLCPP_ERROR(LOGGER, "Timeout waiting for execution result");
          return false;
        }
        rclcpp::spin_some(movegroup_node);
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
  }

  void run()
  {
    // Get offset parameters (in meters)
    // Note: Parameters are already declared by launch file
    double x_offset = get_parameter("x_offset").as_double();
    double y_offset = get_parameter("y_offset").as_double();
    double z_offset = get_parameter("z_offset").as_double();

    RCLCPP_INFO(LOGGER, "Moving with offset [X:%.4f, Y:%.4f, Z:%.4f]m from current position...",
                x_offset, y_offset, z_offset);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // Load robot model for FK/IK
    robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this());
    moveit::core::RobotModelConstPtr robot_model = robot_model_loader.getModel();
    if (!robot_model) {
      RCLCPP_ERROR(LOGGER, "Failed to load robot model");
      return;
    }

    // Get current joint states
    auto current_joints = get_current_joint_states();

    // Calculate target position with offset (in local frame)
    auto target_position = apply_cartesian_offset_to_joints(
        current_joints, x_offset, y_offset, z_offset,
        robot_model, "ur_arm", "robotiq_hande_end", "move_with_offset", false);

    // Move to target position
    if (!execute_movegroup_action("move_with_offset", target_position)) {
      RCLCPP_ERROR(LOGGER, "Failed to move with offset");
      return;
    }

    RCLCPP_INFO(LOGGER, "✅ Moved successfully!");
  }

private:
  std::map<std::string, double> holder1_standby_;
  std::map<std::string, double> sample_holder_standby_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create node with allow_undeclared_parameters
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true);
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<UprightConstraintTest>(options);

  // Wait for initialization
  RCLCPP_INFO(LOGGER, "Waiting for node initialization...");
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Run the test
  node->run();

  RCLCPP_INFO(LOGGER, "Test completed, shutting down...");
  rclcpp::shutdown();
  return 0;
}
