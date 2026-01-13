/**
 * Interactive planning node with upright orientation constraint.
 * Plans with ur_manipulator group (arm + gripper).
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>

using namespace std::chrono_literals;

class UprightPlanningNode : public rclcpp::Node
{
public:
  UprightPlanningNode() : Node("upright_planning_node"), received_joint_states_(false)
  {
    RCLCPP_INFO(get_logger(), "=== Upright Planning Node Starting ===");

    // Subscribe to joint_states to verify we're receiving data
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!received_joint_states_) {
          RCLCPP_INFO(get_logger(), "✓ Receiving joint_states");
          received_joint_states_ = true;
        }
      });
  }

  void initialize()
  {
    // Wait briefly for joint_states
    RCLCPP_INFO(get_logger(), "Waiting for joint_states...");
    auto start = now();
    while (!received_joint_states_ && (now() - start) < rclcpp::Duration(3s))
    {
      rclcpp::spin_some(shared_from_this());
      std::this_thread::sleep_for(100ms);
    }

    if (!received_joint_states_)
    {
      RCLCPP_WARN(get_logger(), "No joint_states received yet. Planning may fail.");
      RCLCPP_WARN(get_logger(), "Make sure ur_control is running and publishing joint_states.");
    }

    // Create MoveGroupInterface for ur_manipulator (arm + gripper)
    RCLCPP_INFO(get_logger(), "Initializing MoveGroupInterface...");
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), "ur_manipulator");

    // Set planning parameters
    move_group_->setPlanningTime(15.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.5);
    move_group_->setMaxAccelerationScalingFactor(0.5);

    // Set end effector link explicitly for ur_manipulator
    std::string ee_link = move_group_->getEndEffectorLink();
    if (ee_link.empty())
    {
      // ur_manipulator doesn't have end effector defined in SRDF
      // Use gripper TCP as end effector
      move_group_->setEndEffectorLink("robotiq_hande_end");
      ee_link = "robotiq_hande_end";
      RCLCPP_INFO(get_logger(), "Set end effector link to: robotiq_hande_end");
    }

    RCLCPP_INFO(get_logger(), "✓ Planning group: %s", move_group_->getName().c_str());
    RCLCPP_INFO(get_logger(), "✓ Planning frame: %s", move_group_->getPlanningFrame().c_str());
    RCLCPP_INFO(get_logger(), "✓ End effector link: %s", ee_link.c_str());

    // Print joint names
    const auto& joints = move_group_->getJointNames();
    RCLCPP_INFO(get_logger(), "✓ Joints (%zu):", joints.size());
    for (const auto& joint : joints)
    {
      RCLCPP_INFO(get_logger(), "    - %s", joint.c_str());
    }

    // Set upright constraint
    setUprightConstraint();

    // Create service for planning
    plan_service_ = create_service<std_srvs::srv::Trigger>(
      "/plan_to_current_target",
      std::bind(&UprightPlanningNode::planCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "\n=== Ready to Plan ===");
    RCLCPP_INFO(get_logger(), "Set target pose in RViz Motion Planning panel");
    RCLCPP_INFO(get_logger(), "Then call: ros2 service call /plan_to_current_target std_srvs/srv/Trigger");
  }

private:
  void setUprightConstraint()
  {
    std::string ee_link = move_group_->getEndEffectorLink();
    if (ee_link.empty())
    {
      // If still empty, set it again
      move_group_->setEndEffectorLink("robotiq_hande_end");
      ee_link = "robotiq_hande_end";
      RCLCPP_WARN(get_logger(), "End effector was empty, set to: robotiq_hande_end");
    }

    // Try to get current pose (may fail if no joint_states)
    geometry_msgs::msg::PoseStamped current_pose;
    try
    {
      current_pose = move_group_->getCurrentPose(ee_link);
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(get_logger(), "Could not get current pose: %s", e.what());
      RCLCPP_WARN(get_logger(), "Using default upright orientation [0,0,0,1]");
      current_pose.pose.orientation.w = 1.0;
      current_pose.pose.orientation.x = 0.0;
      current_pose.pose.orientation.y = 0.0;
      current_pose.pose.orientation.z = 0.0;
    }

    // Create upright orientation constraint
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = move_group_->getPlanningFrame();
    oc.link_name = ee_link;
    oc.orientation = current_pose.pose.orientation;

    // Tolerances: tight on X/Y (upright), free rotation around Z (yaw)
    oc.absolute_x_axis_tolerance = 0.017;  // ~1 degree
    oc.absolute_y_axis_tolerance = 0.017;  // ~1 degree
    oc.absolute_z_axis_tolerance = 3.14159;  // Free rotation
    oc.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.name = "keep_upright";
    constraints.orientation_constraints.push_back(oc);

    move_group_->setPathConstraints(constraints);

    RCLCPP_INFO(get_logger(), "\n✓ Upright constraint set:");
    RCLCPP_INFO(get_logger(), "   Link: %s", oc.link_name.c_str());
    RCLCPP_INFO(get_logger(), "   Orientation: [%.3f, %.3f, %.3f, %.3f]",
                oc.orientation.x, oc.orientation.y,
                oc.orientation.z, oc.orientation.w);
    RCLCPP_INFO(get_logger(), "   Tolerance: ±1° on X/Y, free Z rotation");
  }

  void planCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "\n=== Planning with Upright Constraint ===");

    if (!received_joint_states_)
    {
      RCLCPP_ERROR(get_logger(), "Cannot plan: No joint_states received!");
      RCLCPP_ERROR(get_logger(), "Make sure ur_control is running.");
      response->success = false;
      response->message = "No joint_states available";
      return;
    }

    // Plan to current target (set via RViz)
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "✓ Planning SUCCEEDED!");
      RCLCPP_INFO(get_logger(), "   Trajectory: %zu waypoints, %.2f seconds",
                  plan.trajectory_.joint_trajectory.points.size(),
                  plan.trajectory_.joint_trajectory.points.empty() ? 0.0 :
                  plan.trajectory_.joint_trajectory.points.back().time_from_start.sec +
                  plan.trajectory_.joint_trajectory.points.back().time_from_start.nanosec * 1e-9);

      // Execute
      RCLCPP_INFO(get_logger(), "Executing trajectory...");
      auto exec_result = move_group_->execute(plan);

      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(get_logger(), "✓ Execution SUCCEEDED!");
        response->success = true;
        response->message = "Planned and executed successfully";
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "✗ Execution FAILED!");
        response->success = false;
        response->message = "Planning succeeded but execution failed";
      }
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "✗ Planning FAILED!");
      RCLCPP_ERROR(get_logger(), "   Try adjusting target pose or increasing planning time");
      response->success = false;
      response->message = "Planning failed";
    }
  }

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  bool received_joint_states_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UprightPlanningNode>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
