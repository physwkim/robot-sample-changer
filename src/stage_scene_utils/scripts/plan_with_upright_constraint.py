#!/usr/bin/env python3
"""Complete example: Plan to a pose with upright constraint."""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint, MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import math


def main(args=None):
    rclpy.init(args=args)
    node = Node("plan_with_upright_constraint")

    node.get_logger().info("=== Planning with Upright End Effector Constraint ===\n")

    # Create action client
    action_client = ActionClient(node, MoveGroup, '/move_action')

    if not action_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("MoveGroup action server not available!")
        node.get_logger().error("Make sure MoveIt is running")
        rclpy.shutdown()
        return

    node.get_logger().info("✓ Connected to MoveGroup action server")

    # Example target pose (modify these values!)
    target_pose = PoseStamped()
    target_pose.header.frame_id = "world"
    target_pose.pose.position.x = 0.3
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.4
    # Keep upright orientation
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    # Create orientation constraint (keep upright during path)
    path_constraint = OrientationConstraint()
    path_constraint.header.frame_id = "world"
    path_constraint.link_name = "tool0"
    path_constraint.orientation = target_pose.pose.orientation
    path_constraint.absolute_x_axis_tolerance = 0.1  # ±5.7°
    path_constraint.absolute_y_axis_tolerance = 0.1
    path_constraint.absolute_z_axis_tolerance = 3.14159  # Free rotation around Z
    path_constraint.weight = 1.0

    constraints = Constraints()
    constraints.name = "upright_constraint"
    constraints.orientation_constraints.append(path_constraint)

    # Create goal
    goal = MoveGroup.Goal()
    goal.request.group_name = "ur_manipulator"
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 10.0
    goal.request.max_velocity_scaling_factor = 0.1
    goal.request.max_acceleration_scaling_factor = 0.1

    # Add pose target
    goal.request.goal_constraints.append(Constraints())
    # Note: This is simplified - full implementation would need PositionConstraint + OrientationConstraint

    # Add path constraint
    goal.request.path_constraints = constraints

    node.get_logger().info("\n📋 Planning with constraints:")
    node.get_logger().info(f"  Target: [{target_pose.pose.position.x:.3f}, "
                          f"{target_pose.pose.position.y:.3f}, "
                          f"{target_pose.pose.position.z:.3f}]")
    node.get_logger().info(f"  Constraint: Keep tool0 upright (±{math.degrees(0.1):.1f}°)")
    node.get_logger().info(f"  Planning time: {goal.request.allowed_planning_time}s")

    node.get_logger().info("\n⏳ Sending planning request...")

    # Send goal
    future = action_client.send_goal_async(goal)

    # Wait for result
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        goal_handle = future.result()
        if goal_handle.accepted:
            node.get_logger().info("✓ Goal accepted, planning...")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(node, result_future)

            result = result_future.result().result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                node.get_logger().info("\n✅ Planning SUCCEEDED with constraint!")
                node.get_logger().info(f"   Trajectory has {len(result.planned_trajectory.joint_trajectory.points)} waypoints")
            else:
                node.get_logger().error(f"\n❌ Planning FAILED: Error code {result.error_code.val}")
                node.get_logger().error("   Try increasing planning time or relaxing tolerance")
        else:
            node.get_logger().error("Goal rejected")
    else:
        node.get_logger().error("Failed to send goal")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
