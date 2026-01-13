#!/usr/bin/env python3
"""Example: Plan with orientation constraint (keep end effector upright)."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest
from sensor_msgs.msg import JointState
import math


def main():
    rclpy.init()
    node = Node("plan_with_constraint")

    # This is an EXAMPLE showing how to add constraints
    # You would integrate this into your planning workflow

    node.get_logger().info("=== Orientation Constraint Example ===")
    node.get_logger().info("\nIn RViz Motion Planning plugin:")
    node.get_logger().info("1. Go to 'Path Constraints' section")
    node.get_logger().info("2. Check 'Use Path Constraints'")
    node.get_logger().info("3. Add constraint:")
    node.get_logger().info("   - Type: Orientation")
    node.get_logger().info("   - Link: tool0 (or robotiq_hande_end)")
    node.get_logger().info("   - Target orientation: [0, 0, 0, 1] (upright)")
    node.get_logger().info("   - Tolerance: 0.1 radians (~5.7 degrees)")
    node.get_logger().info("\n4. Click 'Plan' - path will keep end effector upright!")

    node.get_logger().info("\n=== Other useful constraints ===")
    node.get_logger().info("• Joint Constraint: Limit specific joint angles")
    node.get_logger().info("  Example: shoulder_pan_joint between -90° to 90°")
    node.get_logger().info("\n• Position Constraint: Keep in certain region")
    node.get_logger().info("  Example: Stay within 0.5m radius sphere")
    node.get_logger().info("\n• Cartesian Path: Move in straight line")
    node.get_logger().info("  Use: move_group.computeCartesianPath()")

    # Example constraint definition (for reference)
    node.get_logger().info("\n=== Code Example ===")
    example_code = """
# Orientation constraint example
constraint = OrientationConstraint()
constraint.header.frame_id = "world"
constraint.link_name = "tool0"
constraint.orientation.w = 1.0  # Upright
constraint.absolute_x_axis_tolerance = 0.1
constraint.absolute_y_axis_tolerance = 0.1
constraint.absolute_z_axis_tolerance = 3.14  # Free rotation around z
constraint.weight = 1.0

constraints = Constraints()
constraints.orientation_constraints.append(constraint)

# Add to planning request
motion_plan_request.path_constraints = constraints
"""
    node.get_logger().info(example_code)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
