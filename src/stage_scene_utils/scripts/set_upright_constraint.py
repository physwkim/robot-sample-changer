#!/usr/bin/env python3
"""Set upright constraint using pymoveit2 (doesn't execute, just shows how)."""

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import math


def main():
    rclpy.init()
    node = Node("set_upright_constraint")

    node.get_logger().info("=== Setting Upright Constraint Example ===\n")

    # UR3e joint names
    ur3e_joint_names = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    # Create MoveIt2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur3e_joint_names,
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur_manipulator",
    )

    node.get_logger().info("✓ MoveIt2 initialized")

    # Set orientation constraint (keep upright)
    # Upright orientation: [x, y, z, w] = [0, 0, 0, 1]
    # Tolerance: (x_tol, y_tol, z_tol) in radians
    # - Small tolerance on X and Y axes (±5.7°) to keep upright
    # - Large tolerance on Z axis (free rotation around Z)
    moveit2.set_path_orientation_constraint(
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],  # Upright orientation
        tolerance=(0.1, 0.1, 3.14159),   # (x_tol, y_tol, z_tol) in radians
        frame_id="world",
        target_link="tool0",
        weight=1.0
    )

    node.get_logger().info("\n✅ Upright constraint SET")
    node.get_logger().info(f"   Tolerance: ±{math.degrees(0.1):.1f}° on X/Y axes")
    node.get_logger().info("   Free rotation around Z axis")
    node.get_logger().info("\n📌 Now all planning will keep end effector upright!")
    node.get_logger().info("   Use RViz Motion Planning to plan and execute")
    node.get_logger().info("\n   To clear constraint later:")
    node.get_logger().info("   moveit2.clear_path_constraints()")

    # Keep node alive
    node.get_logger().info("\n⏸  Press Ctrl+C to exit and clear constraint...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Clear on exit
    moveit2.clear_path_constraints()
    node.get_logger().info("\n✓ Constraint cleared")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
