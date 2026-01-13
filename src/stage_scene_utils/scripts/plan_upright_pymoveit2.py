#!/usr/bin/env python3
"""Plan with upright constraint using pymoveit2."""

import rclpy
from rclpy.node import Node
from pymoveit2 import MoveIt2
import math


def main():
    rclpy.init()
    node = Node("plan_upright_pymoveit2")

    node.get_logger().info("=== Planning with Upright Constraint (pymoveit2) ===\n")

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

    node.get_logger().info("✓ MoveIt2 interface initialized")

    # Set orientation constraint (keep upright)
    node.get_logger().info("📋 Setting constraint:")
    node.get_logger().info("  Link: tool0")
    node.get_logger().info("  Target: upright [0, 0, 0, 1]")
    node.get_logger().info(f"  Tolerance: ±{math.degrees(0.1):.1f}° on X/Y axes")
    node.get_logger().info("  Free rotation around Z axis")

    moveit2.set_path_orientation_constraint(
        quat_xyzw=[0.0, 0.0, 0.0, 1.0],  # Upright orientation
        tolerance=(0.1, 0.1, 3.14159),   # (x_tol, y_tol, z_tol) in radians
        frame_id="world",
        target_link="tool0",
        weight=1.0
    )

    # Example target pose (modify these values!)
    target_position = [0.3, 0.2, 0.4]
    target_orientation = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w] - upright

    node.get_logger().info(f"\n🎯 Target pose:")
    node.get_logger().info(f"  Position: [{target_position[0]:.3f}, "
                          f"{target_position[1]:.3f}, "
                          f"{target_position[2]:.3f}]")
    node.get_logger().info(f"  Orientation: upright")

    # Plan to target
    node.get_logger().info("\n⏳ Planning with upright constraint...")

    moveit2.move_to_pose(
        position=target_position,
        quat_xyzw=target_orientation,
        cartesian=False,
        tolerance_position=0.001,
        tolerance_orientation=0.01,
        weight_position=1.0,
        weight_orientation=1.0,
    )

    node.get_logger().info("\n✅ Planning request sent!")
    node.get_logger().info("   Robot will move with upright constraint")

    # Wait for movement to complete
    moveit2.wait_until_executed()

    # Clear constraints
    moveit2.clear_path_constraints()
    node.get_logger().info("\n✓ Constraints cleared")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
