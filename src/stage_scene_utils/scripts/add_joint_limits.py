#!/usr/bin/env python3
"""Add custom joint limits to restrict movement range."""

import rclpy
from rclpy.node import Node
import sys


def main():
    rclpy.init()
    node = Node("add_joint_limits")

    node.get_logger().info("=== Custom Joint Limits Guide ===\n")

    node.get_logger().info("To restrict joint movement ranges, edit:")
    node.get_logger().info("  /home/stevek/ws/src/erobs/src/custom-ur-descriptions/")
    node.get_logger().info("  ur3e_hande_moveit_config/config/joint_limits.yaml\n")

    example = """
Example: Restrict shoulder_pan_joint to -90° to +90°:

joint_limits:
  shoulder_pan_joint:
    has_velocity_limits: true
    max_velocity: 2.0943951023931953
    has_acceleration_limits: false
    max_acceleration: 0.0
    # Add position limits:
    has_position_limits: true
    min_position: -1.5708   # -90 degrees
    max_position: 1.5708    # +90 degrees

After editing:
1. Rebuild: colcon build --packages-select ur3e_hande_moveit_config
2. Restart MoveIt
3. Joint will not exceed these limits during planning
"""

    node.get_logger().info(example)

    node.get_logger().info("\n=== Available UR3e Joints ===")
    joints = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint"
    ]
    for j in joints:
        node.get_logger().info(f"  - {j}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
