#!/usr/bin/env python3
"""Check current ACM for stage object."""

import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents


def main():
    rclpy.init()
    node = Node("check_acm")

    object_name = sys.argv[1] if len(sys.argv) > 1 else "stage"

    # Create service client
    client = node.create_client(GetPlanningScene, '/get_planning_scene')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /get_planning_scene not available!")
        rclpy.shutdown()
        return

    # Request ACM
    request = GetPlanningScene.Request()
    request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Failed to get planning scene")
        rclpy.shutdown()
        return

    acm = future.result().scene.allowed_collision_matrix

    # Find object in ACM
    if object_name not in acm.entry_names:
        node.get_logger().info(f"'{object_name}' not found in ACM")
        rclpy.shutdown()
        return

    obj_idx = acm.entry_names.index(object_name)

    node.get_logger().info(f"\n=== ACM entries for '{object_name}' ===")

    # Find all allowed collisions
    allowed = []
    for i, name in enumerate(acm.entry_names):
        if i != obj_idx and acm.entry_values[obj_idx].enabled[i]:
            allowed.append(name)

    if allowed:
        node.get_logger().info(f"Collision ALLOWED with {len(allowed)} links:")
        for link in sorted(allowed):
            node.get_logger().info(f"  - {link}")
    else:
        node.get_logger().info("No allowed collisions found")

    # Check robot links - arm and gripper
    arm_links = ["shoulder_link", "upper_arm_link", "forearm_link",
                 "wrist_1_link", "wrist_2_link", "wrist_3_link"]
    gripper_links = ["robotiq_hande_left_finger", "robotiq_hande_right_finger",
                     "robotiq_hande_link", "robotiq_hande_coupler"]
    base_links = ["base_link", "base_link_inertia", "base"]

    node.get_logger().info(f"\n=== ARM LINKS collision status (should all be BLOCKED) ===")
    for link in arm_links:
        if link in acm.entry_names:
            link_idx = acm.entry_names.index(link)
            if acm.entry_values[obj_idx].enabled[link_idx]:
                node.get_logger().error(f"  ✗ {link}: ALLOWED (WRONG! Should be BLOCKED!)")
            else:
                node.get_logger().info(f"  ✓ {link}: BLOCKED (correct)")
        else:
            node.get_logger().warn(f"  ? {link}: not in ACM")

    node.get_logger().info(f"\n=== GRIPPER LINKS collision status (should all be BLOCKED) ===")
    for link in gripper_links:
        if link in acm.entry_names:
            link_idx = acm.entry_names.index(link)
            if acm.entry_values[obj_idx].enabled[link_idx]:
                node.get_logger().error(f"  ✗ {link}: ALLOWED (WRONG! Should be BLOCKED!)")
            else:
                node.get_logger().info(f"  ✓ {link}: BLOCKED (correct)")
        else:
            node.get_logger().warn(f"  ? {link}: not in ACM")

    node.get_logger().info(f"\n=== BASE LINKS collision status (should all be ALLOWED) ===")
    for link in base_links:
        if link in acm.entry_names:
            link_idx = acm.entry_names.index(link)
            if acm.entry_values[obj_idx].enabled[link_idx]:
                node.get_logger().info(f"  ✓ {link}: ALLOWED (correct)")
            else:
                node.get_logger().warn(f"  ✗ {link}: BLOCKED (should be ALLOWED)")
        else:
            node.get_logger().warn(f"  ? {link}: not in ACM")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
