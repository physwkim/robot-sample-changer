#!/usr/bin/env python3
"""Reset ACM by removing non-robot entries."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene


def main():
    rclpy.init()
    node = Node("reset_acm")

    # Create service clients
    get_client = node.create_client(GetPlanningScene, '/get_planning_scene')
    apply_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')

    if not get_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /get_planning_scene not available!")
        rclpy.shutdown()
        return

    if not apply_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /apply_planning_scene not available!")
        rclpy.shutdown()
        return

    # Get current planning scene
    get_request = GetPlanningScene.Request()
    get_request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX

    future = get_client.call_async(get_request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Failed to get planning scene")
        rclpy.shutdown()
        return

    scene = future.result().scene
    acm = scene.allowed_collision_matrix

    # Find robot links and non-robot entries
    robot_prefixes = ['base', 'shoulder', 'upper_arm', 'forearm', 'wrist', 'flange', 'tool0', 'robotiq']

    robot_links = []
    non_robot_entries = []

    for name in acm.entry_names:
        is_robot = any(name.startswith(prefix) for prefix in robot_prefixes) or name == 'world'
        if is_robot:
            robot_links.append(name)
        else:
            non_robot_entries.append(name)

    if not non_robot_entries:
        node.get_logger().info("ACM is already clean (no non-robot entries)")
        rclpy.shutdown()
        return

    node.get_logger().info(f"Found {len(non_robot_entries)} non-robot entries in ACM: {non_robot_entries}")
    node.get_logger().info(f"Removing them and keeping {len(robot_links)} robot links...")

    # Create new ACM with only robot links
    new_acm = type(acm)()
    new_acm.entry_names = robot_links

    # Rebuild ACM matrix for robot links only
    for i, link_i in enumerate(robot_links):
        old_i = acm.entry_names.index(link_i)

        new_row = type(acm.entry_values[0])()
        new_row.enabled = []

        for j, link_j in enumerate(robot_links):
            old_j = acm.entry_names.index(link_j)
            new_row.enabled.append(acm.entry_values[old_i].enabled[old_j])

        new_acm.entry_values.append(new_row)

    # Apply cleaned ACM
    apply_request = ApplyPlanningScene.Request()
    apply_request.scene = PlanningScene()
    apply_request.scene.allowed_collision_matrix = new_acm
    apply_request.scene.is_diff = False

    future = apply_client.call_async(apply_request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() and future.result().success:
        node.get_logger().info("✓ Successfully cleaned ACM")
    else:
        node.get_logger().error("✗ Failed to clean ACM")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
