#!/usr/bin/env python3
"""Clear path constraints from the planning scene."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, Constraints


def main():
    rclpy.init()
    node = Node("clear_constraints")

    node.get_logger().info("=== Clearing Path Constraints ===\n")

    # Create service clients
    get_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    apply_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

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
    get_request.components.components = (
        PlanningSceneComponents.SCENE_SETTINGS |
        PlanningSceneComponents.ROBOT_STATE |
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX |
        PlanningSceneComponents.WORLD_OBJECT_NAMES |
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
    )

    future = get_client.call_async(get_request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Failed to get planning scene")
        rclpy.shutdown()
        return

    scene = future.result().scene

    # Clear path constraints
    scene.path_constraints = Constraints()

    # Apply updated scene
    apply_request = ApplyPlanningScene.Request()
    apply_request.scene = scene

    node.get_logger().info("📤 Clearing path constraints...")
    apply_future = apply_client.call_async(apply_request)
    rclpy.spin_until_future_complete(node, apply_future)

    if apply_future.result() is not None and apply_future.result().success:
        node.get_logger().info("\n✅ Path constraints CLEARED!")
        node.get_logger().info("\n   To verify:")
        node.get_logger().info("   ros2 run stage_scene_utils check_constraints.py")
    else:
        node.get_logger().error("❌ Failed to apply planning scene")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
