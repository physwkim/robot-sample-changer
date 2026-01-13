#!/usr/bin/env python3
"""Test if collision checking is working."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity, GetPlanningScene
from moveit_msgs.msg import RobotState, PlanningSceneComponents
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = Node("test_collision")

    # Create service clients
    validity_client = node.create_client(GetStateValidity, '/check_state_validity')
    scene_client = node.create_client(GetPlanningScene, '/get_planning_scene')

    if not validity_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /check_state_validity not available!")
        node.get_logger().error("Make sure MoveIt is running")
        rclpy.shutdown()
        return

    if not scene_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /get_planning_scene not available!")
        rclpy.shutdown()
        return

    # Get current planning scene to get robot state
    scene_request = GetPlanningScene.Request()
    scene_request.components.components = (
        PlanningSceneComponents.ROBOT_STATE |
        PlanningSceneComponents.WORLD_OBJECT_NAMES
    )

    scene_future = scene_client.call_async(scene_request)
    rclpy.spin_until_future_complete(node, scene_future)

    if scene_future.result() is None:
        node.get_logger().error("Failed to get planning scene")
        rclpy.shutdown()
        return

    current_scene = scene_future.result().scene

    # Check if there are any collision objects
    num_objects = len(current_scene.world.collision_objects)
    node.get_logger().info(f"Planning scene has {num_objects} collision object(s)")
    if num_objects == 0:
        node.get_logger().warn("WARNING: No collision objects in scene!")
        node.get_logger().warn("Add 'stage' with: ros2 run stage_scene_utils add_stage_to_scene")

    # Test current robot state
    request = GetStateValidity.Request()
    request.robot_state = current_scene.robot_state
    request.group_name = "ur_manipulator"

    node.get_logger().info("Checking current robot state for collisions...")

    future = validity_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Service call failed")
        rclpy.shutdown()
        return

    result = future.result()

    node.get_logger().info(f"\n=== COLLISION CHECK RESULT ===")
    node.get_logger().info(f"Valid: {result.valid}")

    if result.valid:
        node.get_logger().info("✓ Current state is VALID (no collision)")
    else:
        node.get_logger().warn("✗ Current state is INVALID (collision detected!)")

    if result.contacts:
        node.get_logger().info(f"\nDetected {len(result.contacts)} collision(s):")
        for i, contact in enumerate(result.contacts[:5]):  # Show first 5
            node.get_logger().info(f"  {i+1}. {contact.contact_body_1} <-> {contact.contact_body_2}")
            node.get_logger().info(f"     Depth: {contact.depth:.4f}m")
    else:
        node.get_logger().info("No collision contacts reported")

    # Additional info
    node.get_logger().info(f"\nCost sources: {len(result.cost_sources)}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
