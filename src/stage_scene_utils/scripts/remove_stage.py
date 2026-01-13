#!/usr/bin/env python3
"""Remove stage object from planning scene."""

import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import time


def main():
    rclpy.init()
    node = Node("remove_stage")

    # Get object name from command line or use default
    object_name = sys.argv[1] if len(sys.argv) > 1 else "stage"

    node.get_logger().info(f"Removing object '{object_name}' from planning scene...")

    # Create service clients
    get_client = node.create_client(GetPlanningScene, '/get_planning_scene')
    apply_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')

    # Wait for services to be available
    if not get_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /get_planning_scene not available!")
        rclpy.shutdown()
        return

    if not apply_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /apply_planning_scene not available!")
        rclpy.shutdown()
        return

    # First, get current scene to check if object exists
    get_request = GetPlanningScene.Request()
    get_request.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES

    get_future = get_client.call_async(get_request)
    rclpy.spin_until_future_complete(node, get_future)

    if get_future.result() is None:
        node.get_logger().error("Failed to get current planning scene")
        rclpy.shutdown()
        return

    current_objects = [obj.id for obj in get_future.result().scene.world.collision_objects]
    node.get_logger().info(f"Current objects in scene: {current_objects}")

    if object_name not in current_objects:
        node.get_logger().warn(f"Object '{object_name}' not found in planning scene")
        rclpy.shutdown()
        return

    # Create planning scene diff message with REMOVE operation
    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.robot_state.is_diff = True

    # Create collision object with REMOVE operation
    collision_object = CollisionObject()
    collision_object.id = object_name
    collision_object.header.frame_id = "world"
    collision_object.operation = CollisionObject.REMOVE

    planning_scene.world.collision_objects.append(collision_object)

    # Create service request
    request = ApplyPlanningScene.Request()
    request.scene = planning_scene

    # Call service
    node.get_logger().info(f"Sending REMOVE request for '{object_name}'...")
    future = apply_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        if future.result().success:
            node.get_logger().info(f"Successfully removed '{object_name}' from planning scene")
        else:
            node.get_logger().error(f"Service returned False - failed to remove '{object_name}'")
    else:
        node.get_logger().error("Service call failed")

    # Verify removal
    time.sleep(1.0)
    get_future2 = get_client.call_async(get_request)
    rclpy.spin_until_future_complete(node, get_future2)

    if get_future2.result() is not None:
        remaining_objects = [obj.id for obj in get_future2.result().scene.world.collision_objects]
        node.get_logger().info(f"Remaining objects: {remaining_objects}")

        if object_name not in remaining_objects:
            node.get_logger().info(f"✓ Confirmed: '{object_name}' has been removed")
        else:
            node.get_logger().error(f"✗ Object '{object_name}' still exists in scene!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
