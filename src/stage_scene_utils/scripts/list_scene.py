#!/usr/bin/env python3
"""List all objects in planning scene."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents


def main():
    rclpy.init()
    node = Node("list_scene")

    # Create service client
    client = node.create_client(GetPlanningScene, '/get_planning_scene')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /get_planning_scene not available!")
        rclpy.shutdown()
        return

    # Request full world state
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES |
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
        PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Failed to get planning scene")
        rclpy.shutdown()
        return

    scene = future.result().scene

    # List collision objects
    node.get_logger().info("\n=== COLLISION OBJECTS IN PLANNING SCENE ===")
    if not scene.world.collision_objects:
        node.get_logger().info("  (none)")
    else:
        for obj in scene.world.collision_objects:
            obj_type = "Unknown"
            if obj.meshes:
                obj_type = f"Mesh ({len(obj.meshes[0].vertices)} vertices)"
            elif obj.primitives:
                obj_type = f"Primitive ({obj.primitives[0].type})"
            elif obj.planes:
                obj_type = "Plane"

            node.get_logger().info(f"  - {obj.id}: {obj_type} in frame '{obj.header.frame_id}'")

    # List ACM entries
    acm = scene.allowed_collision_matrix
    node.get_logger().info(f"\n=== ALLOWED COLLISION MATRIX ===")
    node.get_logger().info(f"Total entries: {len(acm.entry_names)}")

    # Find non-robot entries (collision objects)
    robot_prefixes = ['base', 'shoulder', 'upper_arm', 'forearm', 'wrist', 'flange', 'tool0', 'robotiq']
    collision_objects_in_acm = []

    for name in acm.entry_names:
        is_robot_link = any(name.startswith(prefix) for prefix in robot_prefixes)
        if not is_robot_link and name != 'world':
            collision_objects_in_acm.append(name)

    if collision_objects_in_acm:
        node.get_logger().info(f"\nCollision objects in ACM:")
        for obj_name in collision_objects_in_acm:
            obj_idx = acm.entry_names.index(obj_name)
            allowed_count = sum(acm.entry_values[obj_idx].enabled)
            node.get_logger().info(f"  - {obj_name}: {allowed_count} allowed collisions")

            # Show which links can collide
            allowed_links = []
            for i, link_name in enumerate(acm.entry_names):
                if acm.entry_values[obj_idx].enabled[i]:
                    allowed_links.append(link_name)

            if allowed_links:
                node.get_logger().info(f"    Allowed with: {', '.join(allowed_links[:5])}" +
                                     (" ..." if len(allowed_links) > 5 else ""))

    rclpy.shutdown()


if __name__ == "__main__":
    main()
