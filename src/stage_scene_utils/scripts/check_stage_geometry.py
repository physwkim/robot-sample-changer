#!/usr/bin/env python3
"""Check stage collision geometry in detail."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents


def main():
    rclpy.init()
    node = Node("check_stage_geometry")

    client = node.create_client(GetPlanningScene, '/get_planning_scene')

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /get_planning_scene not available!")
        rclpy.shutdown()
        return

    # Request full world geometry
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY |
        PlanningSceneComponents.WORLD_OBJECT_NAMES
    )

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Failed to get planning scene")
        rclpy.shutdown()
        return

    scene = future.result().scene

    # Find stage object
    stage_obj = None
    for obj in scene.world.collision_objects:
        if obj.id == "stage":
            stage_obj = obj
            break

    if stage_obj is None:
        node.get_logger().error("✗ 'stage' object NOT FOUND in planning scene!")
        node.get_logger().error("Run: ros2 run stage_scene_utils add_stage_to_scene")
        rclpy.shutdown()
        return

    node.get_logger().info("✓ Found 'stage' object")
    node.get_logger().info(f"\n=== STAGE DETAILS ===")
    node.get_logger().info(f"Frame: {stage_obj.header.frame_id}")
    node.get_logger().info(f"Operation: {stage_obj.operation}")

    # Check geometry
    if stage_obj.meshes:
        node.get_logger().info(f"\nMesh count: {len(stage_obj.meshes)}")
        for i, mesh in enumerate(stage_obj.meshes):
            vertices = len(mesh.vertices)
            triangles = len(mesh.triangles)
            node.get_logger().info(f"  Mesh {i}:")
            node.get_logger().info(f"    Vertices: {vertices}")
            node.get_logger().info(f"    Triangles: {triangles}")

            if vertices == 0:
                node.get_logger().error("    ✗ EMPTY MESH! No collision detection possible!")
            else:
                node.get_logger().info("    ✓ Valid mesh")

            # Check pose
            if i < len(stage_obj.mesh_poses):
                pose = stage_obj.mesh_poses[i]
                node.get_logger().info(f"    Position: [{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]")
                node.get_logger().info(f"    Orientation: [{pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f}]")

    elif stage_obj.primitives:
        node.get_logger().info(f"\nPrimitives count: {len(stage_obj.primitives)}")
        node.get_logger().info("  (Using primitive shapes, not mesh)")
    else:
        node.get_logger().error("\n✗ NO GEOMETRY! Stage has no mesh or primitives!")
        node.get_logger().error("This is why collision checking fails!")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
