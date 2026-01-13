#!/usr/bin/env python3
"""Set path constraints directly in the planning scene (for RViz Motion Planning)."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, Constraints, OrientationConstraint
from geometry_msgs.msg import Quaternion
import math


def main():
    rclpy.init()
    node = Node("set_scene_constraints")

    node.get_logger().info("=== Setting Scene Path Constraints (for RViz) ===\n")

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

    node.get_logger().info("📥 Getting current planning scene...")
    future = get_client.call_async(get_request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Failed to get planning scene")
        rclpy.shutdown()
        return

    scene = future.result().scene

    # Create orientation constraint (keep upright)
    constraint = OrientationConstraint()
    constraint.header.frame_id = "world"
    constraint.link_name = "tool0"
    constraint.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)  # Upright
    constraint.absolute_x_axis_tolerance = 0.1  # ±5.7 degrees
    constraint.absolute_y_axis_tolerance = 0.1
    constraint.absolute_z_axis_tolerance = 3.14159  # Free rotation around Z
    constraint.weight = 1.0

    # Create constraints message
    constraints = Constraints()
    constraints.name = "upright_constraint"
    constraints.orientation_constraints.append(constraint)

    # Add constraints to scene
    scene.path_constraints = constraints

    # Apply updated scene
    apply_request = ApplyPlanningScene.Request()
    apply_request.scene = scene

    node.get_logger().info("📤 Applying path constraints to planning scene...")
    apply_future = apply_client.call_async(apply_request)
    rclpy.spin_until_future_complete(node, apply_future)

    if apply_future.result() is not None and apply_future.result().success:
        node.get_logger().info("\n✅ Path constraints SET in planning scene!")
        node.get_logger().info(f"   Constraint: Keep tool0 upright")
        node.get_logger().info(f"   Tolerance: ±{math.degrees(0.1):.1f}° on X/Y axes")
        node.get_logger().info("   Free rotation around Z axis")
        node.get_logger().info("\n📌 Now RViz Motion Planning will use this constraint!")
        node.get_logger().info("\n   To verify:")
        node.get_logger().info("   ros2 run stage_scene_utils check_constraints.py")
        node.get_logger().info("\n   To clear:")
        node.get_logger().info("   ros2 run stage_scene_utils clear_constraints.py")
    else:
        node.get_logger().error("❌ Failed to apply planning scene")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
