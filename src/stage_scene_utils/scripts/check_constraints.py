#!/usr/bin/env python3
"""Check current path constraints in the planning scene."""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
import math


def main():
    rclpy.init()
    node = Node("check_constraints")

    # Create service client
    client = node.create_client(GetPlanningScene, "/get_planning_scene")

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service /get_planning_scene not available!")
        rclpy.shutdown()
        return

    # Request planning scene with path constraints
    request = GetPlanningScene.Request()
    request.components.components = (
        PlanningSceneComponents.SCENE_SETTINGS |
        PlanningSceneComponents.ROBOT_STATE
    )

    node.get_logger().info("=== Checking Path Constraints ===\n")

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        scene = future.result().scene

        if scene.robot_state.attached_collision_objects:
            node.get_logger().info(f"Attached objects: {len(scene.robot_state.attached_collision_objects)}")

        # Check path constraints
        constraints = scene.path_constraints

        if not constraints.name:
            node.get_logger().warn("❌ NO path constraints set!")
            node.get_logger().info("\nTo set constraints, run:")
            node.get_logger().info("  ros2 run stage_scene_utils set_upright_constraint.py")
        else:
            node.get_logger().info(f"✅ Path constraints found: '{constraints.name}'\n")

            # Check orientation constraints
            if constraints.orientation_constraints:
                node.get_logger().info(f"Orientation constraints: {len(constraints.orientation_constraints)}")
                for i, oc in enumerate(constraints.orientation_constraints):
                    node.get_logger().info(f"\n  Constraint {i+1}:")
                    node.get_logger().info(f"    Link: {oc.link_name}")
                    node.get_logger().info(f"    Frame: {oc.header.frame_id}")
                    node.get_logger().info(f"    Target orientation: [{oc.orientation.x:.3f}, "
                                          f"{oc.orientation.y:.3f}, {oc.orientation.z:.3f}, "
                                          f"{oc.orientation.w:.3f}]")
                    node.get_logger().info(f"    Tolerance X: {oc.absolute_x_axis_tolerance:.4f} rad "
                                          f"(±{math.degrees(oc.absolute_x_axis_tolerance):.1f}°)")
                    node.get_logger().info(f"    Tolerance Y: {oc.absolute_y_axis_tolerance:.4f} rad "
                                          f"(±{math.degrees(oc.absolute_y_axis_tolerance):.1f}°)")
                    node.get_logger().info(f"    Tolerance Z: {oc.absolute_z_axis_tolerance:.4f} rad "
                                          f"(±{math.degrees(oc.absolute_z_axis_tolerance):.1f}°)")
                    node.get_logger().info(f"    Weight: {oc.weight}")

            # Check position constraints
            if constraints.position_constraints:
                node.get_logger().info(f"\nPosition constraints: {len(constraints.position_constraints)}")

            # Check joint constraints
            if constraints.joint_constraints:
                node.get_logger().info(f"\nJoint constraints: {len(constraints.joint_constraints)}")

            if not constraints.orientation_constraints and not constraints.position_constraints and not constraints.joint_constraints:
                node.get_logger().warn("\n⚠️  Constraints object exists but is EMPTY!")
    else:
        node.get_logger().error("Failed to get planning scene")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
