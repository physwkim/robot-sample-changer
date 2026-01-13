#!/usr/bin/env python3
"""Check MoveIt planner parameters at runtime."""

import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters


def main():
    rclpy.init()
    node = Node("check_planner_params")

    # Get parameters from move_group node
    param_client = node.create_client(
        GetParameters,
        '/move_group/get_parameters'
    )

    if not param_client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("move_group node not found!")
        node.get_logger().error("Make sure MoveIt is running")
        rclpy.shutdown()
        return

    # Request planner parameters
    request = GetParameters.Request()
    request.names = [
        'ur_manipulator.longest_valid_segment_fraction',
        'planning_plugin',
    ]

    future = param_client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error("Failed to get parameters")
        rclpy.shutdown()
        return

    node.get_logger().info("\n=== PLANNER PARAMETERS ===")
    for param in future.result().values:
        param_type = type(param).__name__
        if hasattr(param, 'double_value'):
            value = param.double_value
        elif hasattr(param, 'string_value'):
            value = param.string_value
        elif hasattr(param, 'integer_value'):
            value = param.integer_value
        else:
            value = str(param)

        node.get_logger().info(f"  Type: {param_type}, Value: {value}")

    node.get_logger().info("\n=== Checking if longest_valid_segment_fraction was applied ===")
    if len(future.result().values) > 0:
        first_param = future.result().values[0]
        if hasattr(first_param, 'double_value'):
            if first_param.double_value == 0.005:
                node.get_logger().info("✓ Configuration APPLIED (0.005)")
            elif first_param.double_value == 0.01:
                node.get_logger().warn("✗ Still using OLD value (0.01) - MoveIt needs restart!")
            else:
                node.get_logger().info(f"  Current value: {first_param.double_value}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
