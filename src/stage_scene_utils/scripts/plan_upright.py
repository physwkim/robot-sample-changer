#!/usr/bin/env python3
"""Plan with end effector kept upright (orientation constraint)."""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import Constraints, OrientationConstraint
from geometry_msgs.msg import Quaternion
import sys


def main():
    rclpy.init()
    node = Node("plan_upright")

    node.get_logger().info("=== Setting Upright Orientation Constraint ===\n")

    # Create orientation constraint
    constraint = OrientationConstraint()
    constraint.header.frame_id = "world"
    constraint.link_name = "tool0"  # or "robotiq_hande_end"

    # Keep upright: no rotation (quaternion [0,0,0,1])
    constraint.orientation = Quaternion()
    constraint.orientation.x = 0.0
    constraint.orientation.y = 0.0
    constraint.orientation.z = 0.0
    constraint.orientation.w = 1.0

    # Tolerances (in radians)
    constraint.absolute_x_axis_tolerance = 0.1  # ~5.7 degrees
    constraint.absolute_y_axis_tolerance = 0.1  # ~5.7 degrees
    constraint.absolute_z_axis_tolerance = 3.14159  # Free to rotate around Z
    constraint.weight = 1.0

    node.get_logger().info("Constraint created:")
    node.get_logger().info(f"  Link: {constraint.link_name}")
    node.get_logger().info(f"  Target orientation: [{constraint.orientation.x}, "
                          f"{constraint.orientation.y}, {constraint.orientation.z}, "
                          f"{constraint.orientation.w}]")
    node.get_logger().info(f"  Tolerance: ±{constraint.absolute_x_axis_tolerance} rad")

    # Save to parameter server for RViz to use
    node.get_logger().info("\n✓ To use this constraint:")
    node.get_logger().info("1. In your MoveIt planning code, add:")
    node.get_logger().info("   move_group.setPathConstraints(constraints)")
    node.get_logger().info("2. Or use MoveGroup Python API:")

    example = """
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import Constraints, OrientationConstraint
from geometry_msgs.msg import Quaternion

# Initialize
roscpp_initialize(sys.argv)
move_group = MoveGroupCommander("ur_manipulator")

# Create constraint
constraint = OrientationConstraint()
constraint.header.frame_id = "world"
constraint.link_name = "tool0"
constraint.orientation = Quaternion(x=0, y=0, z=0, w=1)
constraint.absolute_x_axis_tolerance = 0.1
constraint.absolute_y_axis_tolerance = 0.1
constraint.absolute_z_axis_tolerance = 3.14159
constraint.weight = 1.0

constraints = Constraints()
constraints.orientation_constraints.append(constraint)

# Apply constraint
move_group.set_path_constraints(constraints)

# Now plan with constraint
move_group.set_pose_target(target_pose)
plan = move_group.plan()

# Clear constraint when done
move_group.clear_path_constraints()
"""

    node.get_logger().info(example)

    node.get_logger().info("\n⚠️  Note: Path constraints make planning HARDER")
    node.get_logger().info("   Planning may fail more often or take longer")
    node.get_logger().info("   Increase planning time if needed")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
