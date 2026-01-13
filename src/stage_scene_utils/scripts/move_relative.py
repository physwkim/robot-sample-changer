#!/usr/bin/env python3
"""Move robot end-effector relative to current position in cartesian space."""

import sys
import rclpy
from rclpy.node import Node
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose, Point, Quaternion
import argparse


class RelativeCartesianMover(Node):
    def __init__(self):
        super().__init__('relative_cartesian_mover')

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0, group_name='ur_manipulator'):
        """Move end-effector relative to current position.

        Args:
            dx, dy, dz: Relative movement in meters
            group_name: Planning group name
        """
        # Initialize MoveItPy
        moveit = MoveItPy(node_name="moveit_py_relative_move")
        robot = moveit.get_robot_model()

        # Get planning component
        arm = moveit.get_planning_component(group_name)

        # Get current pose
        robot_state = moveit.get_planning_scene_monitor().state_monitor.get_current_state()
        current_pose = robot_state.get_pose(arm.get_end_effector_link())

        self.get_logger().info(f"Current pose: x={current_pose.position.x:.3f}, "
                              f"y={current_pose.position.y:.3f}, "
                              f"z={current_pose.position.z:.3f}")

        # Calculate target pose
        target_pose = Pose()
        target_pose.position.x = current_pose.position.x + dx
        target_pose.position.y = current_pose.position.y + dy
        target_pose.position.z = current_pose.position.z + dz
        target_pose.orientation = current_pose.orientation

        self.get_logger().info(f"Target pose: x={target_pose.position.x:.3f}, "
                              f"y={target_pose.position.y:.3f}, "
                              f"z={target_pose.position.z:.3f}")

        # Plan and execute
        arm.set_goal_state(pose_stamped_msg=target_pose, pose_link=arm.get_end_effector_link())
        plan_result = arm.plan()

        if plan_result:
            self.get_logger().info("Planning successful! Executing...")
            arm.execute()
            self.get_logger().info("Movement complete!")
            return True
        else:
            self.get_logger().error("Planning failed!")
            return False


def main():
    parser = argparse.ArgumentParser(description='Move robot relative in cartesian space')
    parser.add_argument('--dx', type=float, default=0.0, help='X displacement (m)')
    parser.add_argument('--dy', type=float, default=0.0, help='Y displacement (m)')
    parser.add_argument('--dz', type=float, default=0.0, help='Z displacement (m)')
    parser.add_argument('--group', type=str, default='ur_manipulator',
                       help='Planning group name')

    args = parser.parse_args()

    rclpy.init()
    node = RelativeCartesianMover()

    try:
        success = node.move_relative(
            dx=args.dx,
            dy=args.dy,
            dz=args.dz,
            group_name=args.group
        )
        sys.exit(0 if success else 1)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
