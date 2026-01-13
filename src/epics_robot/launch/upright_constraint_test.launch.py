import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('mtc_tutorial').find('mtc_tutorial')

    # Path to taught waypoints config
    taught_waypoints_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'taught_waypoints.yaml'
    ])

    # Declare launch arguments for Cartesian offset
    x_offset_arg = DeclareLaunchArgument(
        'x_offset',
        default_value='0.0',
        description='X offset in meters (end-effector local frame)'
    )

    y_offset_arg = DeclareLaunchArgument(
        'y_offset',
        default_value='0.04',
        description='Y offset in meters (end-effector local frame, 0.04 = 40mm down)'
    )

    z_offset_arg = DeclareLaunchArgument(
        'z_offset',
        default_value='0.0',
        description='Z offset in meters (end-effector local frame)'
    )

    # Upright constraint test node
    upright_test_node = Node(
        package='mtc_tutorial',
        executable='upright_constraint_test',
        name='upright_constraint_test',
        output='screen',
        parameters=[
            taught_waypoints_file,
            {
                'x_offset': LaunchConfiguration('x_offset'),
                'y_offset': LaunchConfiguration('y_offset'),
                'z_offset': LaunchConfiguration('z_offset'),
            }
        ]
    )

    return LaunchDescription([
        x_offset_arg,
        y_offset_arg,
        z_offset_arg,
        upright_test_node,
    ])
