import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('mtc_tutorial').find('mtc_tutorial')

    # Path to taught waypoints config
    taught_waypoints_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'taught_waypoints.yaml'
    ])

    # Declare launch arguments
    holder_list_arg = DeclareLaunchArgument(
        'holder_list',
        default_value='[1]',
        description='List of holder numbers to process (e.g., [1, 2, 3])'
    )

    holder_z_offset_arg = DeclareLaunchArgument(
        'holder_z_offset',
        default_value='-0.03',
        description='Z offset between holders in meters (default: -30mm)'
    )

    num_cycles_arg = DeclareLaunchArgument(
        'num_cycles',
        default_value='1',
        description='Number of cycles to repeat for each holder'
    )

    repeat_arg = DeclareLaunchArgument(
        'repeat',
        default_value='1',
        description='Number of times to repeat the entire holder sequence'
    )

    step_by_step_arg = DeclareLaunchArgument(
        'step_by_step',
        default_value='false',
        description='Enable step-by-step debug mode'
    )

    # Multi-holder sequence node
    multi_holder_node = Node(
        package='mtc_tutorial',
        executable='multi_holder_sequence',
        name='multi_holder_sequence',
        output='screen',
        parameters=[
            taught_waypoints_file,
            {
                'use_gripper_action': True,
                'gripper_action_name': '/gripper_action_controller/gripper_cmd',
                'gripper_open_position': 0.025,
                'gripper_close_position': 0.01,  # Changed from 0.0 to prevent finger collision
                'gripper_max_effort': 100.0,
                'use_movegroup_action': True,
                'holder_list': LaunchConfiguration('holder_list'),
                'holder_z_offset': LaunchConfiguration('holder_z_offset'),
                'num_cycles': LaunchConfiguration('num_cycles'),
                'repeat': LaunchConfiguration('repeat'),
                'step_by_step': LaunchConfiguration('step_by_step'),
                'arm_group': 'ur_arm',
                'hand_group': 'hand',
                'ik_frame': 'robotiq_hande_end',
                'hand_open': 'open',
                'hand_close': 'close',
            }
        ]
    )

    return LaunchDescription([
        holder_list_arg,
        holder_z_offset_arg,
        num_cycles_arg,
        repeat_arg,
        step_by_step_arg,
        multi_holder_node,
    ])
