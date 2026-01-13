import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('mtc_tutorial').find('mtc_tutorial')

    # Path to taught waypoints config (absolute path for runtime reloading)
    taught_waypoints_file = os.path.join(pkg_share, 'config', 'taught_waypoints.yaml')

    # Declare launch arguments
    epics_trigger_pv_arg = DeclareLaunchArgument(
        'epics_trigger_pv',
        default_value='Robot:Trigger',
        description='EPICS PV name for trigger signal'
    )

    epics_start_step_pv_arg = DeclareLaunchArgument(
        'epics_start_step_pv',
        default_value='Robot:StartStep',
        description='EPICS PV name for start step number'
    )

    epics_wait_pv_arg = DeclareLaunchArgument(
        'epics_wait_pv',
        default_value='Robot:Wait',
        description='EPICS PV for measurement wait (0=wait, 1=continue, 2=skip remaining steps)'
    )

    epics_holder_pv_arg = DeclareLaunchArgument(
        'epics_holder_pv',
        default_value='Robot:Holder',
        description='EPICS PV for holder number (1-10)'
    )

    epics_stop_pv_arg = DeclareLaunchArgument(
        'epics_stop_pv',
        default_value='Robot:Stop',
        description='EPICS PV for pause/resume (1=pause before next step, 0=resume)'
    )

    epics_current_step_pv_arg = DeclareLaunchArgument(
        'epics_current_step_pv',
        default_value='Robot:CurrentStep',
        description='EPICS PV for current step number (updated after each step completes)'
    )

    epics_gripper_pv_arg = DeclareLaunchArgument(
        'epics_gripper_pv',
        default_value='Robot:Gripper',
        description='EPICS PV for gripper state (0=close, 1=open)'
    )

    epics_pause_step_pv_arg = DeclareLaunchArgument(
        'epics_pause_step_pv',
        default_value='Robot:PauseStep',
        description='EPICS PV for pausing at specific step (0=no pause, N=pause after step N until value changes)'
    )

    holder_offset_arg = DeclareLaunchArgument(
        'holder_offset',
        default_value='0.03',
        description='Y offset between holders in meters (default: 30mm)'
    )

    waypoints_yaml_path_arg = DeclareLaunchArgument(
        'waypoints_yaml_path',
        default_value=taught_waypoints_file,
        description='Path to taught waypoints YAML file (will be reloaded on each trigger)'
    )

    # EPICS triggered sequence node
    epics_triggered_node = Node(
        package='mtc_tutorial',
        executable='epics_triggered_sequence',
        name='epics_triggered_sequence',
        output='screen',
        parameters=[
            {
                'epics_trigger_pv': LaunchConfiguration('epics_trigger_pv'),
                'epics_start_step_pv': LaunchConfiguration('epics_start_step_pv'),
                'epics_wait_pv': LaunchConfiguration('epics_wait_pv'),
                'epics_holder_pv': LaunchConfiguration('epics_holder_pv'),
                'epics_stop_pv': LaunchConfiguration('epics_stop_pv'),
                'epics_current_step_pv': LaunchConfiguration('epics_current_step_pv'),
                'epics_gripper_pv': LaunchConfiguration('epics_gripper_pv'),
                'epics_pause_step_pv': LaunchConfiguration('epics_pause_step_pv'),
                'waypoints_yaml_path': LaunchConfiguration('waypoints_yaml_path'),
                'use_gripper_action': True,
                'gripper_action_name': '/gripper_action_controller/gripper_cmd',
                'gripper_open_position': 0.025,
                'gripper_close_position': 0.01,
                'gripper_max_effort': 100.0,
                'gripper_open_threshold': 0.02,  # threshold for gripper open/close detection
                'holder_offset': LaunchConfiguration('holder_offset'),
                'arm_group': 'ur_arm',
                'hand_group': 'hand',
                'ik_frame': 'robotiq_hande_end',
                'hand_open': 'open',
                'hand_close': 'close',
            }
        ]
    )

    # Set log format to hide ROS timestamp (we'll use human-readable timestamps in code)
    set_log_format = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_OUTPUT_FORMAT',
        '[{severity}] [{name}]: {message}'
    )

    return LaunchDescription([
        set_log_format,
        epics_trigger_pv_arg,
        epics_start_step_pv_arg,
        epics_wait_pv_arg,
        epics_holder_pv_arg,
        epics_stop_pv_arg,
        epics_current_step_pv_arg,
        epics_gripper_pv_arg,
        epics_pause_step_pv_arg,
        holder_offset_arg,
        waypoints_yaml_path_arg,
        epics_triggered_node,
    ])
