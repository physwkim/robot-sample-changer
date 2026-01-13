from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur3e')
    description_package_arg = DeclareLaunchArgument('description_package', default_value='ur3e_hande_robot_description')
    description_file_arg = DeclareLaunchArgument('description_file', default_value='ur_with_hande.xacro')
    moveit_config_package_arg = DeclareLaunchArgument('moveit_config_package', default_value='ur3e_hande_moveit_config')
    moveit_config_file_arg = DeclareLaunchArgument('moveit_config_file', default_value='ur.srdf')
    
    # Move relative specific parameters
    group_arg = DeclareLaunchArgument('group', default_value='ur_arm')
    ik_frame_arg = DeclareLaunchArgument('ik_frame', default_value='robotiq_hande_end')
    direction_frame_arg = DeclareLaunchArgument('direction_frame', default_value='robotiq_hande_end')
    distance_arg = DeclareLaunchArgument('distance', default_value='0.005')
    dx_arg = DeclareLaunchArgument('dx', default_value='-1.0')
    dy_arg = DeclareLaunchArgument('dy', default_value='0.0')
    dz_arg = DeclareLaunchArgument('dz', default_value='0.0')
    step_size_arg = DeclareLaunchArgument('step_size', default_value='0.005')
    vel_scale_arg = DeclareLaunchArgument('vel_scale', default_value='0.5')
    acc_scale_arg = DeclareLaunchArgument('acc_scale', default_value='0.5')

    return LaunchDescription([
        # Launch arguments
        ur_type_arg,
        description_package_arg,
        description_file_arg,
        moveit_config_package_arg,
        moveit_config_file_arg,
        group_arg,
        ik_frame_arg,
        direction_frame_arg,
        distance_arg,
        dx_arg,
        dy_arg,
        dz_arg,
        step_size_arg,
        vel_scale_arg,
        acc_scale_arg,
        # Opaque function to generate nodes at runtime
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context, *args, **kwargs):
    # Get launch argument values
    ur_type = context.launch_configurations.get('ur_type', 'ur3e')
    description_package = context.launch_configurations.get('description_package', 'ur3e_hande_robot_description')
    description_file = context.launch_configurations.get('description_file', 'ur_with_hande.xacro')
    moveit_config_package = context.launch_configurations.get('moveit_config_package', 'ur3e_hande_moveit_config')
    moveit_config_file = context.launch_configurations.get('moveit_config_file', 'ur.srdf')
    
    # Get move relative specific parameters
    group = context.launch_configurations.get('group', 'ur_arm')
    ik_frame = context.launch_configurations.get('ik_frame', 'robotiq_hande_end')
    direction_frame = context.launch_configurations.get('direction_frame', 'robotiq_hande_end')
    distance = context.launch_configurations.get('distance', '0.005')
    dx = context.launch_configurations.get('dx', '-1.0')
    dy = context.launch_configurations.get('dy', '0.0')
    dz = context.launch_configurations.get('dz', '0.0')
    step_size = context.launch_configurations.get('step_size', '0.005')
    vel_scale = context.launch_configurations.get('vel_scale', '0.5')
    acc_scale = context.launch_configurations.get('acc_scale', '0.5')
    
    # Get package share directories
    description_package_share = get_package_share_directory(description_package)
    moveit_config_package_share = get_package_share_directory(moveit_config_package)
    
    # Build file paths
    description_file_path = os.path.join(description_package_share, 'urdf', description_file)
    srdf_file_path = os.path.join(moveit_config_package_share, 'srdf', moveit_config_file)
    kinematics_file_path = os.path.join(moveit_config_package_share, 'config', 'kinematics.yaml')
    joint_limits_file_path = os.path.join(moveit_config_package_share, 'config', 'joint_limits.yaml')

    # Xacro mappings
    xacro_mappings = {
        "ur_type": ur_type,
        "description_package": description_package,
        "description_file": description_file,
    }

    # Load MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name=moveit_config_package)
        .robot_description(
            file_path=description_file_path,
            mappings=xacro_mappings,
        )
        .robot_description_semantic(file_path=srdf_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        .to_moveit_configs()
    )

    # Load ExecuteTaskSolutionCapability so we can execute MTC solutions
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the move_group node/action server with MTC capability
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # Move relative node
    move_relative_node = Node(
        package="mtc_tutorial",
        executable="move_relative_mtc",
        name="move_relative_mtc",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "group": group,
                "ik_frame": ik_frame,
                "direction_frame": direction_frame,
                "distance": float(distance),
                "dx": float(dx),
                "dy": float(dy),
                "dz": float(dz),
                "step_size": float(step_size),
                "vel_scale": float(vel_scale),
                "acc_scale": float(acc_scale),
            },
        ],
    )

    return [move_group_node, move_relative_node]
