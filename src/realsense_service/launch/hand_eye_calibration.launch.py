from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch.conditions


def generate_launch_description():
    # Launch 인자 선언
    marker_type_arg = DeclareLaunchArgument(
        'marker_type',
        default_value='aruco',
        description='Marker type: aruco or checkerboard'
    )

    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='DICT_6X6_250',
        description='ArUco dictionary type'
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.05',
        description='ArUco marker size in meters'
    )

    min_samples_arg = DeclareLaunchArgument(
        'min_samples',
        default_value='10',
        description='Minimum number of calibration samples'
    )

    pose_source_arg = DeclareLaunchArgument(
        'pose_source',
        default_value='tf',
        description='Robot pose source: tf, topic, or joint_states'
    )

    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='base_link',
        description='Robot base frame (for TF mode)'
    )

    robot_ee_frame_arg = DeclareLaunchArgument(
        'robot_ee_frame',
        default_value='wrist_3_link',
        description='Robot end-effector frame (for TF mode) - 카메라가 실제로 부착된 링크'
    )

    use_test_robot_arg = DeclareLaunchArgument(
        'use_test_robot',
        default_value='false',
        description='Use test robot pose publisher (for testing without real robot)'
    )

    # RealSense 서비스 노드
    realsense_node = Node(
        package='realsense_service',
        executable='realsense_service_node',
        name='realsense_service_node',
        output='screen',
        parameters=[{
            'enable_streaming': True,
            'publish_rate': 30.0,
            'auto_start': True,
        }]
    )

    # Hand-Eye 캘리브레이션 노드
    calibration_node = Node(
        package='realsense_service',
        executable='hand_eye_calibration_node',
        name='hand_eye_calibration_node',
        output='screen',
        parameters=[{
            'marker_type': LaunchConfiguration('marker_type'),
            'aruco_dict': LaunchConfiguration('aruco_dict'),
            'marker_size': LaunchConfiguration('marker_size'),
            'min_samples': LaunchConfiguration('min_samples'),
            'pose_source': LaunchConfiguration('pose_source'),
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_ee_frame': LaunchConfiguration('robot_ee_frame'),
        }]
    )

    # 테스트용 로봇 포즈 발행 노드 (선택적)
    test_robot_node = Node(
        package='realsense_service',
        executable='robot_pose_publisher',
        name='robot_pose_publisher',
        output='screen',
        condition=launch.conditions.IfCondition(
            LaunchConfiguration('use_test_robot')
        )
    )

    return LaunchDescription([
        marker_type_arg,
        aruco_dict_arg,
        marker_size_arg,
        min_samples_arg,
        pose_source_arg,
        robot_base_frame_arg,
        robot_ee_frame_arg,
        use_test_robot_arg,
        realsense_node,
        calibration_node,
        # test_robot_node,  # 필요시 주석 해제
    ])
