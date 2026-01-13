from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 패키지 경로
    pkg_share = get_package_share_directory('realsense_service')
    rviz_config = os.path.join(pkg_share, 'rviz', 'realsense.rviz')

    # Launch 인자 선언
    enable_streaming_arg = DeclareLaunchArgument(
        'enable_streaming',
        default_value='true',
        description='Enable continuous image streaming'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='6.0',
        description='Image publishing rate (Hz) - 5 프레임 평균 고려'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        description='Auto start camera on node startup'
    )

    # RealSense 서비스 노드
    realsense_node = Node(
        package='realsense_service',
        executable='realsense_service_node',
        name='realsense_service_node',
        output='screen',
        parameters=[{
            'enable_streaming': LaunchConfiguration('enable_streaming'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    # RViz 노드
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        enable_streaming_arg,
        publish_rate_arg,
        auto_start_arg,
        realsense_node,
        rviz_node
    ])
