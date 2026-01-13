from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense_service',
            executable='realsense_service_node',
            name='realsense_service_node',
            output='screen',
            parameters=[
                # 필요한 경우 여기에 파라미터 추가
            ]
        )
    ])
