#!/usr/bin/env python3
"""
RealSense 카메라 + Hand-Eye Calibration TF + RViz를 동시에 실행하는 Launch 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 파라미터
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='Hand-Eye calibration YAML 파일 경로 (비어있으면 최신 파일 자동 선택)'
    )

    parent_frame_arg = DeclareLaunchArgument(
        'parent_frame',
        default_value='tool0',
        description='Camera의 parent frame (일반적으로 tool0 또는 ee_link)'
    )

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='d405',
        description='RealSense 카메라 모델 (d405, d435i 등)'
    )

    # RViz config 파일 경로
    rviz_config = PathJoinSubstitution([
        FindPackageShare('realsense_service'),
        'config',
        'camera_calibration_view.rviz'
    ])

    # 1. RealSense Service Node
    realsense_node = Node(
        package='realsense_service',
        executable='realsense_service_node',
        name='realsense_service_node',
        output='screen',
        parameters=[
            {'enable_streaming': True},
            {'publish_rate': 30.0},
            {'auto_start': True}
        ]
    )

    # 2. Camera TF Broadcaster (Hand-Eye Calibration 결과)
    camera_tf_broadcaster = Node(
        package='realsense_service',
        executable='camera_tf_broadcaster',
        name='camera_tf_broadcaster',
        output='screen',
        parameters=[
            {'calibration_file': LaunchConfiguration('calibration_file')},
            {'parent_frame': LaunchConfiguration('parent_frame')},
            {'camera_model': LaunchConfiguration('camera_model')},
            {'publish_rate': 50.0}
        ]
    )

    # 3. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        calibration_file_arg,
        parent_frame_arg,
        camera_model_arg,
        realsense_node,
        camera_tf_broadcaster,
        rviz_node
    ])
