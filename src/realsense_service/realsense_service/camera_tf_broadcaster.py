#!/usr/bin/env python3
"""
Camera TF Broadcaster

Hand-eye 캘리브레이션 결과를 로드하여 카메라의 TF를 발행하는 노드
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import numpy as np
import yaml
import os


class CameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tf_broadcaster')

        # 파라미터
        self.declare_parameter('calibration_file', '~/calibration_data/hand_eye_calibration_latest.yaml')
        self.declare_parameter('parent_frame', 'tool0')  # end-effector 프레임
        self.declare_parameter('camera_frame', 'camera_link')  # 카메라 프레임
        self.declare_parameter('camera_model', 'd405')  # d405, d435, d455 등
        self.declare_parameter('publish_rate', 10.0)  # Hz - TF는 10Hz로 충분
        self.declare_parameter('publish_optical_frames', True)  # optical 프레임도 발행할지
        self.declare_parameter('calibration_is_camera_to_gripper', False)  # Inverse 하지 않음

        # 파라미터 읽기
        calibration_file = os.path.expanduser(self.get_parameter('calibration_file').value)
        self.parent_frame = self.get_parameter('parent_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.camera_model = self.get_parameter('camera_model').value
        publish_rate = self.get_parameter('publish_rate').value
        self.publish_optical_frames = self.get_parameter('publish_optical_frames').value
        calibration_is_camera_to_gripper = self.get_parameter('calibration_is_camera_to_gripper').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Marker publisher (카메라 시각화)
        self.marker_pub = self.create_publisher(Marker, '/camera_visualization', 10)

        # 캘리브레이션 결과 로드
        self.transform_matrix = self.load_calibration(calibration_file)

        if self.transform_matrix is None:
            self.get_logger().error('캘리브레이션 파일을 로드할 수 없습니다')
            raise RuntimeError('캘리브레이션 로드 실패')

        if calibration_is_camera_to_gripper:
            self.transform_matrix = np.linalg.inv(self.transform_matrix)

        # 타이머 설정
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.broadcast_timer_callback)

        self.get_logger().info('Camera TF Broadcaster 시작')
        self.get_logger().info(f'Parent frame: {self.parent_frame}')
        self.get_logger().info(f'Camera frame: {self.camera_frame}')
        self.get_logger().info(f'Camera model: {self.camera_model}')
        self.get_logger().info(f'Publish rate: {publish_rate} Hz')
        self.print_transform_info(calibration_is_camera_to_gripper)

    def load_calibration(self, filepath):
        """캘리브레이션 파일 로드"""
        try:
            # 파일이 없으면 가장 최신 파일 찾기
            if not os.path.exists(filepath) or 'latest' in filepath:
                directory = os.path.dirname(filepath) or os.path.expanduser('~/calibration_data')
                if os.path.exists(directory):
                    files = [f for f in os.listdir(directory) if f.startswith('hand_eye_calibration_') and f.endswith('.yaml')]
                    if files:
                        files.sort(reverse=True)
                        filepath = os.path.join(directory, files[0])
                        self.get_logger().info(f'최신 캘리브레이션 파일 사용: {filepath}')

            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)

            transform_matrix = np.array(data['transform_matrix'])
            self.get_logger().info(f'캘리브레이션 로드 성공: {filepath}')
            self.get_logger().info(f'캘리브레이션 시간: {data.get("calibration_time", "N/A")}')
            self.get_logger().info(f'샘플 수: {data.get("num_samples", "N/A")}')

            return transform_matrix

        except Exception as e:
            self.get_logger().error(f'캘리브레이션 로드 실패: {e}')
            return None

    def matrix_to_transform_stamped(self, matrix, parent_frame, child_frame):
        """4x4 변환 행렬을 TransformStamped로 변환"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame

        # Translation
        transform.transform.translation.x = matrix[0, 3]
        transform.transform.translation.y = matrix[1, 3]
        transform.transform.translation.z = matrix[2, 3]

        # Rotation (rotation matrix를 quaternion으로 변환)
        R = matrix[:3, :3]
        q = self.rotation_matrix_to_quaternion(R)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        return transform

    def rotation_matrix_to_quaternion(self, R):
        """Rotation matrix를 quaternion으로 변환"""
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s

        return np.array([x, y, z, w])

    def broadcast_timer_callback(self):
        """TF 발행 타이머 콜백"""
        # 1. camera_link 발행 (end-effector -> camera_link)
        camera_tf = self.matrix_to_transform_stamped(
            self.transform_matrix,
            self.parent_frame,
            self.camera_frame
        )
        self.tf_broadcaster.sendTransform(camera_tf)

        # 2. optical 프레임 발행 (ROS 관례: optical 프레임은 Z forward, Y down)
        if self.publish_optical_frames:
            # camera_link -> camera_color_optical_frame
            color_optical_tf = TransformStamped()
            color_optical_tf.header.stamp = self.get_clock().now().to_msg()
            color_optical_tf.header.frame_id = self.camera_frame
            color_optical_tf.child_frame_id = f'{self.camera_frame}_color_optical_frame'

            # Optical frame = camera_link (회전 없음, Identity)
            # 카메라가 TCP Z축 방향을 향함
            color_optical_tf.transform.rotation.x = 0.0
            color_optical_tf.transform.rotation.y = 0.0
            color_optical_tf.transform.rotation.z = 0.0
            color_optical_tf.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(color_optical_tf)

            # camera_link -> camera_depth_optical_frame
            depth_optical_tf = TransformStamped()
            depth_optical_tf.header.stamp = self.get_clock().now().to_msg()
            depth_optical_tf.header.frame_id = self.camera_frame
            depth_optical_tf.child_frame_id = f'{self.camera_frame}_depth_optical_frame'
            # Optical frame = camera_link (회전 없음, Identity)
            # 카메라가 TCP Z축 방향을 향함
            depth_optical_tf.transform.rotation.x = 0.0
            depth_optical_tf.transform.rotation.y = 0.0
            depth_optical_tf.transform.rotation.z = 0.0
            depth_optical_tf.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(depth_optical_tf)

        # 3. 카메라 박스 시각화 Marker 발행
        self.publish_camera_marker()

    def publish_camera_marker(self):
        """카메라를 박스 Marker로 시각화"""
        marker = Marker()
        marker.header.frame_id = self.camera_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "camera"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # RealSense D405 실제 크기 (42mm x 28mm x 22mm)
        marker.scale.x = 0.042  # 가로 42mm
        marker.scale.y = 0.042  # 세로 28mm
        marker.scale.z = 0.023  # 높이 22mm

        # 위치 (camera_link 중심)
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        # 색상 (회색, 반투명)
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.color.a = 0.8

        marker.lifetime = rclpy.duration.Duration(seconds=0.2).to_msg()

        self.marker_pub.publish(marker)

    def print_transform_info(self, calibration_is_camera_to_gripper):
        """변환 정보 출력"""
        translation = self.transform_matrix[:3, 3]

        self.get_logger().info('='*60)
        if calibration_is_camera_to_gripper:
            self.get_logger().info('Camera Transform (gripper_to_camera, inverted):')
        else:
            self.get_logger().info('Camera Transform (gripper_to_camera):')
        self.get_logger().info(f'  Translation: X={translation[0]:.4f}m, Y={translation[1]:.4f}m, Z={translation[2]:.4f}m')

        # RPY 계산
        R = self.transform_matrix[:3, :3]
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

        self.get_logger().info(f'  Rotation (RPY): Roll={np.rad2deg(roll):.2f}°, Pitch={np.rad2deg(pitch):.2f}°, Yaw={np.rad2deg(yaw):.2f}°')
        self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
