#!/usr/bin/env python3
"""
Vision-Guided Pick Example

캘리브레이션된 카메라를 사용한 비전 기반 픽 앤 플레이스 예제
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import cv2
import numpy as np


class VisionGuidedPick(Node):
    def __init__(self):
        super().__init__('vision_guided_pick')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CV Bridge
        self.bridge = CvBridge()

        # Subscriber
        self.color_sub = self.create_subscription(
            Image,
            '/realsense_service_node/color/image_raw',
            self.color_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/realsense_service_node/depth/image_raw',
            self.depth_callback,
            10
        )

        # Publisher
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/vision/target_pose',
            10
        )

        self.current_color = None
        self.current_depth = None

        # 카메라 내부 파라미터 (RealSense D405)
        # TODO: 실제 카메라 캘리브레이션 값으로 교체
        self.fx = 615.0  # focal length x
        self.fy = 615.0  # focal length y
        self.cx = 424.0  # principal point x
        self.cy = 240.0  # principal point y

        self.get_logger().info('Vision Guided Pick 시작')
        self.get_logger().info('사용법:')
        self.get_logger().info('  - Color 이미지에서 물체 검출')
        self.get_logger().info('  - Depth로 3D 위치 계산')
        self.get_logger().info('  - TF로 로봇 좌표계로 변환')

    def color_callback(self, msg):
        """Color 이미지 콜백"""
        self.current_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        """Depth 이미지 콜백"""
        self.current_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')

    def detect_object(self, image):
        """
        간단한 물체 검출 예제 (빨간색 물체)

        실제 사용 시:
        - ArUco 마커 검출
        - 딥러닝 기반 물체 검출
        - 색상 기반 세그멘테이션 등으로 교체
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 (HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 윤곽선 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # 가장 큰 윤곽선
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 100:
                # 중심점 계산
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # 시각화
                    cv2.drawContours(image, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(image, (cx, cy), 5, (0, 0, 255), -1)

                    return cx, cy, image

        return None, None, image

    def pixel_to_3d_point(self, u, v, depth):
        """
        픽셀 좌표와 depth를 3D 포인트로 변환

        Args:
            u: 픽셀 x 좌표
            v: 픽셀 y 좌표
            depth: depth 값 (mm 단위)

        Returns:
            (x, y, z) 카메라 좌표계에서의 3D 포인트 (미터 단위)
        """
        # Depth를 미터로 변환
        z = depth / 1000.0

        # 픽셀 -> 3D 변환
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        return x, y, z

    def transform_to_robot_frame(self, x, y, z, source_frame='camera_color_optical_frame', target_frame='base_link'):
        """
        카메라 좌표계의 점을 로봇 좌표계로 변환

        Args:
            x, y, z: 카메라 좌표계에서의 3D 포인트
            source_frame: 소스 프레임 (카메라)
            target_frame: 타겟 프레임 (로봇 base)

        Returns:
            PoseStamped: 로봇 좌표계에서의 포즈
        """
        try:
            # PointStamped 생성
            point_camera = PointStamped()
            point_camera.header.frame_id = source_frame
            point_camera.header.stamp = self.get_clock().now().to_msg()
            point_camera.point.x = x
            point_camera.point.y = y
            point_camera.point.z = z

            # TF 조회 및 변환
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )

            point_robot = do_transform_point(point_camera, transform)

            # PoseStamped로 변환
            pose = PoseStamped()
            pose.header = point_robot.header
            pose.pose.position = point_robot.point

            # 기본 orientation (필요시 수정)
            pose.pose.orientation.w = 1.0

            return pose

        except Exception as e:
            self.get_logger().error(f'좌표 변환 실패: {e}')
            return None

    def process_frame(self):
        """프레임 처리 및 물체 위치 추정"""
        if self.current_color is None or self.current_depth is None:
            return

        # 1. 물체 검출
        cx, cy, vis_image = self.detect_object(self.current_color.copy())

        if cx is not None and cy is not None:
            # 2. Depth 값 얻기
            depth_value = self.current_depth[cy, cx]

            if depth_value > 0:  # 유효한 depth
                # 3. 픽셀 -> 3D 변환
                x_cam, y_cam, z_cam = self.pixel_to_3d_point(cx, cy, depth_value)

                # 4. 카메라 좌표계 -> 로봇 좌표계 변환
                target_pose = self.transform_to_robot_frame(x_cam, y_cam, z_cam)

                if target_pose is not None:
                    # 5. 결과 발행
                    self.target_pub.publish(target_pose)

                    # 로그
                    self.get_logger().info('='*60)
                    self.get_logger().info('물체 검출!')
                    self.get_logger().info(f'  픽셀 좌표: ({cx}, {cy})')
                    self.get_logger().info(f'  Depth: {depth_value}mm')
                    self.get_logger().info(f'  카메라 좌표: ({x_cam:.3f}, {y_cam:.3f}, {z_cam:.3f})m')
                    self.get_logger().info(f'  로봇 좌표: ({target_pose.pose.position.x:.3f}, '
                                         f'{target_pose.pose.position.y:.3f}, '
                                         f'{target_pose.pose.position.z:.3f})m')
                    self.get_logger().info('='*60)

                    # 시각화
                    cv2.putText(vis_image, f'Robot: ({target_pose.pose.position.x:.2f}, '
                               f'{target_pose.pose.position.y:.2f}, '
                               f'{target_pose.pose.position.z:.2f})',
                               (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 이미지 표시
        cv2.imshow('Vision Guided Pick', vis_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = VisionGuidedPick()

    # 타이머로 주기적 처리
    timer = node.create_timer(0.1, node.process_frame)  # 10Hz

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
