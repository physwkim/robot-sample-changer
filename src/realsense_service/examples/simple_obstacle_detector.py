#!/usr/bin/env python3
"""
MoveIt 없이 카메라로부터 장애물을 감지하는 간단한 예제

MoveIt Planning Scene 대신 장애물 정보를 커스텀 메시지나
마커로 시각화합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import cv2


class SimpleObstacleDetector(Node):
    def __init__(self):
        super().__init__('simple_obstacle_detector')

        # CV Bridge
        self.bridge = CvBridge()

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/obstacle_markers',
            10
        )

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/realsense_service_node/depth/image_raw',
            self.depth_callback,
            10
        )

        self.color_sub = self.create_subscription(
            Image,
            '/realsense_service_node/color/image_raw',
            self.color_callback,
            10
        )

        # Parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('depth_threshold_min', 0.1)  # meters
        self.declare_parameter('depth_threshold_max', 1.5)  # meters
        self.declare_parameter('min_obstacle_area', 100)  # pixels

        update_rate = self.get_parameter('update_rate').value
        self.depth_min = self.get_parameter('depth_threshold_min').value
        self.depth_max = self.get_parameter('depth_threshold_max').value
        self.min_area = self.get_parameter('min_obstacle_area').value

        # Timer for periodic processing
        self.timer = self.create_timer(1.0 / update_rate, self.process_obstacles)

        # Latest images
        self.latest_depth = None
        self.latest_color = None
        self.depth_scale = 0.001  # D405 기본값

        self.get_logger().info('Simple Obstacle Detector 시작')
        self.get_logger().info(f'업데이트 주기: {update_rate} Hz')
        self.get_logger().info(f'Depth 범위: {self.depth_min}m ~ {self.depth_max}m')

    def depth_callback(self, msg):
        """Depth 이미지 수신"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f'Depth 이미지 변환 실패: {e}')

    def color_callback(self, msg):
        """Color 이미지 수신"""
        try:
            self.latest_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Color 이미지 변환 실패: {e}')

    def process_obstacles(self):
        """장애물 감지 및 마커 발행"""
        if self.latest_depth is None:
            return

        try:
            # Depth 이미지를 meter 단위로 변환
            depth_meters = self.latest_depth.astype(np.float32) * self.depth_scale

            # 관심 영역 필터링 (depth_min ~ depth_max 범위)
            mask = np.zeros_like(depth_meters, dtype=np.uint8)
            mask[(depth_meters > self.depth_min) & (depth_meters < self.depth_max)] = 255

            # 연결된 영역(blob) 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 장애물 리스트
            obstacles = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < self.min_area:
                    continue

                # Bounding box
                x, y, w, h = cv2.boundingRect(contour)

                # 중심점의 depth 값
                center_x = x + w // 2
                center_y = y + h // 2
                obstacle_depth = depth_meters[center_y, center_x]

                if obstacle_depth > 0:
                    obstacles.append({
                        'x': center_x,
                        'y': center_y,
                        'depth': obstacle_depth,
                        'width': w,
                        'height': h,
                        'area': area
                    })

            # 마커로 시각화
            self.publish_markers(obstacles)

            if len(obstacles) > 0:
                self.get_logger().info(f'감지된 장애물: {len(obstacles)}개')
                for i, obs in enumerate(obstacles):
                    self.get_logger().info(
                        f'  #{i+1}: 픽셀({obs["x"]}, {obs["y"]}), '
                        f'거리={obs["depth"]:.2f}m, 크기={obs["area"]}px²'
                    )

        except Exception as e:
            self.get_logger().error(f'장애물 감지 실패: {e}')

    def publish_markers(self, obstacles):
        """RViz 마커로 장애물 시각화"""
        marker_array = MarkerArray()

        for i, obs in enumerate(obstacles):
            marker = Marker()
            marker.header.frame_id = 'camera_depth_optical_frame'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # 위치 (카메라 좌표계)
            # TODO: Intrinsic 파라미터로 정확한 3D 좌표 계산
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = float(obs['depth'])
            marker.pose.orientation.w = 1.0

            # 크기
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # 색상 (빨간색)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8

            marker.lifetime.sec = 2

            marker_array.markers.append(marker)

        # 이전 마커 삭제
        if len(obstacles) == 0:
            delete_marker = Marker()
            delete_marker.action = Marker.DELETEALL
            marker_array.markers.append(delete_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    detector = SimpleObstacleDetector()

    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
