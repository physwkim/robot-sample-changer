#!/usr/bin/env python3
"""
RealSense Service 클라이언트 예제

이 스크립트는 RealSense 서비스를 호출하여 이미지를 캡처하는 방법을 보여줍니다.
"""

import rclpy
from rclpy.node import Node
from realsense_service.srv import CaptureImage, SetCameraState
from cv_bridge import CvBridge
import cv2
import sys


class RealSenseClient(Node):
    def __init__(self):
        super().__init__('realsense_client')

        # 서비스 클라이언트 생성
        self.capture_client = self.create_client(
            CaptureImage,
            'capture_image'
        )
        self.state_client = self.create_client(
            SetCameraState,
            'set_camera_state'
        )

        self.bridge = CvBridge()

        # 서비스 대기
        while not self.capture_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스를 기다리는 중...')

    def start_camera(self):
        """카메라 시작"""
        request = SetCameraState.Request()
        request.start = True

        future = self.state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'카메라 시작: {response.message}')
            return response.success
        else:
            self.get_logger().error('서비스 호출 실패')
            return False

    def stop_camera(self):
        """카메라 정지"""
        request = SetCameraState.Request()
        request.start = False

        future = self.state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'카메라 정지: {response.message}')
            return response.success
        else:
            self.get_logger().error('서비스 호출 실패')
            return False

    def capture_image(self, enable_color=True, enable_depth=True, width=848, height=480):
        """이미지 캡처"""
        request = CaptureImage.Request()
        request.enable_color = enable_color
        request.enable_depth = enable_depth
        request.width = width
        request.height = height

        self.get_logger().info('이미지 캡처 요청 중...')
        future = self.capture_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'응답: {response.message}')

            if response.success:
                # Color 이미지 처리
                if enable_color and len(response.color_image.data) > 0:
                    color_image = self.bridge.imgmsg_to_cv2(
                        response.color_image,
                        desired_encoding='bgr8'
                    )
                    cv2.imshow('Color Image', color_image)
                    cv2.imwrite('color_image.png', color_image)
                    self.get_logger().info('Color 이미지 저장: color_image.png')

                # Depth 이미지 처리
                if enable_depth and len(response.depth_image.data) > 0:
                    depth_image = self.bridge.imgmsg_to_cv2(
                        response.depth_image,
                        desired_encoding='16UC1'
                    )
                    # Depth를 시각화하기 위해 정규화
                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.03),
                        cv2.COLORMAP_JET
                    )
                    cv2.imshow('Depth Image', depth_colormap)
                    cv2.imwrite('depth_image.png', depth_colormap)
                    self.get_logger().info('Depth 이미지 저장: depth_image.png')

                if enable_color or enable_depth:
                    self.get_logger().info('아무 키나 눌러서 종료...')
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

                return True
            else:
                self.get_logger().error('이미지 캡처 실패')
                return False
        else:
            self.get_logger().error('서비스 호출 실패')
            return False


def main(args=None):
    rclpy.init(args=args)
    client = RealSenseClient()

    try:
        # 사용 예제
        print("\n=== RealSense Service 클라이언트 예제 ===\n")

        # 1. 카메라 시작
        print("1. 카메라 시작")
        client.start_camera()

        # 2. 이미지 캡처 (Color + Depth)
        print("\n2. Color + Depth 이미지 캡처")
        client.capture_image(enable_color=True, enable_depth=True)

        # 3. Color 이미지만 캡처
        print("\n3. Color 이미지만 캡처")
        client.capture_image(enable_color=True, enable_depth=False)

        # 4. Depth 이미지만 캡처
        print("\n4. Depth 이미지만 캡처")
        client.capture_image(enable_color=False, enable_depth=True)

        # 5. 카메라 정지
        print("\n5. 카메라 정지")
        client.stop_camera()

        print("\n완료!")

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
