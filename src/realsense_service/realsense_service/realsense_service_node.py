#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np

from realsense_service.srv import CaptureImage, SetCameraState


class RealSenseServiceNode(Node):
    def __init__(self):
        super().__init__('realsense_service_node')

        # RealSense 파이프라인 초기화
        self.pipeline = None
        self.config = None
        self.profile = None
        self.is_streaming = False
        self.bridge = CvBridge()

        # Depth 필터 (노이즈 감소)
        self.spatial_filter = rs.spatial_filter()
        self.temporal_filter = rs.temporal_filter()
        self.hole_filling_filter = rs.hole_filling_filter()

        # 기본 해상도 설정
        self.default_width = 848
        self.default_height = 480
        self.default_fps = 30

        # 카메라 intrinsic 파라미터
        self.cam_matrix_depth = None
        self.cam_matrix_color = None
        self.cam_params_depth = None
        self.cam_params_color = None
        self.depth_scale = None

        # 파라미터 선언
        self.declare_parameter('enable_streaming', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('auto_start', True)

        # 파라미터 읽기
        self.enable_streaming = self.get_parameter('enable_streaming').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.auto_start = self.get_parameter('auto_start').value

        # 토픽 퍼블리셔 생성
        self.color_pub = self.create_publisher(Image, '~/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '~/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, '~/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, '~/depth/camera_info', 10)

        # 서비스 생성
        self.capture_srv = self.create_service(
            CaptureImage,
            'capture_image',
            self.capture_image_callback
        )

        self.state_srv = self.create_service(
            SetCameraState,
            'set_camera_state',
            self.set_camera_state_callback
        )

        # 타이머 생성 (스트리밍 모드용)
        self.timer = None
        if self.enable_streaming:
            timer_period = 1.0 / self.publish_rate
            self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info('RealSense Service Node가 시작되었습니다.')
        self.get_logger().info('사용 가능한 서비스:')
        self.get_logger().info('  - /capture_image (CaptureImage)')
        self.get_logger().info('  - /set_camera_state (SetCameraState)')
        self.get_logger().info('발행 토픽:')
        self.get_logger().info('  - ~/color/image_raw (sensor_msgs/Image)')
        self.get_logger().info('  - ~/depth/image_raw (sensor_msgs/Image)')
        self.get_logger().info(f'스트리밍 모드: {"활성화" if self.enable_streaming else "비활성화"}')

        # 자동 시작
        if self.auto_start:
            self.get_logger().info('카메라 자동 시작 중...')
            self.initialize_camera()

    def initialize_camera(self, width=None, height=None):
        """카메라 초기화"""
        try:
            if self.pipeline is not None:
                self.get_logger().warn('카메라가 이미 초기화되어 있습니다.')
                return True

            self.pipeline = rs.pipeline()
            self.config = rs.config()

            # 해상도 설정
            w = width if width and width > 0 else self.default_width
            h = height if height and height > 0 else self.default_height

            # 스트림 설정
            self.config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, self.default_fps)
            self.config.enable_stream(rs.stream.depth, w, h, rs.format.z16, self.default_fps)

            # 파이프라인 시작
            self.profile = self.pipeline.start(self.config)
            self.is_streaming = True

            # 카메라 intrinsic 파라미터 가져오기
            self._get_camera_intrinsics()

            self.get_logger().info(f'RealSense 카메라 초기화 완료 ({w}x{h})')
            self.get_logger().info(f'Depth Scale: {self.depth_scale}')
            self.get_logger().info(f'Color Camera Matrix:\n{self.cam_matrix_color}')
            self.get_logger().info(f'Depth Camera Matrix:\n{self.cam_matrix_depth}')

            return True

        except Exception as e:
            self.get_logger().error(f'카메라 초기화 실패: {str(e)}')
            return False

    def _get_camera_intrinsics(self):
        """카메라 intrinsic 파라미터 가져오기 및 매트릭스 구성"""
        try:
            # Depth 스트림 intrinsic 파라미터
            stream_depth = self.profile.get_stream(rs.stream.depth)
            intrinsic_depth = stream_depth.as_video_stream_profile().get_intrinsics()

            # Color 스트림 intrinsic 파라미터
            stream_color = self.profile.get_stream(rs.stream.color)
            intrinsic_color = stream_color.as_video_stream_profile().get_intrinsics()

            # Depth scale 가져오기
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()

            # Depth 카메라 파라미터 및 매트릭스 구성
            self.cam_params_depth = [
                intrinsic_depth.fx,
                intrinsic_depth.fy,
                intrinsic_depth.ppx,
                intrinsic_depth.ppy
            ]
            self.cam_matrix_depth = np.array([
                [intrinsic_depth.fx, 0, intrinsic_depth.ppx],
                [0, intrinsic_depth.fy, intrinsic_depth.ppy],
                [0, 0, 1]
            ])

            # Color 카메라 파라미터 및 매트릭스 구성
            self.cam_params_color = [
                intrinsic_color.fx,
                intrinsic_color.fy,
                intrinsic_color.ppx,
                intrinsic_color.ppy
            ]
            self.cam_matrix_color = np.array([
                [intrinsic_color.fx, 0, intrinsic_color.ppx],
                [0, intrinsic_color.fy, intrinsic_color.ppy],
                [0, 0, 1]
            ])

            # 해상도 정보 로깅
            self.get_logger().info(f'Color 스트림 해상도: {intrinsic_color.width}x{intrinsic_color.height}')
            self.get_logger().info(f'Depth 스트림 해상도: {intrinsic_depth.width}x{intrinsic_depth.height}')
            self.get_logger().info(f'Color Principal Point: ({intrinsic_color.ppx:.2f}, {intrinsic_color.ppy:.2f}) - 중심: ({intrinsic_color.width/2}, {intrinsic_color.height/2})')
            self.get_logger().info(f'Depth Principal Point: ({intrinsic_depth.ppx:.2f}, {intrinsic_depth.ppy:.2f}) - 중심: ({intrinsic_depth.width/2}, {intrinsic_depth.height/2})')
            self.get_logger().info('카메라 intrinsic 파라미터 로드 완료')

        except Exception as e:
            self.get_logger().error(f'Intrinsic 파라미터 로드 실패: {str(e)}')

    def get_camera_matrix(self, camera_type='color'):
        """카메라 매트릭스 반환

        Args:
            camera_type (str): 'color' 또는 'depth'

        Returns:
            numpy.ndarray: 3x3 카메라 intrinsic 매트릭스
        """
        if camera_type == 'color':
            return self.cam_matrix_color
        elif camera_type == 'depth':
            return self.cam_matrix_depth
        else:
            self.get_logger().error(f'잘못된 카메라 타입: {camera_type}')
            return None

    def get_camera_params(self, camera_type='color'):
        """카메라 파라미터 반환 [fx, fy, ppx, ppy]

        Args:
            camera_type (str): 'color' 또는 'depth'

        Returns:
            list: [fx, fy, ppx, ppy]
        """
        if camera_type == 'color':
            return self.cam_params_color
        elif camera_type == 'depth':
            return self.cam_params_depth
        else:
            self.get_logger().error(f'잘못된 카메라 타입: {camera_type}')
            return None

    def get_depth_scale(self):
        """Depth scale 반환

        Returns:
            float: depth scale (meter 단위 변환 계수)
        """
        return self.depth_scale

    def create_camera_info_msg(self, camera_type='color'):
        """CameraInfo 메시지 생성

        Args:
            camera_type (str): 'color' 또는 'depth'

        Returns:
            CameraInfo: 카메라 정보 메시지
        """
        if camera_type == 'color':
            params = self.cam_params_color
            frame_id = 'camera_link_color_optical_frame'
            stream = self.profile.get_stream(rs.stream.color)
        else:
            params = self.cam_params_depth
            frame_id = 'camera_link_depth_optical_frame'
            stream = self.profile.get_stream(rs.stream.depth)

        intrinsic = stream.as_video_stream_profile().get_intrinsics()

        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.width = intrinsic.width
        msg.height = intrinsic.height
        msg.distortion_model = 'plumb_bob'

        # K: 3x3 intrinsic camera matrix
        msg.k = [
            params[0], 0.0, params[2],
            0.0, params[1], params[3],
            0.0, 0.0, 1.0
        ]

        # D: distortion coefficients [k1, k2, t1, t2, k3]
        msg.d = [intrinsic.coeffs[0], intrinsic.coeffs[1], intrinsic.coeffs[2],
                 intrinsic.coeffs[3], intrinsic.coeffs[4]]

        # R: 3x3 rectification matrix (identity for non-stereo)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # P: 3x4 projection matrix
        msg.p = [
            params[0], 0.0, params[2], 0.0,
            0.0, params[1], params[3], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return msg

    def stop_camera(self):
        """카메라 정지"""
        try:
            if self.pipeline is not None and self.is_streaming:
                self.pipeline.stop()
                self.is_streaming = False
                self.get_logger().info('RealSense 카메라 정지됨')
                return True
            else:
                self.get_logger().warn('카메라가 실행 중이지 않습니다.')
                return False
        except Exception as e:
            self.get_logger().error(f'카메라 정지 실패: {str(e)}')
            return False

    def capture_image_callback(self, request, response):
        """이미지 캡처 서비스 콜백"""
        self.get_logger().info('이미지 캡처 요청 받음')

        try:
            # 카메라가 초기화되지 않았으면 초기화
            if not self.is_streaming:
                if not self.initialize_camera(request.width, request.height):
                    response.success = False
                    response.message = '카메라 초기화 실패'
                    return response

            # 프레임 대기 (안정화를 위해 몇 프레임 건너뛰기)
            for _ in range(5):
                frames = self.pipeline.wait_for_frames()

            # 최종 프레임 가져오기
            frames = self.pipeline.wait_for_frames()

            # Color 이미지 처리
            if request.enable_color:
                color_frame = frames.get_color_frame()
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())
                    response.color_image = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
                    self.get_logger().info('Color 이미지 캡처 완료')

            # Depth 이미지 처리
            if request.enable_depth:
                depth_frame = frames.get_depth_frame()
                if depth_frame:
                    # 노이즈 감소 필터 적용
                    depth_frame = self.spatial_filter.process(depth_frame)
                    depth_frame = self.temporal_filter.process(depth_frame)
                    depth_frame = self.hole_filling_filter.process(depth_frame)

                    depth_image = np.asanyarray(depth_frame.get_data())

                    # Depth를 16UC1로 인코딩
                    response.depth_image = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
                    self.get_logger().info('Depth 이미지 캡처 완료')

            response.success = True
            response.message = '이미지 캡처 성공'

        except Exception as e:
            self.get_logger().error(f'이미지 캡처 실패: {str(e)}')
            response.success = False
            response.message = f'이미지 캡처 실패: {str(e)}'

        return response

    def set_camera_state_callback(self, request, response):
        """카메라 상태 변경 서비스 콜백"""
        if request.start:
            self.get_logger().info('카메라 시작 요청')
            if self.initialize_camera():
                response.success = True
                response.message = '카메라가 시작되었습니다'
            else:
                response.success = False
                response.message = '카메라 시작 실패'
        else:
            self.get_logger().info('카메라 정지 요청')
            if self.stop_camera():
                response.success = True
                response.message = '카메라가 정지되었습니다'
            else:
                response.success = False
                response.message = '카메라 정지 실패'

        return response

    def timer_callback(self):
        """타이머 콜백 - 주기적으로 이미지 발행"""
        if not self.is_streaming:
            return

        try:
            # 프레임 가져오기
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            timestamp = self.get_clock().now().to_msg()

            # Color 이미지 발행
            color_frame = frames.get_color_frame()
            if color_frame:
                color_image = np.asanyarray(color_frame.get_data())
                color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
                color_msg.header.stamp = timestamp
                color_msg.header.frame_id = 'camera_link_color_optical_frame'
                self.color_pub.publish(color_msg)

                # Color CameraInfo 발행
                color_info = self.create_camera_info_msg(camera_type='color')
                color_info.header.stamp = timestamp
                self.color_info_pub.publish(color_info)

            # Depth 이미지 발행
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                # 노이즈 감소 필터 적용
                depth_frame = self.spatial_filter.process(depth_frame)
                depth_frame = self.temporal_filter.process(depth_frame)
                depth_frame = self.hole_filling_filter.process(depth_frame)

                depth_image = np.asanyarray(depth_frame.get_data())

                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
                depth_msg.header.stamp = timestamp
                depth_msg.header.frame_id = 'camera_link_depth_optical_frame'
                self.depth_pub.publish(depth_msg)

                # Depth CameraInfo 발행
                depth_info = self.create_camera_info_msg(camera_type='depth')
                depth_info.header.stamp = timestamp
                self.depth_info_pub.publish(depth_info)

        except Exception as e:
            self.get_logger().warn(f'프레임 가져오기 실패: {str(e)}')

    def __del__(self):
        """소멸자"""
        if self.pipeline is not None and self.is_streaming:
            self.pipeline.stop()
            self.get_logger().info('RealSense 카메라 종료')


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
