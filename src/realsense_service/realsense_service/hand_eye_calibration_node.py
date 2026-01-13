#!/usr/bin/env python3
"""
Hand-Eye Calibration Node for RealSense Camera

이 노드는 로봇 팔에 장착된 RealSense 카메라의 hand-eye 캘리브레이션을 수행합니다.
ArUco 마커 또는 체커보드를 사용하여 카메라와 로봇 base 간의 변환을 계산합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import cv2
import cv2.aruco as aruco
import numpy as np
import yaml
import os
from datetime import datetime


class HandEyeCalibrationNode(Node):
    def __init__(self):
        super().__init__('hand_eye_calibration_node')

        # 파라미터 선언
        self.declare_parameter('marker_type', 'aruco')  # 'aruco' 또는 'checkerboard'
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')
        self.declare_parameter('marker_size', 0.05)  # 마커 크기 (미터)
        self.declare_parameter('checkerboard_rows', 6)
        self.declare_parameter('checkerboard_cols', 9)
        self.declare_parameter('checkerboard_square_size', 0.025)  # 체커보드 사각형 크기 (미터)
        self.declare_parameter('min_samples', 10)  # 최소 샘플 수
        self.declare_parameter('calibration_method', 'Tsai-Lenz')  # Tsai-Lenz, Park, Horaud, Andreff, Daniilidis
        self.declare_parameter('save_directory', '~/calibration_data')

        # 로봇 포즈 획득 방법 파라미터
        self.declare_parameter('pose_source', 'tf')  # 'tf', 'topic', 'joint_states'
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('robot_ee_frame', 'wrist_3_link')
        self.declare_parameter('robot_pose_topic', '/robot/end_effector_pose')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('invert_gripper2base', False)

        # 파라미터 읽기
        self.marker_type = self.get_parameter('marker_type').value
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.checkerboard_rows = self.get_parameter('checkerboard_rows').value
        self.checkerboard_cols = self.get_parameter('checkerboard_cols').value
        self.checkerboard_square_size = self.get_parameter('checkerboard_square_size').value
        self.min_samples = self.get_parameter('min_samples').value
        self.calibration_method = self.get_parameter('calibration_method').value
        self.save_directory = os.path.expanduser(self.get_parameter('save_directory').value)

        # 로봇 포즈 획득 방법 파라미터
        self.pose_source = self.get_parameter('pose_source').value
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_ee_frame = self.get_parameter('robot_ee_frame').value
        self.robot_pose_topic = self.get_parameter('robot_pose_topic').value
        self.joint_states_topic = self.get_parameter('joint_states_topic').value
        self.invert_gripper2base = self.get_parameter('invert_gripper2base').value

        # ArUco 설정
        if self.marker_type == 'aruco':
            aruco_dict_type = getattr(aruco, aruco_dict_name)
            self.aruco_dict = aruco.getPredefinedDictionary(aruco_dict_type)
            self.aruco_params = aruco.DetectorParameters()
            self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # 체커보드 설정
        self.checkerboard_size = (self.checkerboard_cols, self.checkerboard_rows)

        # 카메라 내부 파라미터 (기본값 - camera_info에서 업데이트됨)
        # Color camera intrinsics from camera_info topic
        self.camera_matrix = np.array([
            [435.84725952, 0.0, 425.93365479],
            [0.0, 435.17022705, 247.00737],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)  # RealSense는 일반적으로 왜곡 보정이 적용된 이미지를 제공
        self.camera_info_received = False

        # 데이터 저장
        self.robot_poses = []  # 로봇 end-effector 포즈 (base -> end-effector)
        self.camera_poses = []  # 카메라가 본 마커 포즈 (camera -> marker)
        self.images = []  # 캡처한 이미지 (디버깅용)

        self.bridge = CvBridge()
        self.current_image = None
        self.current_robot_pose = None
        self.current_joint_states = None

        # TF Buffer 및 Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/realsense_service_node/color/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/realsense_service_node/color/camera_info',
            self.camera_info_callback,
            10
        )

        # 로봇 포즈 소스에 따라 구독 설정
        if self.pose_source == 'topic':
            self.robot_pose_sub = self.create_subscription(
                PoseStamped,
                self.robot_pose_topic,
                self.robot_pose_callback,
                10
            )
            self.get_logger().info(f'로봇 포즈 토픽 구독: {self.robot_pose_topic}')

        elif self.pose_source == 'joint_states':
            self.joint_states_sub = self.create_subscription(
                JointState,
                self.joint_states_topic,
                self.joint_states_callback,
                10
            )
            self.get_logger().info(f'조인트 상태 토픽 구독: {self.joint_states_topic}')
            self.get_logger().warn('joint_states 모드는 forward kinematics가 필요합니다')

        elif self.pose_source == 'tf':
            self.get_logger().info(f'TF 사용: {self.robot_base_frame} -> {self.robot_ee_frame}')

        # Service
        self.capture_srv = self.create_service(
            Trigger,
            'capture_calibration_sample',
            self.capture_sample_callback
        )

        self.compute_srv = self.create_service(
            Trigger,
            'compute_calibration',
            self.compute_calibration_callback
        )

        self.reset_srv = self.create_service(
            Trigger,
            'reset_calibration',
            self.reset_calibration_callback
        )

        # 저장 디렉토리 생성
        os.makedirs(self.save_directory, exist_ok=True)

        self.get_logger().info('Hand-Eye Calibration Node 시작')
        self.get_logger().info(f'마커 타입: {self.marker_type}')
        self.get_logger().info(f'최소 샘플 수: {self.min_samples}')
        self.get_logger().info(f'캘리브레이션 방법: {self.calibration_method}')
        self.get_logger().info('서비스:')
        self.get_logger().info('  - /capture_calibration_sample: 샘플 캡처')
        self.get_logger().info('  - /compute_calibration: 캘리브레이션 계산')
        self.get_logger().info('  - /reset_calibration: 데이터 초기화')

        # OpenCV 윈도우 미리 생성 (첫 번째 표시 시 검은 화면 방지)
        dummy_image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.imshow('Calibration', dummy_image)
        cv2.waitKey(1)

    def image_callback(self, msg):
        """이미지 콜백"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def camera_info_callback(self, msg):
        """Camera Info 콜백 - Camera Intrinsics 업데이트"""
        if not self.camera_info_received and len(msg.k) >= 9:
            self.camera_matrix = np.array([
                [msg.k[0], msg.k[1], msg.k[2]],
                [msg.k[3], msg.k[4], msg.k[5]],
                [msg.k[6], msg.k[7], msg.k[8]]
            ])
            self.camera_info_received = True
            self.get_logger().info(f'Camera Intrinsics 업데이트됨:')
            self.get_logger().info(f'  fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}')
            self.get_logger().info(f'  cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f}')

    def robot_pose_callback(self, msg):
        """로봇 포즈 콜백"""
        self.current_robot_pose = msg

    def joint_states_callback(self, msg):
        """조인트 상태 콜백"""
        self.current_joint_states = msg

    def get_current_robot_pose(self):
        """현재 로봇 end-effector 포즈 얻기"""
        if self.pose_source == 'topic':
            if self.current_robot_pose is None:
                self.get_logger().warn('로봇 포즈 토픽을 아직 받지 못했습니다')
                return None
            return self.current_robot_pose

        elif self.pose_source == 'tf':
            try:
                # 현재 시간 얻기
                now = self.get_clock().now()

                # TF에서 변환 조회 (최신 데이터 요청)
                transform = self.tf_buffer.lookup_transform(
                    self.robot_base_frame,
                    self.robot_ee_frame,
                    rclpy.time.Time(),  # 가장 최신 TF 요청
                    timeout=rclpy.duration.Duration(seconds=0.5)  # 0.5초 대기
                )

                # TF 신선도 검증 (TF 타임스탬프가 너무 오래되지 않았는지)
                tf_time = rclpy.time.Time.from_msg(transform.header.stamp)
                time_diff = (now - tf_time).nanoseconds / 1e9  # 초 단위

                if time_diff > 0.5:
                    self.get_logger().warn(
                        f'경고: TF 데이터가 오래됨! (나이: {time_diff:.3f}초)'
                    )
                    self.get_logger().warn(
                        '로봇이 완전히 정지했는지, TF가 계속 발행되고 있는지 확인하세요'
                    )
                    return None

                # 디버그: TF 신선도 로그
                self.get_logger().info(
                    f'TF 수신: {self.robot_ee_frame} (나이: {time_diff:.3f}초)'
                )

                # TransformStamped를 PoseStamped로 변환
                pose_msg = PoseStamped()
                pose_msg.header = transform.header
                pose_msg.pose.position.x = transform.transform.translation.x
                pose_msg.pose.position.y = transform.transform.translation.y
                pose_msg.pose.position.z = transform.transform.translation.z
                pose_msg.pose.orientation = transform.transform.rotation

                return pose_msg

            except TransformException as ex:
                self.get_logger().warn(f'TF 조회 실패: {ex}')
                self.get_logger().warn(f'프레임 확인: {self.robot_base_frame} -> {self.robot_ee_frame}')
                self.get_logger().warn('TF가 발행되고 있는지 확인하세요: ros2 run tf2_ros tf2_echo {base} {ee}')
                return None

        elif self.pose_source == 'joint_states':
            if self.current_joint_states is None:
                self.get_logger().warn('조인트 상태를 아직 받지 못했습니다')
                return None

            # TODO: Forward kinematics 구현 필요
            # 로봇 모델에 따라 조인트 각도를 end-effector 포즈로 변환
            self.get_logger().error('joint_states 모드는 아직 구현되지 않았습니다')
            self.get_logger().error('pose_source를 tf 또는 topic으로 설정하세요')
            return None

        return None

    def detect_marker_pose(self, image):
        """마커 검출 및 포즈 추정"""
        if self.marker_type == 'aruco':
            return self.detect_aruco_pose(image)
        elif self.marker_type == 'checkerboard':
            return self.detect_checkerboard_pose(image)
        else:
            self.get_logger().error(f'지원하지 않는 마커 타입: {self.marker_type}')
            return None, None

    def detect_aruco_pose(self, image):
        """ArUco 마커 검출 및 포즈 추정"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # ArUco 마커 검출 (OpenCV 4.7+ 새로운 API)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # 첫 번째 마커 사용
            marker_corners = corners[0].reshape(4, 2)

            # ArUco 마커의 3D 객체 포인트 (마커 중심이 원점)
            half_size = self.marker_size / 2.0
            obj_points = np.array([
                [-half_size,  half_size, 0],
                [ half_size,  half_size, 0],
                [ half_size, -half_size, 0],
                [-half_size, -half_size, 0]
            ], dtype=np.float32)

            # solvePnP로 포즈 추정
            success, rvec, tvec = cv2.solvePnP(
                obj_points, marker_corners,
                self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if success:
                # 시각화
                image_vis = image.copy()
                aruco.drawDetectedMarkers(image_vis, corners, ids)
                cv2.drawFrameAxes(
                    image_vis, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.marker_size * 0.5
                )

                # Rotation vector을 rotation matrix로 변환
                R, _ = cv2.Rodrigues(rvec)

                # 4x4 변환 행렬 생성
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.flatten()

                return T, image_vis

        return None, image

    def detect_checkerboard_pose(self, image):
        """체커보드 검출 및 포즈 추정"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 체커보드 코너 검출
        ret, corners = cv2.findChessboardCorners(gray, self.checkerboard_size, None)

        if ret:
            # 코너 정밀화
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            # 3D 객체 포인트 생성
            objp = np.zeros((self.checkerboard_rows * self.checkerboard_cols, 3), np.float32)
            objp[:, :2] = np.mgrid[0:self.checkerboard_cols, 0:self.checkerboard_rows].T.reshape(-1, 2)
            objp *= self.checkerboard_square_size

            # 포즈 추정
            ret, rvec, tvec = cv2.solvePnP(
                objp, corners_refined, self.camera_matrix, self.dist_coeffs
            )

            if ret:
                # 시각화
                image_vis = image.copy()
                cv2.drawChessboardCorners(image_vis, self.checkerboard_size, corners_refined, ret)
                cv2.drawFrameAxes(
                    image_vis, self.camera_matrix, self.dist_coeffs,
                    rvec, tvec, self.checkerboard_square_size * 3
                )

                # Rotation vector을 rotation matrix로 변환
                R, _ = cv2.Rodrigues(rvec)

                # 4x4 변환 행렬 생성
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.flatten()

                return T, image_vis

        return None, image

    def pose_msg_to_matrix(self, pose_msg):
        """PoseStamped 메시지를 4x4 변환 행렬로 변환"""
        pose = pose_msg.pose

        # 위치
        t = np.array([pose.position.x, pose.position.y, pose.position.z])

        # 쿼터니언을 회전 행렬로 변환
        q = pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)]
        ])

        # 4x4 변환 행렬
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t

        return T

    def capture_sample_callback(self, request, response):
        """캘리브레이션 샘플 캡처"""
        if self.current_image is None:
            response.success = False
            response.message = '이미지를 받지 못했습니다'
            return response

        # 현재 로봇 포즈 얻기
        robot_pose = self.get_current_robot_pose()
        if robot_pose is None:
            response.success = False
            response.message = '로봇 포즈를 받지 못했습니다'
            return response

        # 마커 검출
        camera_to_marker, vis_image = self.detect_marker_pose(self.current_image)

        if camera_to_marker is None:
            response.success = False
            response.message = '마커를 검출하지 못했습니다'

            # 실패 이미지 표시
            cv2.imshow('Calibration', self.current_image)
            cv2.waitKey(1)
            return response

        # 로봇 포즈 변환
        base_to_ee = self.pose_msg_to_matrix(robot_pose)

        # 데이터 저장
        self.robot_poses.append(base_to_ee)
        self.camera_poses.append(camera_to_marker)
        self.images.append(vis_image)

        # 이미지 표시
        cv2.imshow('Calibration', vis_image)
        cv2.waitKey(1)

        sample_count = len(self.robot_poses)
        response.success = True
        response.message = f'샘플 캡처 성공 ({sample_count}/{self.min_samples})'

        self.get_logger().info(response.message)

        return response

    def compute_calibration_callback(self, request, response):
        """Hand-Eye 캘리브레이션 계산"""
        if len(self.robot_poses) < self.min_samples:
            response.success = False
            response.message = f'샘플 부족: {len(self.robot_poses)}/{self.min_samples}'
            return response

        try:
            # OpenCV hand-eye 캘리브레이션 메소드 매핑
            method_map = {
                'Tsai-Lenz': cv2.CALIB_HAND_EYE_TSAI,
                'Park': cv2.CALIB_HAND_EYE_PARK,
                'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
                'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
                'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS
            }
            method = method_map.get(self.calibration_method, cv2.CALIB_HAND_EYE_TSAI)

            # R_gripper2base 및 t_gripper2base 리스트 생성
            R_gripper2base = []
            t_gripper2base = []

            for pose in self.robot_poses:
                if self.invert_gripper2base:
                    # 저장된 pose는 base -> ee 이므로 역변환으로 gripper -> base를 계산
                    ee_to_base = np.linalg.inv(pose)
                    R_gripper2base.append(ee_to_base[:3, :3])
                    t_gripper2base.append(ee_to_base[:3, 3].reshape(3, 1))
                else:
                    # 입력이 이미 gripper -> base라고 가정
                    R_gripper2base.append(pose[:3, :3])
                    t_gripper2base.append(pose[:3, 3].reshape(3, 1))

            # R_target2cam 및 t_target2cam 리스트 생성
            R_target2cam = []
            t_target2cam = []

            for pose in self.camera_poses:
                R_target2cam.append(pose[:3, :3])
                t_target2cam.append(pose[:3, 3].reshape(3, 1))

            # Hand-Eye 캘리브레이션 수행
            self.get_logger().info(
                f'calibrateHandEye 입력: invert_gripper2base={self.invert_gripper2base}'
            )
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=method
            )

            # 결과 행렬 생성
            T_cam2gripper = np.eye(4)
            T_cam2gripper[:3, :3] = R_cam2gripper
            T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

            # 결과 저장
            self.save_calibration_result(T_cam2gripper)

            response.success = True
            response.message = f'캘리브레이션 성공\n변환 행렬:\n{T_cam2gripper}'

            self.get_logger().info('캘리브레이션 완료')
            self.get_logger().info(f'\n{T_cam2gripper}')

            cv2.destroyAllWindows()

        except Exception as e:
            response.success = False
            response.message = f'캘리브레이션 실패: {str(e)}'
            self.get_logger().error(response.message)

        return response

    def reset_calibration_callback(self, request, response):
        """캘리브레이션 데이터 초기화"""
        self.robot_poses.clear()
        self.camera_poses.clear()
        self.images.clear()

        response.success = True
        response.message = '캘리브레이션 데이터가 초기화되었습니다'

        self.get_logger().info(response.message)

        return response

    def save_calibration_result(self, T_cam2gripper):
        """캘리브레이션 결과 저장"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # YAML 파일로 저장
        result = {
            'calibration_time': timestamp,
            'method': self.calibration_method,
            'num_samples': len(self.robot_poses),
            'camera_to_gripper_transform': {
                'translation': {
                    'x': float(T_cam2gripper[0, 3]),
                    'y': float(T_cam2gripper[1, 3]),
                    'z': float(T_cam2gripper[2, 3])
                },
                'rotation_matrix': T_cam2gripper[:3, :3].tolist()
            },
            'transform_matrix': T_cam2gripper.tolist()
        }

        yaml_file = os.path.join(self.save_directory, f'hand_eye_calibration_{timestamp}.yaml')
        with open(yaml_file, 'w') as f:
            yaml.dump(result, f, default_flow_style=False)

        self.get_logger().info(f'결과 저장: {yaml_file}')

        # 이미지 저장
        for i, img in enumerate(self.images):
            img_file = os.path.join(self.save_directory, f'sample_{timestamp}_{i:02d}.png')
            cv2.imwrite(img_file, img)


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrationNode()

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
