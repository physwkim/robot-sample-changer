# Hand-Eye Calibration 결과 사용 가이드

캘리브레이션 완료 후 결과를 실제 로봇 시스템에 통합하는 방법을 설명합니다.

## 1. 캘리브레이션 결과 확인

캘리브레이션 완료 후 `~/calibration_data/` 디렉토리에 YAML 파일이 생성됩니다.

```bash
cd ~/calibration_data
cat hand_eye_calibration_*.yaml
```

결과 예시:
```yaml
calibration_time: '20250131_143022'
method: Tsai-Lenz
num_samples: 15
camera_to_gripper_transform:
  translation:
    x: 0.05
    y: 0.02
    z: 0.08
  rotation_matrix:
  - [0.999, -0.001, 0.001]
  - [0.001, 0.999, -0.001]
  - [-0.001, 0.001, 0.999]
transform_matrix:
- [0.999, -0.001, 0.001, 0.05]
- [0.001, 0.999, -0.001, 0.02]
- [-0.001, 0.001, 0.999, 0.08]
- [0.0, 0.0, 0.0, 1.0]
```

## 2. 사용 방법 (3가지)

### 방법 1: TF Broadcaster 사용 (권장, 빠른 테스트)

TF로 카메라 위치를 실시간 발행합니다. URDF 수정 없이 바로 사용 가능합니다.

#### 2.1 노드 실행

```bash
cd ~/ws
source install/setup.bash

# 자동으로 최신 캘리브레이션 파일 사용
ros2 run realsense_service camera_tf_broadcaster

# 또는 특정 파일 지정
ros2 run realsense_service camera_tf_broadcaster \
  --ros-args \
  -p calibration_file:=~/calibration_data/hand_eye_calibration_20250131_143022.yaml \
  -p parent_frame:=tool0 \
  -p camera_frame:=camera_link
```

#### 2.2 TF 확인

```bash
# TF 트리 확인
ros2 run tf2_tools view_frames

# 특정 변환 확인
ros2 run tf2_ros tf2_echo base_link camera_link
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

#### 2.3 Launch 파일에 포함

```python
# your_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 로봇 드라이버
        Node(package='your_robot_driver', ...),

        # RealSense 노드
        Node(
            package='realsense_service',
            executable='realsense_service_node',
            ...
        ),

        # Camera TF Broadcaster
        Node(
            package='realsense_service',
            executable='camera_tf_broadcaster',
            parameters=[{
                'parent_frame': 'tool0',
                'camera_frame': 'camera_link',
            }]
        ),
    ])
```

### 방법 2: URDF/XACRO에 추가 (권장, 영구적)

로봇 모델에 카메라를 영구적으로 추가합니다.

#### 2.1 RPY 값 계산

YAML 파일의 rotation matrix를 RPY(Roll-Pitch-Yaw)로 변환:

```python
import numpy as np

# YAML의 rotation_matrix 값 사용
R = np.array([
    [0.999, -0.001, 0.001],
    [0.001, 0.999, -0.001],
    [-0.001, 0.001, 0.999]
])

sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
roll = np.arctan2(R[2,1], R[2,2])
pitch = np.arctan2(-R[2,0], sy)
yaw = np.arctan2(R[1,0], R[0,0])

print(f"RPY (radian): {roll}, {pitch}, {yaw}")
print(f"RPY (degree): {np.rad2deg(roll)}, {np.rad2deg(pitch)}, {np.rad2deg(yaw)}")
```

#### 2.2 로봇 XACRO 수정

```xml
<!-- your_robot.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="your_robot">

  <!-- 기존 로봇 URDF -->
  <xacro:include filename="$(find your_robot_description)/urdf/your_robot.xacro"/>

  <!-- RealSense D405 추가 -->
  <xacro:include filename="$(find realsense_service)/urdf/realsense_d405.xacro"/>

  <xacro:realsense_d405 parent="tool0">
    <!--
    캘리브레이션 결과를 여기에 입력!
    xyz: translation (x, y, z) - 미터 단위
    rpy: rotation (roll, pitch, yaw) - 라디안 단위
    -->
    <origin xyz="0.05 0.02 0.08" rpy="0.0 1.57 0.0"/>
  </xacro:realsense_d405>

</robot>
```

#### 2.3 URDF 확인

```bash
# URDF 체크
check_urdf your_robot.urdf

# RViz에서 시각화
ros2 launch your_robot_description view_robot.launch.py
```

### 방법 3: Static TF Publisher 사용 (간단한 테스트)

명령줄에서 직접 TF 발행:

```bash
# Translation과 Rotation(quaternion) 사용
ros2 run tf2_ros static_transform_publisher \
  0.05 0.02 0.08 \
  0.0 0.707 0.707 0.0 \
  tool0 camera_link

# 또는 RPY 사용 (--rpy 플래그)
ros2 run tf2_ros static_transform_publisher \
  --x 0.05 --y 0.02 --z 0.08 \
  --roll 0.0 --pitch 1.57 --yaw 0.0 \
  --frame-id tool0 --child-frame-id camera_link
```

## 3. 비전 기반 작업에 활용

### 3.1 물체 좌표 변환 예제

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def camera_to_robot_coords(self, x_cam, y_cam, z_cam):
        """카메라 좌표를 로봇 좌표로 변환"""
        # 카메라 좌표계의 점
        point_camera = PointStamped()
        point_camera.header.frame_id = 'camera_color_optical_frame'
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.point.x = x_cam
        point_camera.point.y = y_cam
        point_camera.point.z = z_cam

        # 변환
        transform = self.tf_buffer.lookup_transform(
            'base_link',
            'camera_color_optical_frame',
            rclpy.time.Time()
        )
        point_robot = do_transform_point(point_camera, transform)

        return point_robot.point.x, point_robot.point.y, point_robot.point.z
```

### 3.2 완전한 Pick and Place 예제

`examples/vision_guided_pick.py` 참조:

```bash
# RealSense 노드 실행
ros2 run realsense_service realsense_service_node

# TF Broadcaster 실행
ros2 run realsense_service camera_tf_broadcaster

# Vision 예제 실행
cd ~/ws/realsense_service/examples
python3 vision_guided_pick.py
```

## 4. RealSense D405 사양

- **해상도**: 848 x 480 (기본), 최대 1280 x 720
- **FOV**: 87° x 58° (Depth), 90° x 65° (Color)
- **Depth 범위**: 0.07m - 4m (최적: 0.1m - 1.5m)
- **크기**: 42mm x 28mm x 22mm
- **무게**: 32g

### 카메라 내부 파라미터 (참고용)

실제 사용 시 RealSense SDK에서 자동으로 얻어야 합니다:

```python
import pyrealsense2 as rs

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)

profile = pipeline.start(config)
color_profile = profile.get_stream(rs.stream.color)
intrinsics = color_profile.as_video_stream_profile().get_intrinsics()

print(f"fx: {intrinsics.fx}")
print(f"fy: {intrinsics.fy}")
print(f"cx: {intrinsics.ppx}")
print(f"cy: {intrinsics.ppy}")
```

## 5. 문제 해결

### TF가 발행되지 않음

```bash
# TF 확인
ros2 topic echo /tf
ros2 topic echo /tf_static

# Camera TF Broadcaster 로그 확인
ros2 run realsense_service camera_tf_broadcaster --ros-args --log-level debug
```

### 좌표 변환이 이상함

1. **캘리브레이션 재수행**: 샘플 수를 늘리고 다양한 각도에서 수집
2. **프레임 이름 확인**: `ros2 run tf2_ros tf2_echo base_link camera_link`
3. **변환 행렬 확인**: YAML 파일의 transform_matrix 확인

### 물체 위치가 부정확함

1. **카메라 내부 파라미터**: RealSense SDK에서 실제 값 사용
2. **Depth 정확도**: D405는 근거리(7cm-1.5m)에 최적화됨
3. **조명**: 충분한 조명과 반사 없는 환경

## 6. 실전 팁

### 6.1 캘리브레이션 검증

```bash
# TF Broadcaster 실행
ros2 run realsense_service camera_tf_broadcaster

# 알려진 위치에 물체 배치
# Vision 노드로 물체 좌표 측정
# 실제 위치와 비교하여 오차 확인 (보통 2-5mm 이내)
```

### 6.2 정기적 재캘리브레이션

- 카메라 마운트가 충격을 받았을 때
- 정밀도가 떨어졌다고 느껴질 때
- 3-6개월마다 (예방 차원)

### 6.3 여러 카메라 사용

```python
# camera_tf_broadcaster를 여러 개 실행
# 각각 다른 캘리브레이션 파일과 프레임 이름 사용

# Camera 1
ros2 run realsense_service camera_tf_broadcaster \
  --ros-args \
  -p calibration_file:=~/calibration_data/camera1_calibration.yaml \
  -p camera_frame:=camera1_link

# Camera 2
ros2 run realsense_service camera_tf_broadcaster \
  --ros-args \
  -p calibration_file:=~/calibration_data/camera2_calibration.yaml \
  -p camera_frame:=camera2_link
```

## 7. 참고 자료

- RealSense D405 데이터시트: https://www.intelrealsense.com/depth-camera-d405/
- TF2 튜토리얼: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
- URDF 튜토리얼: https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html
