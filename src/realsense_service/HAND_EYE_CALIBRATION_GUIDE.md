# Hand-Eye Calibration 가이드

로봇 팔에 장착된 RealSense 카메라의 hand-eye 캘리브레이션을 수행하는 가이드입니다.

## 개요

Hand-eye 캘리브레이션은 로봇의 end-effector에 장착된 카메라의 위치와 방향을 정확히 파악하기 위한 과정입니다. 이를 통해 카메라 좌표계와 로봇 좌표계 간의 변환 관계를 구할 수 있습니다.

## 필요한 준비물

1. **ArUco 마커 또는 체커보드**
   - ArUco 마커: 6x6 250 dictionary (기본값)
   - 마커 크기: 5cm x 5cm (기본값, 조정 가능)
   - 또는 체커보드: 9x6 (내부 코너 기준)
   - 체커보드 사각형 크기: 2.5cm (조정 가능)

2. **로봇 팔**
   - End-effector 포즈를 ROS2 토픽으로 발행할 수 있어야 함
   - 토픽: `/robot/end_effector_pose` (geometry_msgs/PoseStamped)
   - 프레임: `robot_base` 기준

3. **RealSense 카메라**
   - 로봇 팔의 end-effector에 고정 장착
   - 카메라가 움직이지 않도록 단단히 고정

## ArUco 마커 준비

### ArUco 마커 생성

```python
import cv2
import cv2.aruco as aruco
import numpy as np

# ArUco 딕셔너리 선택
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# 마커 생성 (ID 0)
marker_size = 200  # 픽셀
marker_image = aruco.generateImageMarker(aruco_dict, 0, marker_size)

# 저장
cv2.imwrite('aruco_marker.png', marker_image)
```

또는 온라인 생성기 사용:
- https://chev.me/arucogen/

### 마커 출력

1. 생성된 마커를 인쇄합니다
2. 정확한 크기로 인쇄되었는지 확인합니다 (5cm x 5cm 권장)
3. 평평한 보드에 붙입니다
4. 마커가 평평하고 왜곡되지 않도록 주의합니다

## 사용 방법

### 1. 시스템 시작

#### 터미널 1: RealSense 및 캘리브레이션 노드 실행

```bash
cd ~/ws
source install/setup.bash
ros2 launch realsense_service hand_eye_calibration.launch.py
```

파라미터 옵션:
```bash
# TF 사용 (권장 - 대부분의 로봇 시스템)
ros2 launch realsense_service hand_eye_calibration.launch.py \
  pose_source:=tf \
  robot_base_frame:=base_link \
  robot_ee_frame:=tool0 \
  marker_type:=aruco \
  marker_size:=0.05 \
  min_samples:=15

# 토픽 사용 (legacy)
ros2 launch realsense_service hand_eye_calibration.launch.py \
  pose_source:=topic \
  robot_pose_topic:=/robot/end_effector_pose
```

#### 터미널 2: 로봇 시스템 확인

**방법 1: TF 사용 (권장)**

실제 로봇 시스템이 TF를 발행하는지 확인:

```bash
# TF 프레임 확인
ros2 run tf2_ros tf2_echo base_link tool0

# 또는 TF 트리 전체 확인
ros2 run tf2_tools view_frames
```

대부분의 ROS2 로봇 드라이버는 자동으로 TF를 발행합니다.

**방법 2: 토픽 사용**

로봇 컨트롤러가 end-effector 포즈를 토픽으로 발행하는 경우:

```bash
# 토픽 확인
ros2 topic list | grep pose
ros2 topic echo /robot/end_effector_pose
```

**테스트용 (실제 로봇이 없는 경우):**
```bash
ros2 run realsense_service robot_pose_publisher
```

#### 터미널 3: 캘리브레이션 헬퍼 실행

```bash
ros2 run realsense_service calibration_helper
```

### 2. 데이터 수집

1. **로봇 이동**
   - 로봇을 다양한 포즈로 이동시킵니다
   - ArUco 마커가 카메라 시야에 들어오도록 합니다
   - 마커가 화면에 명확하게 보여야 합니다

2. **샘플 캡처**
   - 캘리브레이션 헬퍼 창에서 `SPACE` 키를 누릅니다
   - 성공 메시지가 표시되면 다음 포즈로 이동합니다
   - 실패하면 로봇 위치를 조정하고 다시 시도합니다

3. **다양성 확보**
   - 로봇의 작업 공간 전체에 걸쳐 샘플을 수집합니다
   - 다양한 각도와 거리에서 촬영합니다
   - 최소 10개, 권장 15-20개의 샘플을 수집합니다

### 3. 캘리브레이션 계산

충분한 샘플이 수집되면:

1. 캘리브레이션 헬퍼 창에서 `C` 키를 누릅니다
2. 계산이 완료되면 결과가 표시됩니다
3. 결과는 `~/calibration_data/` 디렉토리에 저장됩니다

### 4. 결과 확인

```bash
cd ~/calibration_data
cat hand_eye_calibration_*.yaml
```

결과 파일 예시:
```yaml
calibration_time: '20250101_120000'
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

## 서비스 API

캘리브레이션 노드는 다음 서비스를 제공합니다:

### 1. 샘플 캡처

```bash
ros2 service call /capture_calibration_sample std_srvs/srv/Trigger
```

### 2. 캘리브레이션 계산

```bash
ros2 service call /compute_calibration std_srvs/srv/Trigger
```

### 3. 데이터 초기화

```bash
ros2 service call /reset_calibration std_srvs/srv/Trigger
```

## 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `marker_type` | string | aruco | 마커 타입 (aruco 또는 checkerboard) |
| `aruco_dict` | string | DICT_6X6_250 | ArUco 딕셔너리 타입 |
| `marker_size` | double | 0.05 | ArUco 마커 크기 (미터) |
| `checkerboard_rows` | int | 6 | 체커보드 행 수 |
| `checkerboard_cols` | int | 9 | 체커보드 열 수 |
| `checkerboard_square_size` | double | 0.025 | 체커보드 사각형 크기 (미터) |
| `min_samples` | int | 10 | 최소 샘플 수 |
| `calibration_method` | string | Tsai-Lenz | 캘리브레이션 방법 |
| `save_directory` | string | ~/calibration_data | 결과 저장 디렉토리 |
| **`pose_source`** | string | **tf** | **로봇 포즈 획득 방법 (tf, topic, joint_states)** |
| **`robot_base_frame`** | string | **base_link** | **로봇 base 프레임 (TF 사용 시)** |
| **`robot_ee_frame`** | string | **tool0** | **로봇 end-effector 프레임 (TF 사용 시)** |
| `robot_pose_topic` | string | /robot/end_effector_pose | 로봇 포즈 토픽 (topic 모드 사용 시) |
| `joint_states_topic` | string | /joint_states | 조인트 상태 토픽 (joint_states 모드 사용 시) |

### 캘리브레이션 방법

- `Tsai-Lenz`: 가장 일반적으로 사용 (권장)
- `Park`: Park의 방법
- `Horaud`: Horaud의 방법
- `Andreff`: Andreff의 방법
- `Daniilidis`: Daniilidis의 방법

## 팁과 주의사항

### 좋은 캘리브레이션을 위한 팁

1. **다양한 포즈**
   - 로봇의 작업 공간 전체에 걸쳐 샘플 수집
   - 다양한 각도 (최소 30-40도 차이)
   - 다양한 거리 (20cm - 60cm)

2. **마커 품질**
   - 고품질 인쇄
   - 평평한 표면
   - 충분한 조명
   - 반사 없음

3. **샘플 수**
   - 최소 10개
   - 권장 15-20개
   - 더 많을수록 정확도 향상

4. **로봇 정확도**
   - 로봇 포즈가 정확해야 함
   - 진동 없이 안정된 상태에서 캡처

### 일반적인 문제 해결

#### 마커 검출 실패

- 조명 확인
- 마커가 카메라 시야에 완전히 들어오는지 확인
- 마커가 너무 작거나 크지 않은지 확인
- 마커가 왜곡되지 않았는지 확인

#### 캘리브레이션 결과가 좋지 않음

- 더 많은 샘플 수집
- 샘플의 다양성 증가
- 로봇 포즈의 정확도 확인
- 마커 크기 파라미터 확인

#### 로봇 포즈를 받지 못함

- 로봇 컨트롤러가 실행 중인지 확인
- 토픽 이름 확인: `ros2 topic list`
- 토픽 타입 확인: `ros2 topic info /robot/end_effector_pose`

## 실제 로봇 통합

실제 로봇을 사용하는 경우, 로봇 컨트롤러에서 다음을 구현해야 합니다:

```python
# 예시: 로봇 포즈 발행
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/robot/end_effector_pose',
            10
        )

    def publish_current_pose(self):
        # 로봇의 현재 end-effector 포즈 가져오기
        x, y, z, qx, qy, qz, qw = self.get_robot_pose()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'robot_base'
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)
```

## 결과 사용

캘리브레이션 결과 (camera_to_gripper 변환)를 사용하여:

1. **TF 브로드캐스터로 발행**
   ```python
   from tf2_ros import TransformBroadcaster
   # 변환 행렬을 TF로 발행
   ```

2. **비전 기반 작업**
   - 카메라에서 본 객체 위치를 로봇 좌표계로 변환
   - Pick and place 작업
   - 비전 가이드 조립

3. **URDF/XACRO 업데이트**
   - 로봇 모델에 정확한 카메라 위치 반영

## 참고 자료

- OpenCV Hand-Eye Calibration: https://docs.opencv.org/master/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
- ArUco 마커: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
- ROS2 TF2: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html
