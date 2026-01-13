# RealSense Service

ROS2 환경에서 Intel RealSense 카메라를 서비스로 제공하는 노드입니다.

## 기능

- RealSense 카메라의 Color 및 Depth 이미지를 서비스로 제공
- Color 및 Depth 이미지를 토픽으로 연속 발행 (RViz 시각화 지원)
- 카메라 시작/정지 제어
- 해상도 설정 가능
- 발행 주기 설정 가능
- **Hand-Eye 캘리브레이션**: 로봇 팔에 장착된 카메라의 위치 캘리브레이션 (ArUco 마커 또는 체커보드 사용)

## 의존성

### ROS2 패키지
- rclpy
- sensor_msgs
- cv_bridge

### Python 패키지
- pyrealsense2
- opencv-python (opencv-contrib-python 권장)
- numpy
- pyyaml

## 설치

### 1. RealSense SDK 설치

```bash
sudo apt-get install ros-humble-librealsense2*
pip3 install pyrealsense2
```

### 2. 패키지 빌드

```bash
cd ~/ws
colcon build --packages-select realsense_service
source install/setup.bash
```

## 사용법

### 1. 노드 실행

#### 기본 실행 (스트리밍 활성화)

```bash
ros2 run realsense_service realsense_service_node
```

#### Launch 파일로 실행

```bash
ros2 launch realsense_service realsense_service.launch.py
```

#### RViz와 함께 실행

```bash
ros2 launch realsense_service realsense_with_rviz.launch.py
```

#### 파라미터와 함께 실행

```bash
ros2 run realsense_service realsense_service_node --ros-args \
  -p enable_streaming:=true \
  -p publish_rate:=30.0 \
  -p auto_start:=true
```

### 2. 발행되는 토픽

노드가 실행되면 다음 토픽으로 이미지를 발행합니다:

- `/realsense_service_node/color/image_raw` (sensor_msgs/Image) - Color 이미지
- `/realsense_service_node/depth/image_raw` (sensor_msgs/Image) - Depth 이미지

토픽 확인:

```bash
ros2 topic list
ros2 topic echo /realsense_service_node/color/image_raw
```

### 3. RViz에서 이미지 보기

RViz를 사용하여 이미지를 시각화할 수 있습니다:

```bash
# 노드 실행
ros2 launch realsense_service realsense_with_rviz.launch.py
```

또는 수동으로 RViz 설정:

1. RViz 실행: `rviz2`
2. Fixed Frame을 `camera_color_optical_frame`으로 설정
3. Add 버튼 클릭 → Image 선택
4. Topic을 `/realsense_service_node/color/image_raw` 또는 `/realsense_service_node/depth/image_raw`로 설정

### 4. 서비스 호출 예제

#### 이미지 캡처

Color와 Depth 이미지를 모두 캡처:

```bash
ros2 service call /capture_image realsense_service/srv/CaptureImage "{enable_color: true, enable_depth: true, width: 848, height: 480}"
```

Color 이미지만 캡처:

```bash
ros2 service call /capture_image realsense_service/srv/CaptureImage "{enable_color: true, enable_depth: false, width: 848, height: 480}"
```

#### 카메라 제어

카메라 시작:

```bash
ros2 service call /set_camera_state realsense_service/srv/SetCameraState "{start: true}"
```

카메라 정지:

```bash
ros2 service call /set_camera_state realsense_service/srv/SetCameraState "{start: false}"
```

## 파라미터

노드는 다음 파라미터를 지원합니다:

| 파라미터 | 타입 | 기본값 | 설명 |
|---------|------|--------|------|
| `enable_streaming` | bool | true | 토픽으로 이미지 연속 발행 활성화 |
| `publish_rate` | double | 30.0 | 이미지 발행 주기 (Hz) |
| `auto_start` | bool | true | 노드 시작 시 자동으로 카메라 시작 |

파라미터 확인:

```bash
ros2 param list
ros2 param get /realsense_service_node enable_streaming
```

파라미터 변경:

```bash
ros2 param set /realsense_service_node publish_rate 15.0
```

## 서비스 인터페이스

### CaptureImage.srv

```
# Request
bool enable_color      # Color 이미지 캡처 여부
bool enable_depth      # Depth 이미지 캡처 여부
int32 width            # 이미지 너비 (0이면 기본값 848)
int32 height           # 이미지 높이 (0이면 기본값 480)
---
# Response
bool success           # 캡처 성공 여부
string message         # 응답 메시지
sensor_msgs/Image color_image
sensor_msgs/Image depth_image
```

### SetCameraState.srv

```
# Request
bool start             # true: 시작, false: 정지
---
# Response
bool success           # 성공 여부
string message         # 응답 메시지
```

## Python 클라이언트 예제

`examples/` 디렉토리의 `client_example.py`를 참조하세요.

## Hand-Eye Calibration

로봇 팔에 장착된 RealSense 카메라의 hand-eye 캘리브레이션을 수행할 수 있습니다.

### 빠른 시작

```bash
# 캘리브레이션 시스템 시작
ros2 launch realsense_service hand_eye_calibration.launch.py

# 다른 터미널에서 헬퍼 실행
ros2 run realsense_service calibration_helper
```

### 주요 기능

- ArUco 마커 또는 체커보드 기반 캘리브레이션
- 여러 캘리브레이션 알고리즘 지원 (Tsai-Lenz, Park, Horaud, Andreff, Daniilidis)
- 자동 마커 검출 및 포즈 추정
- 인터랙티브 데이터 수집
- YAML 형식으로 결과 저장

### 사용 방법

자세한 사용 방법은 [HAND_EYE_CALIBRATION_GUIDE.md](HAND_EYE_CALIBRATION_GUIDE.md)를 참조하세요.

### 필요한 준비물

1. ArUco 마커 (6x6 250 dictionary, 5cm x 5cm) 또는 체커보드
2. 로봇 팔 (end-effector 포즈를 `/robot/end_effector_pose` 토픽으로 발행)
3. RealSense 카메라 (로봇 팔에 고정 장착)

### 노드

- `hand_eye_calibration_node`: 캘리브레이션 계산 노드
- `calibration_helper`: 데이터 수집 헬퍼
- `robot_pose_publisher`: 테스트용 로봇 포즈 발행 노드

## 문제 해결

### 카메라 연결 오류
- RealSense 카메라가 제대로 연결되어 있는지 확인
- `realsense-viewer`로 카메라 동작 확인

### 권한 오류
```bash
sudo usermod -a -G plugdev $USER
# 로그아웃 후 다시 로그인
```

## 라이센스

Apache-2.0
