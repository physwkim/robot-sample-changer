# 빌드 가이드

## 사전 요구사항

### 1. Colcon 설치

ROS2 빌드 도구인 colcon을 설치합니다:

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```

### 2. RealSense 의존성 설치

```bash
# RealSense SDK 설치
sudo apt-get update
sudo apt-get install -y \
    ros-humble-librealsense2* \
    ros-humble-cv-bridge

# Python 패키지 설치
pip3 install pyrealsense2 opencv-python numpy
```

## 빌드 방법

### 1. 워크스페이스 빌드

```bash
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select realsense_service
```

### 2. 환경 설정

```bash
source ~/ws/install/setup.bash
```

## 실행

### 서비스 노드 실행

#### 방법 1: 직접 실행

```bash
ros2 run realsense_service realsense_service_node
```

#### 방법 2: Launch 파일 사용

```bash
ros2 launch realsense_service realsense_service.launch.py
```

### 클라이언트 예제 실행

새 터미널에서:

```bash
cd ~/ws/realsense_service/examples
python3 client_example.py
```

## 테스트

### 서비스 확인

```bash
# 실행 중인 서비스 목록 확인
ros2 service list

# 서비스 타입 확인
ros2 service type /capture_image
ros2 service type /set_camera_state
```

### 간단한 서비스 호출 테스트

```bash
# 카메라 시작
ros2 service call /set_camera_state realsense_service/srv/SetCameraState "{start: true}"

# 이미지 캡처
ros2 service call /capture_image realsense_service/srv/CaptureImage "{enable_color: true, enable_depth: true, width: 848, height: 480}"
```

## 문제 해결

### colcon 빌드 오류

1. 의존성 확인:
```bash
rosdep install --from-paths . --ignore-src -r -y
```

2. 빌드 캐시 정리:
```bash
rm -rf build install log
colcon build --packages-select realsense_service
```

### RealSense 카메라 인식 오류

1. 카메라 연결 확인:
```bash
lsusb | grep Intel
```

2. realsense-viewer로 카메라 테스트:
```bash
realsense-viewer
```

3. USB 권한 설정:
```bash
sudo usermod -a -G plugdev $USER
# 로그아웃 후 다시 로그인
```

### Import 오류

빌드 후 반드시 환경 설정을 해야 합니다:
```bash
source ~/ws/install/setup.bash
```

## 추가 정보

- ROS2 Humble 공식 문서: https://docs.ros.org/en/humble/
- RealSense SDK: https://github.com/IntelRealSense/librealsense
