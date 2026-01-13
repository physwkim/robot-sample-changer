# RViz에서 RealSense 이미지 보기 가이드

RViz를 사용하여 RealSense 카메라의 Color 및 Depth 이미지를 실시간으로 시각화하는 방법을 설명합니다.

## 방법 1: Launch 파일 사용 (권장)

가장 간단한 방법입니다. Launch 파일이 노드와 RViz를 자동으로 설정합니다.

```bash
cd ~/ws
source install/setup.bash
ros2 launch realsense_service realsense_with_rviz.launch.py
```

이 명령은 다음을 자동으로 실행합니다:
- RealSense 서비스 노드
- 미리 설정된 RViz

## 방법 2: 수동 설정

### 1단계: 노드 실행

첫 번째 터미널에서:

```bash
cd ~/ws
source install/setup.bash
ros2 run realsense_service realsense_service_node
```

### 2단계: RViz 실행

두 번째 터미널에서:

```bash
rviz2
```

### 3단계: RViz 설정

1. **Fixed Frame 설정**
   - 왼쪽 패널에서 "Global Options" → "Fixed Frame"
   - `camera_color_optical_frame`으로 변경

2. **Color 이미지 추가**
   - 왼쪽 하단의 "Add" 버튼 클릭
   - "By display type" 탭에서 "Image" 선택
   - "OK" 클릭
   - 새로 추가된 "Image" 항목 확장
   - "Topic" → `/realsense_service_node/color/image_raw` 선택

3. **Depth 이미지 추가**
   - 다시 "Add" 버튼 클릭
   - "Image" 선택 후 "OK"
   - "Topic" → `/realsense_service_node/depth/image_raw` 선택
   - "Normalize Range"를 체크하면 Depth 이미지를 더 잘 볼 수 있습니다

## 방법 3: 설정 파일 사용

미리 만들어진 RViz 설정을 사용할 수 있습니다:

### 노드 실행

```bash
cd ~/ws
source install/setup.bash
ros2 run realsense_service realsense_service_node
```

### RViz 실행 (설정 파일 사용)

```bash
rviz2 -d ~/ws/install/realsense_service/share/realsense_service/rviz/realsense.rviz
```

## 토픽 확인

이미지가 발행되고 있는지 확인:

```bash
# 토픽 목록 확인
ros2 topic list

# Color 이미지 정보 확인
ros2 topic info /realsense_service_node/color/image_raw

# Depth 이미지 정보 확인
ros2 topic info /realsense_service_node/depth/image_raw

# 발행 주기 확인
ros2 topic hz /realsense_service_node/color/image_raw
```

## 파라미터 조정

### 발행 주기 변경

느린 컴퓨터에서는 발행 주기를 줄일 수 있습니다:

```bash
# 30Hz → 15Hz로 변경
ros2 launch realsense_service realsense_with_rviz.launch.py publish_rate:=15.0
```

### 스트리밍 비활성화

서비스 모드만 사용하려면:

```bash
ros2 run realsense_service realsense_service_node --ros-args \
  -p enable_streaming:=false \
  -p auto_start:=false
```

## 문제 해결

### 이미지가 표시되지 않을 때

1. **노드가 실행 중인지 확인**
   ```bash
   ros2 node list
   # /realsense_service_node가 보여야 함
   ```

2. **토픽이 발행되고 있는지 확인**
   ```bash
   ros2 topic echo /realsense_service_node/color/image_raw --once
   ```

3. **카메라가 연결되어 있는지 확인**
   ```bash
   # RealSense viewer로 테스트
   realsense-viewer
   ```

4. **Fixed Frame 확인**
   - RViz에서 Fixed Frame이 올바르게 설정되어 있는지 확인
   - `camera_color_optical_frame` 또는 `camera_depth_optical_frame` 사용

### Depth 이미지가 검게 보일 때

- "Normalize Range" 옵션을 활성화하세요
- RViz의 Image display 설정에서:
  - "Min Value": 0
  - "Max Value": 자동 또는 5000 정도로 설정

### NumPy 경고가 나타날 때

```bash
pip3 install 'numpy<2'
```

## 추가 기능

### 이미지 저장

RViz에서 이미지 위에서 우클릭 → "Save Image" 선택

### 다중 창 보기

RViz에서 여러 Image display를 추가하여 Color와 Depth를 동시에 볼 수 있습니다.

## Launch 파일 옵션

`realsense_with_rviz.launch.py`는 다음 파라미터를 지원합니다:

```bash
ros2 launch realsense_service realsense_with_rviz.launch.py \
  enable_streaming:=true \
  publish_rate:=30.0 \
  auto_start:=true
```

- `enable_streaming`: 토픽 발행 활성화 (default: true)
- `publish_rate`: 발행 주기 Hz (default: 30.0)
- `auto_start`: 자동으로 카메라 시작 (default: true)
