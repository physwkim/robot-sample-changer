# Multi-Holder Sample Loading Sequence

자동으로 Z offset을 적용하여 여러 holder를 처리하는 개선된 버전입니다.

## 주요 개선사항

### 1. **Taught Waypoints 분리 (YAML 파일)**
- 하드코딩 대신 `config/taught_waypoints.yaml`에서 관리
- **필수 waypoint 4개만** teaching 필요:
  - `holder1_standby`: Holder1 대기 위치
  - `holder1_on_position`: Holder1 샘플 픽업 위치
  - `sample_holder_standby`: Sample holder 대기 위치
  - `sample_holder_on_position`: Sample holder 샘플 배치 위치

### 2. **자동 Cartesian Offset 계산**
- 나머지 waypoint는 **상대 offset으로 자동 계산**:
  - `above` = `on_position` + 5mm -Y (뒤로)
  - `retreat` = `above` + 50mm -Z (후퇴)
- FK/IK를 사용하여 정확한 Cartesian 좌표 계산

### 3. **Multi-Holder Z-Offset 자동 적용**
- Holder1 위치만 teaching
- Holder2, Holder3는 자동으로 Z offset 적용:
  - Holder2 = Holder1 + (-30mm) Z
  - Holder3 = Holder1 + (-60mm) Z

## 사용 방법

### Launch 파일 사용 (권장)

```bash
source /home/stevek/ws/install/setup.bash

# 단일 holder
ros2 launch mtc_tutorial multi_holder_sequence.launch.py

# 여러 holder (1, 2, 3)
ros2 launch mtc_tutorial multi_holder_sequence.launch.py \
  holder_list:="[1, 2, 3]" \
  holder_z_offset:=-0.03 \
  num_cycles:=2

# Step-by-step debug 모드
ros2 launch mtc_tutorial multi_holder_sequence.launch.py \
  holder_list:="[1, 2]" \
  step_by_step:=true
```

### 직접 실행

```bash
source /home/stevek/ws/install/setup.bash

ros2 run mtc_tutorial multi_holder_sequence \
  --ros-args \
  --params-file $(ros2 pkg prefix mtc_tutorial)/share/mtc_tutorial/config/taught_waypoints.yaml \
  -p use_gripper_action:=true \
  -p gripper_action_name:=/gripper_action_controller/gripper_cmd \
  -p use_movegroup_action:=true \
  -p holder_list:="[1, 2, 3]" \
  -p holder_z_offset:=-0.03
```

## Waypoint Re-Teaching

Waypoint를 다시 teaching해야 할 때:

1. `config/taught_waypoints.yaml` 파일 편집
2. 4개 핵심 waypoint 값만 업데이트:
   ```yaml
   holder1_standby: [gripper, sp, w3, w2, w1, elbow, sl]
   holder1_on_position: [gripper, sp, w3, w2, w1, elbow, sl]
   sample_holder_standby: [gripper, sp, w3, w2, w1, elbow, sl]
   sample_holder_on_position: [gripper, sp, w3, w2, w1, elbow, sl]
   ```
3. 저장 후 다시 실행 (재빌드 불필요)

## Offset 파라미터 조정

`config/taught_waypoints.yaml`에서 offset 값 변경:

```yaml
above_y_offset: -0.005     # -5mm (on_position에서 -Y 방향)
retreat_z_offset: -0.05    # -50mm (above에서 -Z 방향 후퇴)
```

## 장점

✅ **유지보수 용이**: 4개 위치만 teaching
✅ **자동 계산**: FK/IK로 정확한 상대 위치 계산
✅ **재teaching 간편**: YAML 파일만 수정, 재빌드 불필요
✅ **Multi-holder 지원**: Holder2, 3은 자동 Z offset
✅ **안전**: IK 실패 시 fallback 메커니즘

## 기존 버전

기존 하드코딩 버전 (`sample_load_sequence`)도 여전히 사용 가능:

```bash
ros2 run mtc_tutorial sample_load_sequence --ros-args \
  -p use_gripper_action:=true \
  -p use_movegroup_action:=true
```

백업 파일: `src/sample_load_sequence.cpp.backup`
