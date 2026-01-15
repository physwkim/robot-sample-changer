# CLAUDE.md

ROS2 Humble - UR3e + HandE 로봇 시스템

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행

### 1. UR 드라이버

```bash
ros2 launch ur3e_hande_robot_description ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.5.10 \
    description_file:=ur_with_hande.xacro \
    runtime_config_package:=ur3e_hande_robot_description \
    controllers_file:=ur_hande_controllers.yaml \
    gripper_use_fake_hardware:=false \
    use_tool_communication:=true \
    tool_voltage:=24 \
    launch_rviz:=false
```

### 2. MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    description_package:=ur3e_hande_robot_description \
    description_file:=ur_with_hande.xacro \
    moveit_config_package:=ur3e_hande_moveit_config \
    moveit_config_file:=ur.srdf \
    launch_rviz:=false
```

### 3. EPICS IOC (EPICS Sequence 실행 전 필수)

```bash
softIoc -d db/robot.db
```

### 4. EPICS Sequence

```bash
ros2 run epics_robot epics_triggered_sequence \
    --ros-args \
    -p arm_group:=ur_manipulator \
    -p waypoints_yaml_path:=/home/stevek/ws/src/epics_robot/config/taught_waypoints.yaml \
    -p epics_trigger_pv:="Robot:Trigger" \
    -p epics_start_step_pv:="Robot:StartStep" \
    -p epics_wait_pv:="Robot:Wait" \
    -p epics_holder_pv:="Robot:Holder" \
    -p epics_stop_pv:="Robot:Stop" \
    -p epics_current_step_pv:="Robot:CurrentStep" \
    -p epics_gripper_pv:="Robot:Gripper" \
    -p epics_gripper_rbv_pv:="Robot:Gripper_RBV" \
    -p gripper_open_threshold:=0.02 \
    -p epics_pause_step_pv:="Robot:PauseStep" \
    -p epics_loaded_pv:="Robot:Loaded"
```

### 5. Stage Scene (선택사항)

```bash
ros2 run stage_scene_utils add_stage_to_scene \
    --ros-args \
    -p scale:=[0.01,0.01,0.01] \
    -p position:=[-0.15,0.39,-0.002] \
    -p rotation:=[0.0,0.0,3.14159]
```

### 6. RealSense 카메라 (선택사항)

```bash
ros2 run realsense_service realsense_service_node
```

## 구조

```
ws/
├── src/
│   ├── epics_robot/               # EPICS 시퀀스, auto_holder_exchange.py
│   ├── ur3e_hande_robot_description/
│   ├── ur3e_hande_moveit_config/
│   ├── robotiq_hande_driver/
│   ├── robotiq_hande_description/
│   ├── serial/
│   ├── stage_scene_utils/
│   ├── moveit_task_constructor/
│   ├── ros2_robotiq_gripper/
│   ├── ur_moveit_config/
│   ├── realsense_service/         # RealSense D405 카메라 서비스
│   └── robot_gui/                 # EPICS 기반 로봇 제어 GUI (silx/PyQt)
├── resources/                     # STL 메쉬 파일
├── db/robot.db                    # EPICS IOC 데이터베이스
└── .gitignore
```

## EPICS PV 레퍼런스

| PV 이름 | 타입 | 설명 |
|---------|------|------|
| Robot:Trigger | bo | 시퀀스 시작 트리거 (0=Off, 1=On) |
| Robot:Wait | mbbo | 측정 대기 상태 (0=Wait, 1=Continue, 2=Abort) |
| Robot:CalibMode | mbbo | 캘리브레이션 모드 (0=Normal, 1=Holder Calib, 2=Sample Holder Calib) |
| Robot:StartStep | longout | 시작 스텝 번호 (0-300) |
| Robot:Holder | longout | 홀더 번호 (1-10) |
| Robot:Stop | bo | 일시정지 요청 (0=Run, 1=Pause) |
| Robot:CurrentStep | longin | 현재 실행 중인 스텝 (0-30) |
| Robot:PauseStep | longin | 지정 스텝에서 일시정지 |
| Robot:Gripper | bo | 그리퍼 명령 (0=Close, 1=Open) |
| Robot:Gripper_RBV | bi | 그리퍼 상태 피드백 (0=Close, 1=Open) |
| Robot:Loaded | bi | 샘플 로드 상태 (0=Not Loaded, 1=Loaded) |

### Robot:Loaded PV

측정 프로그램 연동용 PV. Step 12 완료 후 measurement wait 시작 시 `Loaded=1`로 설정되고, wait 종료 후 `Loaded=0`으로 리셋됩니다.

## 그리퍼 통신

UR TCP:54321 → Socat → /tmp/ttyUR → Modbus RTU
