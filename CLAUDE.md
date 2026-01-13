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
    -p gripper_open_threshold:=0.02 \
    -p epics_pause_step_pv:="Robot:PauseStep"
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
│   └── realsense_service/         # RealSense D405 카메라 서비스
├── resources/                     # STL 메쉬 파일
├── db/robot.db                    # EPICS IOC 데이터베이스
└── .gitignore
```

## 그리퍼 통신

UR TCP:54321 → Socat → /tmp/ttyUR → Modbus RTU
