# CLAUDE.md

ROS2 Humble - UR3e + HandE 로봇 시스템

## 빌드

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### EPICS 구조

EPICS base(libca/softIoc)는 `~/epics-base`(= `~/epics`)에 빌드돼 있습니다.
- **IOC**: `robot_ioc` (Rust, `~/codes/epics-rs` 기반)가 `robot.db` PV를 CA로 서빙
  (softIoc 대체, autosave 포함). systemd/procServ로 부팅 자동시작 — `epics_rs_robot/deploy/` 참고.
- **클라이언트**: `epics_triggered_sequence`(C++ 노드)는 **libca를 직접 링크**해
  CA로 IOC에 붙습니다(브리지 없음). `robot_gui`(pyepics)도 CA로 직접 접속.

robot_ioc 빌드 (release):
```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd ~/ws/src/epics_rs_robot && cargo build --release -p robot_ioc
```
epics_robot(C++) 빌드 — CMake가 `EPICS_BASE` 기본값 `~/epics-base`를 사용하고
바이너리에 rpath를 박으므로 별도 env 없이 libca를 찾습니다. (다른 경로면
`export EPICS_BASE=...` 후 colcon build.)

### robot_gui conda 환경 (silx)

robot_gui는 ROS를 쓰지 않는 순수 Python EPICS CA 클라이언트(silx/PyQt6/pyepics)입니다.
시스템/base Python에는 silx가 없으므로 전용 conda 환경 `robot_gui`에서 실행합니다
(`4_Robot_GUI.desktop` → `launch_robot_gui.sh`가 이 환경을 자동 activate).
환경 재생성:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda create -n robot_gui --override-channels -c conda-forge python=3.11 -y
conda activate robot_gui
python -m ensurepip --upgrade
python -m pip install silx PyQt6 pyepics numpy pyyaml
# 실행: cd ~/ws/src && python -m robot_gui.main
```

## 실행

### 1. UR 드라이버

```bash
ros2 launch ur3e_hande_robot_description ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=192.168.192.10 \
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

`robot_ioc`(Rust)가 robot.db PV를 CA로 서빙. 보통 **systemd 서비스**로 자동 실행됩니다
(`epics_rs_robot/deploy/` 참고). 수동 실행이 필요하면:

```bash
~/ws/src/epics_rs_robot/target/release/robot_ioc
```

C++ 노드(`epics_triggered_sequence`)와 robot_gui(pyepics)는 **libca로 IOC에 직접 CA 접속**합니다
(브리지 없음). 멀티홈 환경에서 "Identical PV on multiple servers" 경고가 거슬리면
`export EPICS_CA_AUTO_ADDR_LIST=NO EPICS_CA_ADDR_LIST=127.0.0.1` 로 로컬 고정.

#### 충돌/크래시 후 재개 (resume-after-crash)

상태는 PV에 보존됩니다. 노드만 죽으면 실행 중인 IOC가 PV(CurrentStep/Holder/CalibMode/Loaded)를
유지하고, IOC/전원이 재시작돼도 **autosave**(`robot_ioc/autosave/robot_state.sav`, 1초 주기)가
복원합니다. 불변식: `CurrentStep > 0` = 중단된 시퀀스, `0` = idle (정상 완료 시 노드가 0으로 리셋).

재개 절차 (운영자 수동):
1. `CurrentStep`(GUI/caget)에서 중단 지점 확인 → `StartStep`을 재개할 스텝으로 설정.
2. 프리드라이브로 로봇을 안전한 위치로 이동.
3. 티치펜던트에서 보호정지(protective stop) 해제.
4. `Robot:Trigger=1` 로 재개 (StartStep 미만 스텝은 자동 skip).

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
