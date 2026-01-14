================================================================================
                    UR3e + Robotiq HandE 로봇 시스템
                         ROS2 Humble 기반
================================================================================

1. 개요
--------------------------------------------------------------------------------
이 시스템은 UR3e 로봇 암과 Robotiq HandE 그리퍼를 ROS2와 MoveIt2를 통해
제어합니다. EPICS를 통해 외부 시스템과 연동하여 자동화된 샘플 홀더 교환
시퀀스를 수행할 수 있습니다.


2. 시스템 요구사항
--------------------------------------------------------------------------------
- Ubuntu 22.04
- ROS2 Humble
- EPICS Base (softIoc 명령어 사용 가능해야 함)
- UR 로봇 IP: 192.168.5.10 (기본값)


3. 빌드 방법
--------------------------------------------------------------------------------
터미널을 열고 다음 명령어를 순서대로 실행합니다:

    cd ~/ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install
    source install/setup.bash

* 참고: 새 터미널을 열 때마다 아래 두 줄을 실행해야 합니다:
    source /opt/ros/humble/setup.bash
    source ~/ws/install/setup.bash


4. 실행 방법
--------------------------------------------------------------------------------
총 4개의 터미널이 필요합니다. 각 터미널에서 먼저 환경설정을 실행하세요:

    source /opt/ros/humble/setup.bash
    source ~/ws/install/setup.bash

--------------------------------------------------------------------------------
[터미널 1] UR 로봇 드라이버 실행
--------------------------------------------------------------------------------
로봇과의 통신을 담당합니다. 로봇이 켜져 있고 네트워크 연결이 되어 있어야 합니다.

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

* 로봇 IP가 다른 경우 robot_ip:= 값을 변경하세요.
* 시작 후 로봇 티치펜던트에서 "External Control" 프로그램을 실행해야 합니다.

--------------------------------------------------------------------------------
[터미널 2] MoveIt 실행
--------------------------------------------------------------------------------
로봇 모션 플래닝을 담당합니다.

    ros2 launch ur_moveit_config ur_moveit.launch.py \
        ur_type:=ur3e \
        description_package:=ur3e_hande_robot_description \
        description_file:=ur_with_hande.xacro \
        moveit_config_package:=ur3e_hande_moveit_config \
        moveit_config_file:=ur.srdf \
        launch_rviz:=false

--------------------------------------------------------------------------------
[터미널 3] EPICS IOC 실행
--------------------------------------------------------------------------------
EPICS 프로세스 변수를 제공합니다. EPICS Sequence 실행 전에 반드시 먼저
실행해야 합니다.

    cd ~/ws
    softIoc -d db/robot.db

* 실행 후 "epics>" 프롬프트가 나타나면 정상입니다.
* 종료하려면 exit 입력 후 Enter

--------------------------------------------------------------------------------
[터미널 4] EPICS Sequence 실행
--------------------------------------------------------------------------------
자동화된 로봇 시퀀스를 실행합니다.

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


5. Stage Scene 추가 (선택사항)
--------------------------------------------------------------------------------
MoveIt에 스테이지 충돌 객체를 추가하려면 별도 터미널에서 실행:

    ros2 run stage_scene_utils add_stage_to_scene \
        --ros-args \
        -p scale:=[0.01,0.01,0.01] \
        -p position:=[-0.15,0.39,-0.002] \
        -p rotation:=[0.0,0.0,3.14159]

--------------------------------------------------------------------------------
[터미널 5] RealSense 카메라 (선택사항)
--------------------------------------------------------------------------------
Intel RealSense D405 카메라를 사용할 경우 실행합니다.
카메라가 USB로 연결되어 있어야 합니다.

    ros2 run realsense_service realsense_service_node

RViz와 함께 실행하려면:

    ros2 launch realsense_service realsense_with_rviz.launch.py

카메라 서비스 테스트:

    ros2 service call /capture_image realsense_service/srv/CaptureImage \
        "{enable_color: true, enable_depth: true, width: 848, height: 480}"

* 기본 해상도: 848x480 @ 30fps (D405 최적화)
* 토픽: /realsense_service_node/color/image_raw, /realsense_service_node/depth/image_raw
* 서비스: /capture_image, /set_camera_state


6. 좌표계 설명
--------------------------------------------------------------------------------
이 시스템에서는 두 가지 좌표계를 사용합니다:

[Global Frame - 로봇 베이스 기준 좌표계]

    로봇을 정면에서 바라볼 때:

         Z (위)
         |
         |
         +------ Y (왼쪽)
        /
       /
      X (앞쪽, 로봇이 바라보는 방향)

    - X축: 로봇 전방 (Forward)
    - Y축: 로봇 왼쪽 (Left)
    - Z축: 위쪽 (Up, 중력 반대 방향)

[Local Frame - 엔드이펙터(TCP) 기준 좌표계]

    그리퍼 끝에서 바라볼 때:

         Y (아래)
         |
         |
         +------ X (오른쪽)
        /
       /
      Z (진행 방향, 그리퍼가 접근하는 방향)

    - X축: 오른쪽 (Right)
    - Y축: 아래쪽 (Down)
    - Z축: 진행 방향 (Forward/Approach)

* 주의: Local Frame은 엔드이펙터 기준이므로, 로봇 자세에 따라
  Global Frame과 방향이 다릅니다.


7. Waypoint 설정 방법
--------------------------------------------------------------------------------
웨이포인트 설정 파일 위치:
    src/epics_robot/config/taught_waypoints.yaml

[기본 웨이포인트 - 직접 티칭 필요]

4개의 기본 위치를 티칭해야 합니다:

    1. holder1_standby       - 홀더1 대기 위치
    2. holder1_on_position   - 홀더1 샘플 집는 위치
    3. sample_holder_standby - 샘플홀더 대기 위치
    4. sample_holder_on_position - 샘플홀더 놓는 위치

조인트 값 순서 (7개):
    [gripper, shoulder_pan, wrist_3, wrist_2, wrist_1, elbow, shoulder_lift]

[웨이포인트 티칭 방법]

1. 로봇을 원하는 위치로 수동 조작 (Freedrive 모드 사용)

2. 현재 조인트 값 확인:
    ros2 topic echo /joint_states --once

3. taught_waypoints.yaml 파일에서 해당 위치 값 수정

예시:
    holder1_on_position: [0.024, -1.327, 0.0001, -1.330, -3.301, -1.380, -1.600]

[오프셋 설정 - 미세 조정용]

티칭 후 미세 조정이 필요할 때 오프셋을 사용합니다 (단위: 미터):

    # Holder1 위치 오프셋 (Local Frame 기준)
    holder1_on_position_x_offset: 0.0015   # X축 오프셋 (+: 오른쪽)
    holder1_on_position_y_offset: 0.0      # Y축 오프셋 (+: 아래쪽)
    holder1_on_position_z_offset: -0.0005  # Z축 오프셋 (+: 진행방향)

    # Sample holder 위치 오프셋 (Local Frame 기준)
    sample_holder_on_position_x_offset: 0.0015
    sample_holder_on_position_y_offset: -0.0005
    sample_holder_on_position_z_offset: -0.0005

[자동 계산 웨이포인트]

다음 웨이포인트는 on_position에서 자동 계산됩니다:

    - above: on_position에서 Y 방향으로 5mm 위로 이동
    - retreat: above에서 Z 방향으로 50mm 후퇴

    above_y_offset: -0.005   # -5mm (Local Y, 위로)
    retreat_z_offset: -0.05  # -50mm (Local Z, 후퇴)

[멀티 홀더 오프셋]

홀더 2~10번에 대한 추가 오프셋 (홀더1 기준 상대값):

    # X축 추가 오프셋 (Index 0 = 홀더2, Index 8 = 홀더10)
    holder_multi_x_offsets: [0.000, 0.000, 0.000, 0.000, 0.0015, ...]

    # Z축 추가 오프셋
    holder_multi_z_offsets: [0.00025, 0.00020, 0.00020, ...]

[설정 변경 후]

설정 파일 수정 후에는 EPICS Sequence 노드를 재시작해야 합니다.
빌드는 필요하지 않습니다 (symlink 사용).


8. EPICS PV 제어 방법
--------------------------------------------------------------------------------
EPICS IOC 터미널(터미널 3)에서 다음 명령어로 로봇을 제어할 수 있습니다:

시퀀스 시작:
    caput Robot:Trigger 1

시퀀스 정지:
    caput Robot:Stop 1

현재 스텝 확인:
    caget Robot:CurrentStep

홀더 번호 설정 (1-10):
    caput Robot:Holder 1

시작 스텝 설정:
    caput Robot:StartStep 0

캘리브레이션 모드 설정:
    caput Robot:CalibMode 0    # 일반 모드 (전체 시퀀스)
    caput Robot:CalibMode 1    # Holder 캘리브레이션
    caput Robot:CalibMode 2    # Sample Holder 캘리브레이션


9. 캘리브레이션 모드 (위치 미세 조정)
--------------------------------------------------------------------------------
홀더 또는 샘플홀더 위치를 미세 조정할 때 사용합니다.
시료를 집어서 above 위치에서 멈추므로 중심 정렬을 확인할 수 있습니다.

[Holder 캘리브레이션 모드] (CalibMode = 1)

    목적: 홀더 위치 확인 및 조정
    동작: 스텝 0-5 수행 -> above에서 멈춤 -> 트리거 대기 -> 스텝 20-23 수행

    사용 방법:
    1. caput Robot:CalibMode 1
    2. caput Robot:Holder 5        # 확인할 홀더 번호
    3. caput Robot:Trigger 1       # 시작 -> 스텝 5에서 멈춤
    4. (시료를 잡은 상태에서 정렬 확인)
    5. caput Robot:Trigger 1       # 시료 돌려놓기

[Sample Holder 캘리브레이션 모드] (CalibMode = 2)

    목적: 샘플홀더 위치 확인 및 조정
    동작: 스텝 0-8 수행 -> sample holder above에서 멈춤 -> 트리거 대기 -> 스텝 16-23 수행

    사용 방법:
    1. caput Robot:CalibMode 2
    2. caput Robot:Holder 1        # 시료를 가져올 홀더 번호
    3. caput Robot:Trigger 1       # 시작 -> 스텝 8에서 멈춤
    4. (시료를 잡은 상태에서 샘플홀더 정렬 확인)
    5. caput Robot:Trigger 1       # 시료 돌려놓기

캘리브레이션 완료 후:
    caput Robot:CalibMode 0        # 일반 모드로 복귀


10. 종료 방법
--------------------------------------------------------------------------------
각 터미널에서 Ctrl+C를 눌러 프로그램을 종료합니다.
종료 순서: 터미널 4 -> 터미널 3 -> 터미널 2 -> 터미널 1


11. 문제 해결
--------------------------------------------------------------------------------
문제: "ur.urdf.xacro 파일을 찾을 수 없습니다"
해결: colcon build를 다시 실행하고 source install/setup.bash 실행

문제: "Joint model group 'ur_arm' not found"
해결: arm_group:=ur_manipulator 파라미터가 포함되어 있는지 확인

문제: 로봇이 움직이지 않음
해결:
  1. 로봇 티치펜던트에서 External Control 프로그램 실행 확인
  2. 로봇이 Remote Control 모드인지 확인
  3. ros2 control list_controllers 명령으로 컨트롤러 상태 확인

문제: 그리퍼가 동작하지 않음
해결:
  1. tool_voltage:=24 설정 확인
  2. gripper_use_fake_hardware:=false 설정 확인
  3. use_tool_communication:=true 설정 확인


12. 폴더 구조
--------------------------------------------------------------------------------
ws/
├── src/
│   ├── epics_robot/               - EPICS 연동 시퀀스 노드
│   ├── ur3e_hande_robot_description/ - 로봇 URDF 및 설정
│   ├── ur3e_hande_moveit_config/  - MoveIt 설정
│   ├── robotiq_hande_driver/      - 그리퍼 드라이버
│   ├── stage_scene_utils/         - 스테이지 씬 유틸리티
│   ├── realsense_service/         - RealSense D405 카메라 서비스
│   └── ...
├── db/
│   └── robot.db                   - EPICS IOC 데이터베이스
└── resources/                     - STL 메쉬 파일


================================================================================
