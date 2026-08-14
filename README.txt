================================================================================
                    UR3e + Robotiq HandE 로봇 시스템
                       순수 Rust 스택 (ROS 없음)
================================================================================

1. 개요
--------------------------------------------------------------------------------
이 시스템은 UR3e 로봇 암과 Robotiq HandE 그리퍼로 자동화된 샘플 홀더 교환
시퀀스를 수행합니다. 제어는 단일 Rust 데몬(robot-sequencer)이 담당합니다:

    - 로봇 구동: ur-driver (RTDE, 로봇에 직접 접속 — ROS 드라이버 대체)
    - 모션 플래닝/IK: cspace (Rust MoveIt 포트 — MoveIt 대체)
    - 그리퍼: robotiq-hande (TCP 54321 직접 Modbus RTU)
    - 외부 연동: EPICS CA (epics-ca-rs — robot_ioc의 PV 읽기/쓰기)

ROS2/MoveIt/colcon은 더 이상 필요 없습니다.


2. 시스템 요구사항
--------------------------------------------------------------------------------
- Ubuntu 22.04
- Rust toolchain (cargo)
- sibling 체크아웃: ~/ur-driver, ~/cspace, ~/epics-rs (robot_sequencer 빌드용)
- UR 로봇 IP: 192.168.192.10
- 로봇은 Remote Control 모드 (펜던트 프로그램 실행 불필요 — 데몬이
  headless로 external control 프로그램을 전송)


3. 빌드 방법
--------------------------------------------------------------------------------
    export PATH="$HOME/.cargo/bin:$PATH"
    cd ~/ws/src/robot_sequencer
    cargo build --release

EPICS IOC(robot_ioc)와 모니터링 IOC는 별도 빌드:

    cd ~/ws/src/epics_rs_robot && cargo build --release -p robot_ioc
    cd ~/work/epics-rs-iocs && cargo build --release -p ur-robot-ioc


4. 실행 방법
--------------------------------------------------------------------------------
[1] EPICS IOC — 보통 systemd로 자동 실행됩니다 (procServ 콘솔 20001).
    수동 실행: ~/ws/src/epics_rs_robot/target/release/robot_ioc

[2] robot-sequencer 데몬 — 바탕화면 "1_Robot_Sequencer" 아이콘, 또는:

    ~/ws/src/robot_sequencer/target/release/robot-sequencer \
        ~/ws/config/sequencer.yaml

    시작하면 EPICS trigger 대기 상태가 됩니다. 로봇이 켜져 있고
    Remote Control 모드여야 합니다.

[3] Robot GUI (선택) — 바탕화면 "2_Robot_GUI" 아이콘 (conda env 자동).

[4] UR 모니터링 IOC (선택, 읽기 전용) — deploy/ur_monitor_ioc/ 참고.
    Robot:UR: prefix로 로봇 모드/조인트/TCP 상태를 CA로 노출.

URSim 리허설(하드웨어 없이 테스트): config/sequencer_ursim.yaml 사용.


5. 좌표계 설명
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


6. Waypoint 설정 방법
--------------------------------------------------------------------------------
웨이포인트 설정 파일 위치:
    ~/ws/config/taught_waypoints.yaml

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

2. 현재 조인트 값 확인 (택 1):
   - 티치펜던트의 Move 화면에서 조인트 값 읽기
   - 모니터링 IOC 실행 중이면:
       caget Robot:UR:Receive:ActualJointPositions
     (순서: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
      — 파일 순서와 다르므로 주의)

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

트리거마다 파일을 다시 읽으므로 데몬 재시작이 필요 없습니다.
다음 Trigger=1 부터 새 값이 적용됩니다.


7. EPICS PV 제어 방법
--------------------------------------------------------------------------------
caput/caget으로 로봇을 제어할 수 있습니다:

시퀀스 시작:
    caput Robot:Trigger 1

시퀀스 일시정지 / 재개:
    caput Robot:Stop 1     # 현재 스텝 완료 후 일시정지
    caput Robot:Stop 0     # 재개

현재 스텝 확인:
    caget Robot:CurrentStep

홀더 번호 설정 (1-10):
    caput Robot:Holder 1

시작 스텝 설정:
    caput Robot:StartStep 0

지정 스텝에서 일시정지:
    caput Robot:PauseStep 15   # 스텝 15 완료 후 멈춤; 다른 값으로 바꾸면 재개

측정 대기 해제 (스텝 12 후):
    caput Robot:Wait 1     # 계속 (스텝 13-23 진행)
    caput Robot:Wait 2     # 나머지 스텝 skip

캘리브레이션 모드 설정:
    caput Robot:CalibMode 0    # 일반 모드 (전체 시퀀스)
    caput Robot:CalibMode 1    # Holder 캘리브레이션
    caput Robot:CalibMode 2    # Sample Holder 캘리브레이션


8. 캘리브레이션 모드 (위치 미세 조정)
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
    4. (시료를 잡은 상태에서 정렬 확인, 필요 시 TCP jog 사용)
    5. caput Robot:Trigger 1       # 시료 돌려놓기

[Sample Holder 캘리브레이션 모드] (CalibMode = 2)

    목적: 샘플홀더 위치 확인 및 조정
    동작: 스텝 0-8 수행 -> sample holder above에서 멈춤 -> 트리거 대기
          -> 스텝 16-23 수행

    사용 방법:
    1. caput Robot:CalibMode 2
    2. caput Robot:Holder 1        # 시료를 가져올 홀더 번호
    3. caput Robot:Trigger 1       # 시작 -> 스텝 8에서 멈춤
    4. (시료를 잡은 상태에서 샘플홀더 정렬 확인, 필요 시 TCP jog 사용)
    5. caput Robot:Trigger 1       # 시료 돌려놓기

[TCP Jog - hold 중 미세 이동]

캘리브레이션 hold 중에만 동작합니다. 방향은 TCP(Local Frame) 기준:

    caput Robot:JogStep 1.0    # 스텝 크기 (mm)
    caput Robot:JogX 1         # +X 방향으로 1스텝 (값: -1/0/+1)
    caput Robot:JogY -1
    caput Robot:JogZ 1

Jog PV는 실행 후 자동으로 0으로 리셋됩니다.

캘리브레이션 완료 후:
    caput Robot:CalibMode 0        # 일반 모드로 복귀


9. Vision 미세 보정 (기본 off)
--------------------------------------------------------------------------------
손목 카메라 노드가 Robot:Vision:* PV로 답하면 픽/플레이스 하강 직전
위치를 자동 보정합니다. config의 vision.enabled: true 로 켭니다
(기본 false — 꺼져 있으면 기존 동작과 완전히 동일).

동작 (Normal 모드 전용, 캘리브레이션 모드에는 적용되지 않음):
  - 스텝 1/12 (standby 도착 후): 픽 정렬 측정 → 스텝 2/3, 13/14 에 반영
  - 스텝 7/18 (standby 도착 후): 플레이스 정렬 측정 → 스텝 8/9, 19/20 에 반영
  - 스텝 5/16 상승 후: 그립 오프셋 측정 (다음 플레이스 보정에 합산)
  - 스텝 12/23 (standby 복귀 후): 안착(seating) 확인

측정은 above가 아니라 standby에서 합니다. above에서는 파지점이 480행짜리
화면보다 55행 아래로 투영되고 화면 중앙에는 한 칸 위 홀더가 잡힙니다
(doc/vision_correction_plan.md §12.4). standby에서는 같은 점이 (306, 330),
샘플홀더 시트가 (313, 277)로 중앙 부근입니다.

보정 게이트 (3축 합성 크기 기준):
  - min_correction (기본 0.05mm) 미만: 노이즈로 보고 무시
  - max_correction (기본 3.0mm) 이하: 적용
  - 초과: 에러 정지 — 자동 적용하지 않음. CurrentStep이 보존되므로
    원인 확인 후 아래 10. 재개 절차대로 StartStep으로 재개

vision.observe_only: true 면 측정하고 로그만 남기며 절대 이동/정지하지
않습니다 (카메라 검증 단계용).

카메라 없이 리허설: vision_sim이 고정 답으로 핸드셰이크에 응답합니다.
    ./target/release/vision_sim --dx 0.8 --dy -0.5


10. 충돌/중단 후 재개 (resume)
--------------------------------------------------------------------------------
상태는 EPICS PV에 보존됩니다 (autosave 포함 — 전원 재시작에도 유지).
CurrentStep > 0 이면 중단된 시퀀스입니다.

    1. caget Robot:CurrentStep         # 중단 지점 확인
    2. caput Robot:StartStep <스텝>    # 재개할 스텝 설정
    3. 프리드라이브로 로봇을 안전한 위치로 이동
    4. 티치펜던트에서 보호정지(protective stop) 해제
    5. caput Robot:Trigger 1           # 재개 (StartStep 미만 스텝 자동 skip)


11. 종료 방법
--------------------------------------------------------------------------------
robot-sequencer: 터미널 실행 시 Ctrl+C, 런처 실행 시 프로세스 종료.
robot_ioc(systemd): sudo systemctl stop robot-ioc


12. 문제 해결
--------------------------------------------------------------------------------
문제: 데몬이 "RTDE init: Input variable ... controlled by another RTDE client"
      오류로 종료됨
해결: 이전 robot-sequencer 프로세스가 남아 있는지 확인 후 종료
      (pgrep -a robot-sequencer). 모니터링 IOC는 receive 전용이라 무관.

문제: 로봇이 움직이지 않음
해결:
  1. 로봇이 Remote Control 모드인지 확인
  2. 보호정지/비상정지 상태 확인 (데몬이 시작 시 보호정지는 자동 해제 시도)
  3. 데몬 로그에서 bring-up 오류 확인

문제: 그리퍼가 동작하지 않음
해결:
  1. 펜던트에서 툴 통신 설정(RS485, 115200) 확인
  2. 로봇 IP로 TCP 54321 접속 가능한지 확인
  3. 다른 프로세스(이전 데몬)가 54321을 점유 중인지 확인

문제: PV가 보이지 않음 (caget 실패)
해결:
  1. robot_ioc 실행 확인 (telnet localhost 20001)
  2. 이 호스트에는 5064를 공유하는 CA 서버가 여럿 있습니다(d435i-ioc 등).
     ss -ulnp | grep 5064 로 확인. 로컬 고정이 필요하면
     export EPICS_CA_NAME_SERVERS=127.0.0.1:5064 를 쓰고,
     EPICS_CA_ADDR_LIST=127.0.0.1 은 쓰지 마세요 — search가
     유니캐스트가 되어 엉뚱한 IOC로만 배달됩니다(CLAUDE.md 참고).

문제: "no answer to request N ... within X s" 에러로 정지 (vision 켠 경우)
해결:
  1. 카메라 노드(또는 vision_sim) 실행 여부 확인
  2. 카메라 없이 돌리려면 config에서 vision.enabled: false


13. 폴더 구조
--------------------------------------------------------------------------------
ws/
├── src/
│   ├── robot_sequencer/           - Rust 시퀀스 데몬 (ur-driver + cspace)
│   ├── epics_rs_robot/            - Rust EPICS IOC(robot_ioc) + 배포(deploy)
│   └── robot_gui/                 - EPICS 기반 로봇 제어 GUI (silx/PyQt)
├── model/                         - URDF/SRDF/메쉬 (정적, xacro 불필요)
├── config/                        - sequencer.yaml, taught_waypoints.yaml
├── resources/urscript/            - external control 프로그램, RTDE recipe
├── deploy/ur_monitor_ioc/         - 읽기 전용 UR 모니터링 IOC
├── db/robot.db                    - EPICS IOC 데이터베이스
└── desktop/, scripts/             - 바탕화면 런처


================================================================================


14. 변경 이력
--------------------------------------------------------------------------------

[2026-08-11] Vision 미세 보정 훅 추가 (기본 off)

* Normal 시퀀스의 4개 하강 직전에 손목 카메라 보정 훅 — Robot:Vision:*
  PV 핸드셰이크, 게이트(무시/적용/에러 정지), observe_only 모드.
  섹션 9 참조. vision.enabled: false(기본)면 기존 동작과 동일.
* vision_sim 바이너리 추가 — 카메라 없이 URSim 리허설용 응답 시뮬레이터.

[2026-08-11] ROS 제거 — 순수 Rust 스택 전환 (ros-free 브랜치)

* ROS2/MoveIt 스택 제거, robot_sequencer(Rust 데몬) 하나로 대체
  - ur-driver(RTDE 직접 접속)가 ur_robot_driver 대체 — headless
    external control 전송, 펜던트 프로그램 불필요.
  - cspace(Rust MoveIt 포트)가 MoveIt 대체 — RRTConnect 플래닝 +
    Cartesian 보간 + TOTG 시간 파라미터라이즈, stage 충돌 씬 유지.
  - robotiq-hande가 TCP 54321 직접 Modbus RTU — socat/가상 tty 제거.
  - C++ epics_triggered_sequence의 스텝/일시정지/재개/캘리브레이션/
    그리퍼 settle 로직 전부 이식 (스텝 번호/PV 의미 동일).
* TCP jog를 EPICS PV(JogX/Y/Z, JogStep)로 통합 — 캘리브레이션 hold 중
  CA로 미세 이동 (기존 TCP 소켓 jog 대체).
* 웨이포인트 파일이 ws/config/로 이동, 트리거마다 재로딩(재시작 불필요).
* URSim 리허설 설정(sequencer_ursim.yaml) 추가 — 하드웨어 없이 전체
  시퀀스/일시정지/skip/재개/캘리브레이션/jog 검증 가능.
* 읽기 전용 UR 모니터링 IOC(deploy/ur_monitor_ioc, Robot:UR: prefix) 추가.

[2026-06-01] EPICS 계층 재정비 및 시퀀스 안정화

* 그리퍼(Hand-E) 활성화 실패 수정
  - libmodbus 응답 타임아웃을 2초로 설정 (robotiq_hande_driver).
  - 이중 socat 충돌 제거.
* EPICS IOC 를 Rust(epics-rs) 기반 robot_ioc 로 전환
  - autosave 로 실행 상태 영속화 → 크래시/전원 재시작 후에도 복원.
  - procServ + systemd 로 부팅 자동시작(src/epics_rs_robot/deploy/).
* 시퀀스 노드 안전/안정성 개선
  - 시작 시 Trigger=0 강제, 완료 시 StartStep=0 리셋, 그리퍼 settle 대기.
* 충돌/중단 후 재개(resume) 운영 절차 정립.

================================================================================
