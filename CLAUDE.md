# CLAUDE.md

UR3e + Robotiq Hand-E 샘플 체인저 — **순수 Rust 스택** (ROS 없음)

시퀀스 데몬 `robot_sequencer`가 ur-driver(RTDE)로 로봇을 직접 구동하고,
cspace(Rust MoveIt 포트)로 플래닝/IK, epics-ca-rs로 EPICS CA 통신,
robotiq-hande로 그리퍼를 제어합니다. 이전 ROS2 스택(ur_robot_driver +
MoveIt + C++ 노드)은 `ros-free` 브랜치에서 제거됐습니다.

## 의존 체크아웃 (sibling)

`src/robot_sequencer/Cargo.toml`이 `../../../` 상대 경로로 참조:

| 크레이트 | 경로 (bl9b) | 경로 (개발머신) |
|----------|-------------|----------------|
| ur-driver, robotiq-hande | `/home/bl9b/ur-driver` | `~/work/ur-driver` |
| cspace-{core,collision,planning,planners} | `/home/bl9b/cspace` | `~/work/cspace` |
| epics-ca-rs | `/home/bl9b/epics-rs` | `~/work/epics-rs` |

모니터링 IOC용 `epics-rs-iocs`도 같은 위치 규칙 — 이 호스트의 실제 체크아웃은
`/home/bl9b/work/epics-rs-iocs`이고 `/home/bl9b/epics-rs-iocs`는 없습니다.
robot_ioc(`src/epics_rs_robot`)은 기존 그대로 `/home/bl9b/codes/epics-rs`
(v0.18.6)를 고정 참조 — 개발머신에서는 빌드되지 않으며 변경하지 않았습니다.

## 빌드

```bash
export PATH="$HOME/.cargo/bin:$PATH"
cd ~/ws/src/robot_sequencer && cargo build --release
# robot_ioc (bl9b에서만):
cd ~/ws/src/epics_rs_robot && cargo build --release -p robot_ioc
# 모니터링 IOC:
cd ~/epics-rs-iocs && cargo build --release -p ur-robot-ioc
```

테스트/린트: `cargo nextest run` / `cargo clippy --all-targets -- -D warnings`
(robot_sequencer는 독립 workspace라 crate 디렉토리에서 실행).

## 실행

### 1. EPICS IOC (필수)

`robot_ioc`(Rust)가 `db/robot.db` PV를 CA로 서빙 (autosave 포함,
procServ 콘솔 20001).

**유저 레벨 systemd**로 돕니다 — `~/.config/systemd/user/robot-ioc.service`,
`enabled` + linger on이라 부팅부터 뜹니다. 다른 IOC 둘과 같은 방식이고
sudo가 필요 없습니다(20001·5064 모두 비특권, 경로는 전부 홈 아래).

```bash
systemctl --user status|restart|stop robot-ioc
```

`0_Robot_IOC.desktop` = `scripts/launch_robot_ioc.sh` — 유닛을 띄우고
PV가 답하는지까지 확인합니다. 시퀀서보다 먼저입니다.

`ROBOT_DB`는 반드시 이 저장소의 `db/`입니다 — 바이너리의 컴파일 기본값
`~/ws/db`는 CalibMode 3-7 라벨도 `Robot:MapSource`도 없는 구 사본이라
그립 널과 홀더 간 이동이 쓰는 PV가 사라집니다. 유닛이 이걸 명시하고,
런처는 `Robot:MapSource`가 있는지로 어느 db가 올라왔는지 확인합니다.

`/etc/systemd/system/robot-ioc.service`에 구 시스템 유닛 사본이 남아
있으면 지우세요(`ROBOT_DB=~/ws/db`를 들고 있고, 이름이 시스템·유저
양쪽에 존재하게 됩니다) — `src/epics_rs_robot/deploy/README.md`.

### 2. robot-sequencer 데몬

```bash
~/ws/src/robot_sequencer/target/release/robot-sequencer ~/ws/config/sequencer.yaml
```

데스크톱 런처 `1_Robot_Sequencer.desktop` = `scripts/launch_robot_sequencer.sh`
(중복 실행 방지 포함). 로봇은 **Remote Control 모드**여야 하며, 펜던트
프로그램은 필요 없음 — 데몬이 headless로 external control 프로그램을
전송합니다. 시작 시 Trigger=0 강제(재시작 직후 stale trigger 자동실행 방지).

- `config/sequencer.yaml` — 실기 (192.168.192.10, ur3e, Hand-E TCP 54321)
- `config/sequencer_ursim.yaml` — URSim 리허설 (192.168.56.101, ur5e,
  simulated gripper, stage scene 없음)

### 3. robot_gui_rs (선택)

Rust RsDM GUI (`src/robot_gui_rs`, 독립 cargo workspace) — rsdm/rsplot을
`../../../rsplot` 상대 경로로 참조하므로 `~/work/rsplot` 체크아웃 필요.
빌드: `cd src/robot_gui_rs && cargo build --release`.

`2_Robot_GUI.desktop` → `launch_robot_gui.sh` → `robot-gui <waypoints.yaml>`.
탭 둘입니다 — **Operate**(State: 상태 + 그립 널 결과 / Run: 마운트·그립
널·퍽 이동·그리퍼·Advanced)와 **Teach**(캘리브레이션 hold, jog + 누적·apply,
오프셋·틸트 테이블). Teach 맨 위 **Calibration hold**가 홀더 N의 퍽을
집어 그 시트 위(모드 1) 또는 스테이지 위(모드 2)에 세웁니다 — Apply가
설 자리가 있는 hold가 이 둘뿐이라 표의 Holder 행과 Stage 행이 각각
여기서 나옵니다. hold를 끝내는 건 `Wait`이 아니라 **두 번째 Trigger**
이고(`calibration_hold`가 `wait_for_trigger`), 버튼은 데몬이 hold에 서
있다고 말할 때만(`State=4` + 살아 있는 `Alive`) 눌립니다 — 안 그러면
hold를 끝내는 게 아니라 `CalibMode`에 남아 있던 모드로 새 런을 띄웁니다.
`Jog:Target`은 시트 이름을 라벨에 쓸 뿐 게이트가 아닙니다(그 값은 데몬
보다 오래 살아남습니다 — "Robot:State / Robot:Alive" 절). Teach만 라이브 상태가 아니라 티칭 파일을 편집하므로 갈랐고,
스크롤 위치도 탭마다 따로 답니다. 테이블은 편집 셀만 텍스트 편집으로
저장하므로(데몬의 트림 persist와 같은 규율) 동시 쓰기에 안전합니다.

카메라는 헤더의 **Camera window** 버튼으로 여는 별도 네이티브 창입니다
(`--camera`는 카메라 전용 프로세스로 뜨는 기존 동작 유지). 두 이미지 모두
같은 위젯이라 조작이 같습니다 — 휠로 확대, 드래그로 이동, 호버하면
픽셀 좌표와 값(깊이는 mm + counts, 컬러는 RGB). 그 값은 툴팁이 아니라
**커서 옆에 직접 그립니다** — 툴팁은 위젯(=패널 전체)에 붙어서 가장자리에
서고 잠깐 머물러야 뜨는데, 이건 힌트가 아니라 프로브라 커서가 닿는 프레임에
바로 답해야 합니다. 깊이 컬러맵 범위는
Auto(2-98 퍼센타일) 또는 counts 직접 입력입니다. `RSDepthUnits_RBV`가
mm 환산 계수라 상수로 박지 않습니다. silx식 ImageView(측면 히스토그램·
프로파일 도구)는 이 카메라에 묻는 질문이 아니라 뺐습니다.

멀티홈이라 CA 비컨이 인터페이스마다 두 번 들어오고 클라이언트가 이걸
"IOC 재시작"으로 읽습니다. GUI 로그 필터가 `epics_ca_rs::client=error`로
그 경고만 내립니다(진짜 장애는 화면에 DISCONNECTED로 뜹니다).

State의 **그립 널 결과**는 `Robot:Null:` PV를 그대로 읽습니다 — 상태,
반복 번호, 누적 보정 세 축, 마지막 닫힘 렌치, 한 줄 메시지.

Status에 **TCP 렌치가 실시간으로** 뜹니다(Force N / Torque Nm,
각 축 + 크기). 소스는 ur-monitor-ioc의 `Robot:UR:Receive:ActualTCPForce`
— 그쪽 RTDE receive 스트림이라 시퀀서와 경합하지 않습니다. 프레임은
**base**이고, 그립 널이 쓰는 트림과 같은 축입니다(x→x 트림, y→z 트림,
z→깊이(y) 트림).

- CA는 브로드캐스트 search 그대로 둡니다 — 이 프로세스는 robot_ioc,
  D405 IOC, ur-monitor-ioc 셋 모두에 붙으므로
  `EPICS_CA_NAME_SERVERS`/`ADDR_LIST` 금지.
- 이미지는 **pvAccess 기본**: UDP 5076도 5064처럼 여러 IOC가 공유해서
  search가 엉키므로 TCP 직결(`ROBOT_GUI_PVA_SERVER`, 기본
  `127.0.0.1:5085` = st.d405.cmd의 `EPICS_PVAS_SERVER_PORT`).
  depth(Z16, `RS405:depthPva1:Image`)는 RsdmImageView(폭 640 고정 —
  NTNDArray dimension 서브필드는 rsdm 주소로 못 읽음), color(RGB8
  ubyte, `RS405:Pva1:Image`)는 RsdmImageView가 Bytes를 못 그려서
  자체 텍스처 위젯.

`3_Camera_Viewer.desktop` → `launch_camera_viewer.sh` = 같은 바이너리
`--camera`(Camera 탭으로 시작). D405 IOC가 없으면 `run-d405-ioc.sh`로
자동 기동(procServ 콘솔 20003). 유저 유닛 `d405-ioc.service`도 있지만
`disabled`이라 부팅 시에는 뜨지 않습니다 — `systemctl --user start d405-ioc`.
**스트리밍 중에 강제 종료하지 마세요**: RealSense 펌웨어가 걸린 채 남아
USB 재연결까지 필요했던 적이 있습니다. 유닛의 `ExecStop`이 `Acquire 0`을
먼저 씁니다.

구 Python GUI(silx/PyQt6/pyepics, conda env `robot_gui`)도 그대로 실행
가능: `cd ~/ws/src && python -m robot_gui.main`. 오프셋 저장은 RsDM
GUI와 같은 텍스트 편집(주석 보존, tmp+rename, 재파싱 검증)입니다.

### 4. UR 모니터링 IOC (선택, 읽기 전용)

`deploy/ur_monitor_ioc/` — epics-rs-iocs ur-robot IOC의 dashboard +
RTDE receive만 로드, `Robot:UR:` prefix (조인트/TCP/안전 상태).
control/io/jog/gripper 포트는 시퀀서와 배타적이라 제외. procServ 20002.

**유저 레벨 systemd**로 돕니다 (`~/.config/systemd/user/ur-monitor-ioc.service`,
`systemctl --user status|restart ur-monitor-ioc`, linger on). robot_ioc의
시스템 유닛과 달리 sudo가 필요 없습니다 — 읽기 전용에 20002/5064만 씁니다.

멀티홈 환경에서 CA 경고("Identical process variable names on multiple
servers" — 같은 IOC가 두 인터페이스로 답해서 뜨는 무해한 경고)가 거슬리면:
`export EPICS_CA_NAME_SERVERS=127.0.0.1:5064`

**`EPICS_CA_ADDR_LIST=127.0.0.1`(+`AUTO_ADDR_LIST=NO`)는 쓰지 마세요.**
search가 유니캐스트로 바뀌는데, 이 호스트에는 5064를 공유하는 CA 서버가
robot_ioc 말고도 있습니다(d435i-ioc 등). 브로드캐스트 search는 커널이
바인딩된 모든 소켓에 사본을 주지만, 유니캐스트는 그중 하나에만 배달되므로
엉뚱한 IOC가 받으면 `Robot:*`를 못 찾고 5초 뒤 "PV is not connected"로
데몬이 죽습니다. `NAME_SERVERS`는 TCP로 특정 서버에 직접 질의해서 로컬
고정과 정상 동작을 둘 다 얻습니다.

## 충돌/크래시 후 재개 (resume-after-crash)

상태는 PV에 보존. 데몬만 죽으면 IOC가 PV(CurrentStep/Holder/CalibMode/Loaded)
유지, IOC/전원 재시작도 autosave(`robot_ioc/autosave/robot_state.sav`, 1초)가
복원. 불변식: `CurrentStep > 0` = 중단된 시퀀스, `0` = idle.

1. `CurrentStep` 확인 → `StartStep`을 재개 스텝으로 설정.
2. 프리드라이브로 로봇을 안전 위치로 이동.
3. 티치펜던트에서 보호정지 해제.
4. `Robot:Trigger=1` 재개 (StartStep 미만 스텝 자동 skip).

보호정지(충격감지)는 **데몬 재시작 없이** 복구됩니다: external-control
프로그램이 죽어도 다음 트리거가 자동 재전송하고(`ensure_program`),
보호정지 해제는 CalibMode=4(Recover) 트리거에 게이트되어 unlock →
재전송 → 스탠바이 복귀가 한 번에 됩니다. 펜던트도, 데몬 재시작도
필요 없습니다(재시작은 Hand-E 재활성화로 파지를 풀 수 있어 금지).

## 구조

```
ws/
├── src/
│   ├── robot_sequencer/   # Rust 시퀀스 데몬 (독립 cargo workspace)
│   │   └── src/{main,sequence,motion,gripper,epics,model,waypoints,
│   │             config,seatcheck}.rs
│   ├── epics_rs_robot/    # Rust EPICS IOC(robot_ioc) + deploy (기존 유지)
│   ├── robot_gui_rs/      # RsDM GUI (Rust, 독립 workspace, ~/work/rsplot 참조)
│   └── robot_gui/         # 구 Python GUI (silx/PyQt, conda) — 실행 가능
├── model/                 # 정적 URDF/SRDF/메쉬 (ur3e + ur5e URSim용)
├── config/                # sequencer.yaml, sequencer_ursim.yaml, taught_waypoints.yaml
├── resources/urscript/    # external_control.urscript, RTDE recipe
├── deploy/ur_monitor_ioc/ # 읽기 전용 UR 모니터링 IOC
├── db/robot.db            # EPICS IOC 데이터베이스
├── display/               # pydm 스크린 (d435i_dual_view.py, CA 폴백용)
├── desktop/ scripts/      # 데스크톱 런처
└── doc/
```

## EPICS PV 레퍼런스

| PV 이름 | 타입 | 설명 |
|---------|------|------|
| Robot:Trigger | bo | 시퀀스 시작 트리거 (0=Off, 1=On) |
| Robot:Wait | mbbo | 측정 대기 상태 (0=Wait, 1=Continue, 2=Abort) |
| Robot:CalibMode | mbbo | 캘리브레이션 모드 (0=Normal, 1=Holder Calib, 2=Sample Holder Calib, 3=Hand-Eye Calib, 4=Recover, 5=Seat Probe, 6=Grip Null, 7=Holder Transfer) |
| Robot:StartStep | longout | 시작 스텝 번호 (0-300) |
| Robot:Holder | longout | 시트 번호 (1-10 = 랙 홀더, 0 = 스테이지, 그립 널 전용) |
| Robot:MapSource | longout | 그립 널(6)·홀더 간 이동(7)의 소스 홀더 (0=제자리 퍽, 1-10) |
| Robot:Stop | bo | 일시정지 요청 (0=Run, 1=Pause) |
| Robot:CurrentStep | longin | 현재 실행 중인 스텝 (0-30) |
| Robot:State | longin | 데몬이 서 있는 루프 (0=Idle, 1=Running, 2=MeasWait, 3=Paused, 4=Hold) |
| Robot:Alive | longin | 서비스 패스마다 +1 하는 하트비트 (Running 중에는 멈춤) |
| Robot:PauseStep | longin | 지정 스텝에서 일시정지 |
| Robot:Gripper | bo | 그리퍼 명령 (0=Close, 1=Open) |
| Robot:Gripper_RBV | bi | 그리퍼 상태 피드백 (0=Close, 1=Open) |
| Robot:Loaded | bi | 샘플 로드 상태 (0=Not Loaded, 1=Loaded) |
| Robot:SeatCheck | bo | 카메라 시트 점유 확인 스위치 (0=Off, 1=On, autosave 없음) |
| Robot:JogX/Y/Z | longout | TCP jog 방향 (-1/0/+1, 툴 프레임) |
| Robot:JogStep | ao | jog 스텝 크기 (mm, 0.01-10) |
| Robot:Jog:DX/DY/DZ | ai | 이번 런에서 jog한 누적량 (mm, 툴 x/y/z) |
| Robot:Jog:Target | stringin | 누적량을 쓸 시트 (빈 문자열 = 없음) |
| Robot:Jog:Apply | bo | 누적량을 그 시트 트림에 더해 쓰기 |
| Robot:Vision:Req | longout | 비전 측정 요청 id (시퀀서가 씀) |
| Robot:Vision:Kind | mbbo | 요청 종류 (0=None, 1=Pick Align, 2=Grip Offset, 3=Place Align, 4=Seating) |
| Robot:Vision:Done | longin | 응답 완료 id 에코 (비전 노드가 씀) |
| Robot:Vision:Valid | bi | 측정 유효 (0=Invalid, 1=Valid) |
| Robot:Vision:DX/DY/DZ | ao | 적용할 TCP-로컬 보정 (mm, 비전 노드가 씀) |
| Robot:Vision:Quality | ao | 검출 품질 0-1 |
| Robot:Vision:Seated | bi | 안착 판정 (0=Not Seated, 1=Seated) |
| Robot:Vision:Tilt | ao | 퍽 상면 기울기 (deg) |
| Robot:Null:State | longin | 그립 널 상태 (0=Idle, 1=Running, 2=Settled, 3=Failed) |
| Robot:Null:Iter | longin | 진행 중인 반복 번호 |
| Robot:Null:DX/DY/DZ | ai | 누적 보정 (mm, 툴 x / 툴 y=깊이 / 툴 z; 깊이는 조향 안 하므로 항상 0) |
| Robot:Null:Force | ai | 마지막 닫힘 렌치 크기 (N, 조향 축만 — 깊이 제외) |
| Robot:Null:Msg | stringin | 결과 한 줄 (39자) |

### Jog와 Jog Apply

jog는 **데몬이 서 있는 모든 대기 상태**에서 동작합니다 — idle 트리거
대기, 캘리브레이션 hold, hand-eye aiming, 시트 프로브 hold, PauseStep
정지, measurement wait. 궤적을 실행하는 중에는 서비스되지 않습니다
(그건 jog가 아니라 이동 중단입니다). idle에서 jog해도 안전합니다: 모든
런이 스텝 0-1에서 티칭된 standby로 계획 이동하며 시작하므로 다음
트리거가 되돌립니다.

누적량은 **대기에 진입할 때마다 0**이 됩니다. 대기는 데몬이 명령한
자세에서 시작하므로 "이 대기가 열린 뒤 jog한 양" = 그 자세로부터의
변위이고, 그게 곧 트림입니다. 앞선 대기의 누적을 끌고 오면 그 사이의
티칭 이동이 이미 되돌린 jog까지 세게 됩니다. 0으로 만드는 곳은 여기와
apply가 누적량을 소비할 때 둘뿐입니다.

누적량 `Robot:Jog:D*`는 **툴 프레임 mm**입니다. `Motion::jog`와
`Model::apply_cartesian_offset`이 같은 강체 이동이라 누적량이 곧
트림이고, 따로 프레임을 달고 다닐 필요가 없습니다(비전 `Correction`과
다른 점).

`Robot:Jog:Apply=1`이면 그 누적량을 `Robot:Jog:Target`이 가리키는 시트의
x/y/z 트림에 **더해서** taught_waypoints.yaml에 쓰고 누적량을 0으로
되돌립니다. 움직이지 않은 축은 슬롯을 아예 건드리지 않습니다. 반영은
**다음 트리거**부터입니다(파일을 그때 다시 읽으므로) — 진행 중인 런의
복귀 스텝은 집어 온 자리로 그대로 돌아갑니다.

Target은 hold가 서 있는 시트이지 `Robot:Holder`가 아닙니다 — 모드 2는
`Holder`가 랙을 가리키는 채로 스테이지 위에 섭니다. 그래서 시트가 있는
곳은 캘리브레이션 hold 둘뿐이고(모드 1 = 그 홀더, 모드 2 = 스테이지),
나머지 대기에서는 Target이 비어 있고 Apply는 거부됩니다. 트림은 시트의
`on_position`을 움직이는 값이라 standby에서 jog한 양은 그 측정이
아닙니다.

### Robot:State / Robot:Alive — GUI 버튼의 유일한 게이트

데몬은 **서비스 패스**(`service_hold`)에서만 운전자 명령을 읽습니다 —
`Robot:Gripper`, jog, `Jog:Apply`, `Wait`, 두 번째 `Trigger`. 이 패스는
멈춰 서 있는 루프(`wait_for_trigger` / `wait_for_measurement` /
`wait_for_pause_step_change`)에서만 돌고, 팔이 움직이는 동안에는 돌지
않습니다. 그래서 GUI의 컨트롤은 **`State`(어느 루프인지) + `Alive`(아직
그 말을 하고 있는지) 둘로만** 열립니다. `Alive`가 2초(서비스 패스 20회)
멈추면 데몬이 안 듣는 것으로 보고 해당 컨트롤을 회색 처리합니다.

`State=1`(Running)은 "일하는 중, 명령 안 읽음"이고 그동안 `Alive`는
멈춥니다 — 정상입니다. 그래서 데몬은 **막히는 일 앞뒤로 Running을
찍습니다**(`while_moving`): 스텝 실행뿐 아니라 서비스 패스 안의 jog
모션과 그리퍼 명령도 포함. 덕분에 "Running이면 조용한 게 정상, 서 있는
상태(Idle/MeasWait/Paused/Hold)인데 조용하면 죽은 것"이 유일한 규칙이
되고, GUI는 jog 한 번에 빨간 NOT RESPONDING을 띄우지 않습니다(노랑
"running — no commands read").

두 레코드는 **autosave 대상이 아닙니다**(`robot_state.req`). 그게 요점
입니다 — `CurrentStep`·`Loaded`·`Jog:Target`은 재개용 마커라 데몬보다
오래 살아남고, IOC가 autosave에서 복원까지 합니다. 그 값으로 버튼을
열면 죽은 런의 잔해 위에서 버튼이 눌립니다:

- `CurrentStep == 12`로 연 Continue → 아무도 안 읽는 `Wait`에 씀
- `Jog:Target`으로 연 hold 종료 버튼 → hold를 끝내는 게 아니라
  `CalibMode`에 남은 모드로 **새 런을 띄움**
- 스텝 실행 중 그리퍼 Open/Close → 눌리지만 움직이지 않음

규칙: **데몬이 지금 그 상태를 서비스하고 있다고 말할 때만 컨트롤을
연다.** 런 값은 라벨(어느 시트인지)에만 쓰고 게이트에는 쓰지 않습니다.
`Robot:State`를 쓰는 주인은 `set_state` 하나뿐이고, 항상 현재 beat와
같이 씁니다.

### Robot:Loaded PV

측정 프로그램 연동용. Step 12 완료 후 measurement wait 시작 시 `Loaded=1`,
그리고 **wait 지점을 지나는 모든 경로에서** `Loaded=0` — continue,
skip, 그리고 wait 자체가 없는 회수 errand(`StartStep=13`)까지. 회수
errand가 0을 쓰는 이유는 그 1을 세운 마운트 런이 wait에서 끝났기
때문입니다. 내리는 주인은 회수 leg 하나뿐입니다.

### Vision 미세 보정 (기본 off)

`sequencer.yaml`의 `vision.enabled: true`면 Normal 모드에 look-then-move
보정이 걸립니다: 시퀀서가 Req(id)/Kind로 측정을 요청하고, 비전 노드가
DX/DY/DZ(mm, TCP-로컬)를 쓰고 Done에 id를 에코. deadband(`min_correction`)
미만은 무시, `max_correction` 초과·Valid=0·타임아웃은 시퀀스 정지(StartStep
재개는 기존과 동일).

측정은 **standby 자세**에서 합니다 — 스텝 1/12 후 픽 정렬, 스텝 7/18 후
플레이스 정렬. 보정은 뒤따르는 above/on 쌍(2·3, 8·9, 13·14, 19·20)에
적용됩니다. above에서 재지 않는 이유는 거기서 파지점이 480행 화면보다
55행 아래로 투영되고 화면 중앙이 한 칸 위 홀더이기 때문입니다
(doc/vision_correction_plan.md §12.4). 측정 자세와 적용 자세가 달라서
보정량은 관측 자세의 툴 프레임을 달고 다니며(`Correction`), 적용 시
대상 자세의 툴 프레임으로 회전됩니다. 스텝 5/16 후 파지 편차(Grip
Offset)를 측정해 다음 놓기 보정에 합산하고, 스텝 12/23 후 안착
검사(Seated/Tilt) — 불합격이면 정지.
`observe_only: true`는 Phase C 관찰 모드(측정·로그만, 이동/정지 없음).
캘리브레이션 모드에는 훅이 없습니다(티칭 오차를 가리므로). 카메라 없는
리허설은 `vision_sim` 바이너리가 비전 노드를 대신합니다
(`vision_sim --dx 0.8 --grip-dx 0.3` 등, src/bin/vision_sim.rs).

### 시트 점유 확인 (`seat_check`, 기본 off / 실기 on)

팔이 시트에 들어가기 **전에** D405 depth로 그 시트가 스텝이 가정하는
상태인지 묻습니다 — 집으러 가면 퍽이 있어야 하고, 놓으러 가면 비어
있어야 합니다. 아니면 그 자리에서 정지(StartStep 재개는 기존과 동일).
게이트는 Normal의 1·7·12·18, 그리고 `carry_puck`의 1·18(모드 7과
모드 6의 가져오기)입니다. 그립 널의 반복 자체에는 없습니다 — 집기는
`empty_close`가 이미 막고, 놓기는 방금 꺼낸 그 시트입니다.

규칙을 **픽셀이 아니라 기하로** 씁니다. 창은 시트 자신의 자세에서
툴 x ±5 mm, 툴 y −6…+2 mm인 사각형이고, 물을 때마다
`Model::fk` → `T_ee_cam` → depth 내부파라미터로 투영합니다. 하드코딩
ROI는 카메라 마운트와 티칭된 standby **양쪽**에 묶이고, 둘 중 하나가
움직이면 **조용히** 틀립니다(엉뚱한 패치의 중앙값도 중앙값이라서).
그래서 standby를 다시 티칭하는 비용은 0이고, **카메라를 옮기면
`CalibMode=3` 한 번과 재풀이로 `T_ee_cam.yaml`만 갱신**하면 됩니다.

판정값은 창의 중앙값에서 **그 시트 파지점의 투영 거리를 뺀 값**입니다
(절대 거리가 아니라 차이라서 시트마다 다른 사거리를 저절로 흡수합니다
— 랙은 136-142 mm, 스테이지는 208 mm). 2026-08-19 전 시트 실측:

- 점유: −3.3(h1) −4.1(h5) −4.5(h4) −5.0(h6) −4.3/−5.0(스테이지)
- 비움: +40대가 h2 h3 h4 h7 h8 h9, **+16.9(h1) +17.2(h10)**,
  스테이지는 +512(보어가 뚫려 있어 먼 배경이 보입니다)

h1·h10은 웰 바닥이 파지점 17 mm 뒤에 보이고 나머지는 웰을 통과해
버립니다. 그래서 임계는 랙 다수만 보고 정한 15 mm가 아니라 **+7 mm**
— 양쪽에 10 mm씩 남고, depth의 프레임간 요동 ±1 mm의 열 배입니다.

유효 픽셀 하한은 10%입니다. 비율이 판정의 품질을 재지 못하기
때문입니다 — h7은 연속 6프레임에서 21-29%인데 답은 +40.4…+42.0으로
같고, 못 읽는 픽셀이 프레임마다 **같은 자리**라 프레임을 쌓아도
24%에서 안 올라갑니다(실측). 하한은 중앙값을 낼 표본이 아예 없는
경우만 걸러냅니다.

읽지 못하면(하한 미달, 프레임 미도착) **경고만 하고 계속**합니다.
진짜 보호는 여전히 렌치이고(0.2 mm가 4.4 N), 깊이 프레임이 안 왔다고
런을 거부하면 지키는 것 없이 가용성만 깎습니다. 프레임이 안 오는
가장 흔한 원인은 `RS405:image2:EnableCallbacks`가 Disable인 것이고,
경고문이 그걸 지목합니다.

**간극(clearance) 검사가 아닙니다.** 홀더 standby에서 1 px는
0.34 mm, `T_ee_cam` 병진의 1σ가 0.45 mm, depth 시간잡음이 ±1 mm인데
랙 웰 유격은 ±0.032 mm입니다. 점유는 스케일이 다른 질문이라 되는
것뿐입니다.

운영 메모: 게이트가 켜진 상태에서 스테이지에 퍽이 남아 있으면
Normal 런은 스텝 7에서 정지합니다(그게 맞습니다). 치우는 건
`StartStep=13` 한 번 — GUI의 "Return from stage"가 쓰는 값이고,
데몬이 먼저 스테이지 standby까지 계획해서 간 뒤 13-17로 퍽을 집어
18-23으로 랙에 넣습니다.

**끄는 스위치: `Robot:SeatCheck`** (GUI Advanced의 "Seat check"
체크박스). `sequencer.yaml`의 `seat_check.enabled`는 검사가 존재
하는지를 정하고(핸드아이 파일을 읽습니다), 이 레코드는 그게 도는지를
정합니다 — 데몬이 게이트마다 한 번 읽으므로 런 도중에 꺼도 다음
시트부터 듣습니다. 읽기가 실패하면(레코드 없음, 타임아웃) **켜짐**
으로 답합니다. autosave에 넣지 않은 것도 같은 이유입니다: 꺼둔 안전
검사가 IOC 재시작으로 조용히 되살아나거나 조용히 꺼져 있으면 안
됩니다. 레코드는 On으로 올라오고, 끄는 건 장비 앞에 선 사람이 하는
결정입니다.

### 그립 널 (`CalibMode=6`)

트리거 한 번으로 그 시트의 티칭 자세를 **파지가 퍽에 하중을
주지 않는 위치**로 몰아넣습니다. 반복마다 퍽을 집고(스텝 0-5) 바로
되놓으며(20-23), 닫는 순간 툴에 남은 렌치를 읽어 그만큼 트림을
씁니다. 수렴하거나 반복 상한에 닿으면 끝납니다.

**닫힘은 반복당 2회이고, 두 값이 어긋나면 3회째로 갈라 중앙값을
씁니다.** 한 번의 닫힘은 여기서 측정이 아닙니다 — 한 자세에서
닫음마다 흩어지는 폭이 `settled_n`의 상당 부분이고, 가끔 아예 다른
데 앉습니다. h10에서 여섯 번이 base y로 −1.17 −0.98 −1.03 **+1.81**
−1.13 −0.89 N을 읽었고 팔은 매번 같은 자세 0.01 mm 안에 있었습니다
(2026-08-20). 나머지 성분도 같이 움직여서(base z 4.97 대 4.0-4.2,
base Tx −0.475 대 −0.39 Nm) 노이즈 낀 표본이 아니라 다르게 앉은
파지이고, 그건 다시 닫아 봐야만 갈립니다.

루프는 그 한 번을 못 견딥니다. 이상치는 정의상 이웃과 `settled_n`
이상 떨어져 있어서 **시컨트가 진짜 응답으로 읽습니다** — 그 위에서
반대로 한 스텝 가고, 되돌아오는 값을 그 잘못된 스텝으로 나눕니다.
h10에서 4-6반복이 0.008 mm를 **뒤로** 갔고, 총 0.038 mm를 움직인
채 반복이 끝났습니다. 어긋남의 기준과 중앙값의 축 선택은 모두
`settled_n`과 `NULL_STEERED`를 그대로 씁니다 — 3회째가 지키는 건
스텝이고, 스텝은 조향 축밖에 안 읽습니다(깊이가 아무리 벌어져도
3회째를 사지 않습니다). 중앙값 자체는 세 성분 모두에 겁니다.

**`Holder`가 시트를 고릅니다 — 1-10은 랙, 0은 스테이지**입니다.
스테이지는 `sample_holder_on_position_{x,y,z}_offset` 스칼라 셋에
쓰고(랙은 `holder_multi_*_offsets` 리스트), 그 밖에는 같은 루프
같은 규칙입니다. 스테이지에서는 `MapSource`가 0이어야 합니다 —
가져오기는 랙↔랙이고, 스테이지 위 퍽은 Normal 런이 올려놓는 것이라
거기서 이미 앉아 있는 퍽만 씁니다. `Holder=0`은 그립 널 전용이고,
다른 모드는 `compute_run_waypoints`가 거부합니다(랙 피치를 0번째로
외삽해서 홀더 1보다 한 칸 앞으로 가는 일이 없도록).

랙에서 퍽은 `MapSource`가 정합니다 — 0이거나 대상과 같으면 그 홀더에 이미
앉아 있는 퍽을 쓰고, 다른 1-10이면 **먼저 그 홀더에서 가져옵니다**
(모드 7과 같은 이동, 같은 코드). 퍽 하나로 랙 전체를 돌 때 트리거
한 번에 가져오기+널이 끝나라고 있는 옵션입니다. 널이 끝나면 퍽은
**대상 홀더에 남습니다** — 다음 홀더의 소스가 바로 여기입니다.
소스를 지정할 때 대상 시트가 비었는지는 `seat_check`가 봅니다(모드
7과 같은 경로) — 꺼져 있으면 아무도 안 봅니다.

시트 프로브 기반 홀더 맵이 있던 자리입니다. 맵은 **팔**을 퍽이 이미
닿아 있는 웰 벽에 밀어붙여서 유격만 재고 자세 오차는 못 봤습니다 —
브래킷 중심이 자기 deadband 안에 떨어져 쓰기가 아예 안 일어났습니다.
닫힘은 **패드**를 퍽에 붙이므로 접촉 비대칭이 곧 자세 오차이고, 수십
N/mm로 나타납니다. h10에서 17.8 N / 2.13 Nm였고 깨끗한 홀더(h4)는
0.3 N입니다.

보정 규칙은 세 축 균일하게 `sign * force / stiffness`이고, **툴
프레임**에서 씁니다. 트림 슬롯이 곧 툴 오프셋이기 때문입니다 —
`Model::apply_cartesian_offset`이 툴 축으로 평행이동하므로 x/y/z
트림은 툴 x/y/z입니다. 그래서 루프는 닫힘 렌치(base)를 그 시트의 툴
프레임으로 돌린 뒤(`Axes::say`) 축별로 나눕니다. 랙 시트에서 이
회전은 x→x, base y→툴 z, base z→툴 y(깊이)로 떨어져 예전의 하드코딩
순열과 같아지고(테스트
`the_tool_frame_rule_reproduces_the_rack_mapping`이 이걸 고정합니다),
접근축 기준 92° 돌아가 있는 스테이지에서는 다르게 떨어집니다 —
한 규칙이 두 시트를 다 덮는 이유입니다.

`NULL_TOOL_SIGN`의 **부호 둘은 "닫히는 핑거가 팔을 어느 쪽으로
끄는가"라는 직관과 반대로 나옵니다** — 실측으로 정한 값이지 유도한
값이 아닙니다. 스테이지에서도 같은 부호가 맞습니다: 툴 x를 일부러
+0.20 mm 밀었더니 닫힘이 +4.37 N(≈22 N/mm, 랙 씨앗과 같은 자릿수),
이 부호로 네 번에 +0.27 N까지 되돌아왔습니다. 반경 유격 0.50 mm짜리
보어가 웰의 열 배로 헐거워도 **측방 측정을 무디게 하지는 않습니다**.

**깊이는 두 시트 어디서도 조향하지 않습니다**(`NULL_STEERED`가 그 축을
끕니다). 힘이 없어서가 아니라 **그 힘의 0이 엉뚱한 곳**에 있어서입니다.
앉은 퍽 아래에는 여유가 있고 닫힘은 퍽을 눌러 내리는 게 아니라 핑거
디텐트에 매답니다. 그래서 패드가 퍽 어깨 밑으로 걸릴 만큼 깊어지기
전까지는 아무것도 안 만납니다 — h1에서 티칭 대비 +0.20 mm는 +0.18 N,
+0.50 mm는 +0.05 N(정상 자세의 +0.18 N과 같음), 그러다 **+1.00 mm에서
−8.27 N**(base Tx 0.88 Nm)입니다. 기울기가 아니라 계단입니다. 그
8.27 N을 되돌리는 데 0.242 mm면 0.10 N까지 떨어졌고(≈34 N/mm), 걸림
가장자리는 티칭 **아래 0.7 mm**쯤이며 그 위는 평평합니다.

즉 깊이 힘의 0은 시트가 원하는 깊이가 아니라 **여유 공간의 가장자리**
입니다. 거기로 널을 맞추면 다음 드리프트가 바로 무는 자리에 자세를
세워둡니다 — 그 h1 런은 반복이 끝날 때까지 주입한 오차 중 +0.758 mm를
그대로 안고 있었고, 계속 돌았으면 그 값을 트림으로 썼습니다. 스테이지가
티칭 자세에서 2.05 N을 읽는 것도 그 자세가 이미 가장자리에 있어서이고,
물러날수록 1.93, 1.53, 0.92 N으로 맞물림이 풀릴 뿐(기울기 37 → 2.5 N/mm)
−0.93 mm에서 닫힘이 3.9가 아닌 **7.5 mm**에 멈춥니다(넥을 놓치고 더 넓은
면을 잡음). 퍽을 쥐고 있는 구간 안에 0은 없습니다.

내려가는 동안 닿는 게 아닙니다: +1.00 mm에서도 시트에 선 채 핑거를 연
팔은 (+2.76, −4.37, −5.84) N으로 티칭의 (+2.75, −4.11, −5.86) N과
구분되지 않고, 8 N은 전부 **닫는 순간** 생깁니다.

`stiffness_n_per_mm`도 같은 순서(툴 x, 툴 y=깊이, 툴 z)입니다.

`settled_n`(0.5 N) 미만인 축은 측정 잡음으로 보고 쓰지도, 수렴
판정에 넣지도 않습니다 — 한 임계값이 두 일을 합니다. 수렴은 **연속
2회** 확인해야 선언됩니다(h8 툴 z가 0.9 N 다섯 번 뒤 한 번 0.11을
뱉었습니다).

`stiffness_n_per_mm`은 **첫 스텝의 씨앗일 뿐**입니다. 이후엔 루프가
자기 스텝에서 잰 기울기(신호가 실린 구간의 시컨트)로 나눕니다 —
씨앗만 믿으면 너무 뻣뻣한 값에서 기어갑니다(h8 툴 z가 100 N/mm로
회당 0.006 mm). 힘이 안 움직인 구간은 "기울기가 이보다 가파를 수
없다"는 상한으로 바꿔 스텝을 키웁니다. 반복당 0.5 mm, 누적 1 mm를
넘는 요구는 트림 오차가 아니라 시트 이상으로 보고 거부합니다.

진행과 결과는 `Robot:Null:` PV로 나갑니다(State/Iter/DX·DY·DZ/Force/Msg
— 위 표). 종료 상태는 `run_grip_null` 한 곳에서만 찍히고 루프의 에러
타입이 메시지를 들고 다니므로, 이동 실패로 `?`가 튀어도 화면에 "running"이
남지 않습니다. IOC db에 레코드가 없으면 데몬은 조용히 발행을 건너뜁니다.

`taught_waypoints.yaml`은 텍스트 편집(주석 보존, tmp+rename, 재파싱
검증)으로 갱신하고, 반복마다 그 파일을 다시 읽어 다음 집기에
반영합니다. StartStep은 0이어야 합니다(중간 재개는 빈 손 파지나
이미 찬 시트로의 릴리즈가 되므로 거부). **집으러 가는 홀더(소스 또는
대상)에 퍽이 있어야 합니다** — 빈 시트는 `empty_close`가 잡아
중단합니다. 실패해도 퍽은 시트에, 팔은 스탠바이에 있고 데몬은 트리거
대기로 돌아갑니다.

### 홀더 간 이동 (`CalibMode=7`)

`MapSource`의 퍽을 `Holder`로 **바로** 옮깁니다. 소스에서 후퇴한 뒤
타깃 standby로 곧장 계획 이동하며, 아무것도 측정하거나 쓰지 않습니다.

스텝 번호는 Normal 시퀀스 것을 씁니다 — 0-6이 소스에서 집기, 18-23이
타깃에 놓기 — 그래서 PauseStep/CurrentStep이 평소대로 동작합니다. StartStep은 0이어야
하며(중간 재개는 빈 손 파지나 이미 찬 시트로의 릴리즈가 되므로 거부),
MapSource는 타깃과 다른 1-10이어야 합니다. 타깃 시트가 비었는지는
`seat_check`(스텝 18)가 봅니다 — **꺼져 있으면 아무도 안 보고**, 이미
퍽이 있는 홀더로 옮기면 겹칩니다.

### Hand-eye 캘리브레이션 (`CalibMode=3`)

비전 노드가 픽셀을 TCP-로컬 보정으로 바꾸려면 먼저 `T_ee_cam`이 있어야
합니다. 그 수집을 데몬이 직접 합니다 — 수집 도구는 팔을 **움직여야**
하는데, 쓰는 경로는 전부 하나뿐입니다. 별도 도구로 하려면 프로덕션
데몬을 내려야 하고, 그건 순서가 거꾸로입니다.

**읽기는 다중, 쓰기는 하나.** 읽기 — RTDE 출력(30004)은 URControl 5.16이
클라이언트별 레시피로 다중화하고, 대시보드(29999)도 동시 접속을 받습니다.
데몬이 자기 스트림을 물고 있는 상태에서 두 번째 RTDE 클라이언트가 125 Hz로
받고 대시보드 2개가 모두 `robotmode`에 답하는 것을 실측했습니다.
`deploy/ur_monitor_ioc/`가 이걸 전제로 합니다. 쓰기는 강제 방식이 셋 다
다릅니다:

- **프로그램 슬롯** — URScript 프로그램은 한 번에 하나. 30001로 새로
  보내면 돌던 것이 교체됩니다. 진짜 단일 소유자.
- **RTDE 입력 레지스터 / speed slider** — 서버가 두 번째 클라이언트를
  거절하지 **않습니다**. 각자 입력 레시피를 등록할 수 있고 나중에 쓴 쪽이
  이깁니다. 거절이 아니라 조용한 덮어쓰기 (소스 기준 추론, 미실측).
- **50001/50003/50004, 그리퍼 54321** — ur-driver가 서버이고
  `"Only one connection is allowed at a time"`로 직접 거절합니다.

**트리거는 2회**입니다 — 다른 캘리브레이션 모드와 같은 패턴(모드 진입 →
jog hold → 커밋). idle 트리거 대기는 jog를 서비스하지 않습니다(그 자리는
티칭된 standby 자세라 jog하면 시퀀스 시작점이 움직입니다).

```bash
caput Robot:CalibMode 3
caput Robot:Trigger 1     # 1회차: aiming hold 진입, 검출기 기동
# 로그에 1초마다 "tag: ... centre (x, y)"가 뜹니다.
# JogX/Y/Z + JogStep으로 100 mm 태그(id 0)를 화면 중앙에 맞춥니다.
# "not detected from here"가 계속 뜨면 여기서 멈추는 게 맞습니다.
caput Robot:Trigger 1     # 2회차: 현재 자세를 home으로 잡고 수집 시작
# 끝나면
<handeye.solve_python> tools/handeye/solve_joint.py <out_dir>/samples_<timestamp>.yaml
```

2회차 트리거 시점의 자세를 home으로 잡고 툴 x/y/z축 둘레 12자세를 돌며(축 3개는
필수 — 회전축을 공유하는 두 상대운동은 AX = XB에서 축퇴) 자세마다 home
복귀. 이동은 계획(RRT) 이동이 아니라 직선 보간 이동이며, 충돌 검사는
시퀀스와 같은 씬을 씁니다. level-tool 제약을 끄는 가드는 없습니다 —
그 제약은 `Motion::move_planned`에서만 읽히고 수집은 전부 `move_direct`라
애초에 걸리지 않습니다. 대신 수집이 끝나면 `handeye_return`이 모드 진입
당시 자세로 팔을 되돌립니다(그러지 않으면 다음 트리거의 step 1이 계획에
실패하고 데몬이 죽습니다). 각도는
`handeye.angle_deg`.

수집 실패(태그 미검출, 검출기 미기동, 표본 부족)는 데몬을 죽이지 않습니다
— 로봇은 이상 없고 팔은 시작 자세로 돌아와 있으므로 로그만 남기고 트리거
대기로 복귀합니다. 이동 실패는 다른 스텝과 동일하게 데몬 종료.
`CurrentStep`은 건드리지 않습니다(재개 대상이 아니므로 불변식 유지).

`db/robot.db`에 상태 3(`THST`/`THVL`)을 추가했지만 **IOC 재시작은 라벨용**
입니다. robot_ioc의 mbbo는 정의되지 않은 상태값도 그대로 받아 서빙하므로
(실측: 구 db로 뜬 IOC에 `caput 3` → readback 3, SEVR=0), 데몬은 재시작
없이도 3을 읽습니다. 재시작 전에는 `caget -s`/GUI가 "Hand-Eye Calib"
대신 "3"으로 보일 뿐입니다.

## 그리퍼 통신

robotiq-hande(Rust)가 UR 툴 통신 포워더 TCP 54321에 직접 Modbus RTU —
socat / 가상 tty 없음. URSim에는 툴 장치가 없으므로 리허설 설정은
`gripper.mode: "simulated"`.
