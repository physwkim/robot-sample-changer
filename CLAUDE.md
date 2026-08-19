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

### 1. EPICS IOC (필수, 보통 systemd 자동)

`robot_ioc`(Rust)가 `db/robot.db` PV를 CA로 서빙 (autosave 포함,
procServ 콘솔 20001). 수동: `~/ws/src/epics_rs_robot/target/release/robot_ioc`

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
탭: Operate(시퀀스 조작/상태) / Camera(D405 color+depth) / Calibration
(jog + 오프셋·틸트 테이블, 편집 셀만 텍스트 편집으로 저장 — 데몬의
holder-map persist와 같은 규율이라 동시 쓰기에 안전).

- CA는 브로드캐스트 search 그대로 둡니다 — 이 프로세스는 robot_ioc과
  D405 IOC 둘 다에 붙으므로 `EPICS_CA_NAME_SERVERS`/`ADDR_LIST` 금지.
- 이미지는 **pvAccess 기본**: UDP 5076도 5064처럼 여러 IOC가 공유해서
  search가 엉키므로 TCP 직결(`ROBOT_GUI_PVA_SERVER`, 기본
  `127.0.0.1:5085` = st.d405.cmd의 `EPICS_PVAS_SERVER_PORT`).
  depth(Z16, `RS405:depthPva1:Image`)는 RsdmImageView(폭 640 고정 —
  NTNDArray dimension 서브필드는 rsdm 주소로 못 읽음), color(RGB8
  ubyte, `RS405:Pva1:Image`)는 RsdmImageView가 Bytes를 못 그려서
  자체 텍스처 위젯.

`3_Camera_Viewer.desktop` → `launch_camera_viewer.sh` = 같은 바이너리
`--camera`(Camera 탭으로 시작). D405 IOC가 없으면 `run-d405-ioc.sh`로
자동 기동(procServ 콘솔 20003, systemd 유닛 없음).

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
│   │   └── src/{main,sequence,motion,gripper,epics,model,waypoints,config}.rs
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
| Robot:Holder | longout | 홀더 번호 (1-10) |
| Robot:MapSource | longout | 그립 널(6)·홀더 간 이동(7)의 소스 홀더 (0=제자리 퍽, 1-10) |
| Robot:Stop | bo | 일시정지 요청 (0=Run, 1=Pause) |
| Robot:CurrentStep | longin | 현재 실행 중인 스텝 (0-30) |
| Robot:PauseStep | longin | 지정 스텝에서 일시정지 |
| Robot:Gripper | bo | 그리퍼 명령 (0=Close, 1=Open) |
| Robot:Gripper_RBV | bi | 그리퍼 상태 피드백 (0=Close, 1=Open) |
| Robot:Loaded | bi | 샘플 로드 상태 (0=Not Loaded, 1=Loaded) |
| Robot:JogX/Y/Z | longout | TCP jog 방향 (-1/0/+1, 캘리브레이션 hold 중) |
| Robot:JogStep | ao | jog 스텝 크기 (mm) |
| Robot:Vision:Req | longout | 비전 측정 요청 id (시퀀서가 씀) |
| Robot:Vision:Kind | mbbo | 요청 종류 (0=None, 1=Pick Align, 2=Grip Offset, 3=Place Align, 4=Seating) |
| Robot:Vision:Done | longin | 응답 완료 id 에코 (비전 노드가 씀) |
| Robot:Vision:Valid | bi | 측정 유효 (0=Invalid, 1=Valid) |
| Robot:Vision:DX/DY/DZ | ao | 적용할 TCP-로컬 보정 (mm, 비전 노드가 씀) |
| Robot:Vision:Quality | ao | 검출 품질 0-1 |
| Robot:Vision:Seated | bi | 안착 판정 (0=Not Seated, 1=Seated) |
| Robot:Vision:Tilt | ao | 퍽 상면 기울기 (deg) |

### Robot:Loaded PV

측정 프로그램 연동용. Step 12 완료 후 measurement wait 시작 시 `Loaded=1`,
wait 종료(1=continue, 2=skip) 시 `Loaded=0`.

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

### 그립 널 (`CalibMode=6`)

트리거 한 번으로 그 홀더의 티칭 시트 자세를 **파지가 퍽에 하중을
주지 않는 위치**로 몰아넣습니다. 반복마다 퍽을 집고(스텝 0-5) 바로
되놓으며(20-23), 닫는 순간 툴에 남은 렌치를 읽어 그만큼 트림을
씁니다. 수렴하거나 반복 상한에 닿으면 끝납니다.

퍽은 `MapSource`가 정합니다 — 0이거나 대상과 같으면 그 홀더에 이미
앉아 있는 퍽을 쓰고, 다른 1-10이면 **먼저 그 홀더에서 가져옵니다**
(모드 7과 같은 이동, 같은 코드). 퍽 하나로 랙 전체를 돌 때 트리거
한 번에 가져오기+널이 끝나라고 있는 옵션입니다. 널이 끝나면 퍽은
**대상 홀더에 남습니다** — 다음 홀더의 소스가 바로 여기입니다.
소스를 지정할 때 **대상 시트가 비었는지는 확인하지 않습니다**(모드
7과 같은 한계).

시트 프로브 기반 홀더 맵이 있던 자리입니다. 맵은 **팔**을 퍽이 이미
닿아 있는 웰 벽에 밀어붙여서 유격만 재고 자세 오차는 못 봤습니다 —
브래킷 중심이 자기 deadband 안에 떨어져 쓰기가 아예 안 일어났습니다.
닫힘은 **패드**를 퍽에 붙이므로 접촉 비대칭이 곧 자세 오차이고, 수십
N/mm로 나타납니다. h10에서 17.8 N / 2.13 Nm였고 깨끗한 홀더(h4)는
0.3 N입니다.

보정 규칙은 세 축 균일하게 `-force / stiffness`이고, 트림 슬롯은
x↔base x, z↔base y, y↔깊이입니다. **부호는 둘 다 "닫히는 핑거가
팔을 어느 쪽으로 끄는가"라는 직관과 반대로 나옵니다** — 실측으로
정한 값이지 유도한 값이 아닙니다.

`settled_n`(0.5 N) 미만인 축은 측정 잡음으로 보고 쓰지도, 수렴
판정에 넣지도 않습니다 — 한 임계값이 두 일을 합니다. 수렴은 **연속
2회** 확인해야 선언됩니다(h8 base y가 0.9 N 다섯 번 뒤 한 번 0.11을
뱉었습니다).

`stiffness_n_per_mm`은 **첫 스텝의 씨앗일 뿐**입니다. 이후엔 루프가
자기 스텝에서 잰 기울기(신호가 실린 구간의 시컨트)로 나눕니다 —
씨앗만 믿으면 너무 뻣뻣한 값에서 기어갑니다(h8 base y가 100 N/mm로
회당 0.006 mm). 힘이 안 움직인 구간은 "기울기가 이보다 가파를 수
없다"는 상한으로 바꿔 스텝을 키웁니다. 반복당 0.5 mm, 누적 1 mm를
넘는 요구는 트림 오차가 아니라 시트 이상으로 보고 거부합니다.

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
MapSource는 타깃과 다른 1-10이어야 합니다. **타깃 시트가 비었는지는
확인하지 않습니다** — 이미 퍽이 있는 홀더로 옮기면 겹칩니다.

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
