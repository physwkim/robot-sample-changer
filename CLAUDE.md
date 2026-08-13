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

모니터링 IOC용 `epics-rs-iocs`도 같은 위치 규칙(`/home/bl9b/epics-rs-iocs`).
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

### 3. robot_gui (선택)

순수 Python EPICS CA 클라이언트(silx/PyQt6/pyepics), conda env `robot_gui`.
`2_Robot_GUI.desktop` → `launch_robot_gui.sh`. 환경 재생성:

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda create -n robot_gui --override-channels -c conda-forge python=3.11 -y
conda activate robot_gui
python -m ensurepip --upgrade
python -m pip install silx PyQt6 pyepics numpy pyyaml
# 실행: cd ~/ws/src && python -m robot_gui.main
```

### 4. UR 모니터링 IOC (선택, 읽기 전용)

`deploy/ur_monitor_ioc/` — epics-rs-iocs ur-robot IOC의 dashboard +
RTDE receive만 로드, `Robot:UR:` prefix (조인트/TCP/안전 상태).
control/io/jog/gripper 포트는 시퀀서와 배타적이라 제외. procServ 20002.

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

## 구조

```
ws/
├── src/
│   ├── robot_sequencer/   # Rust 시퀀스 데몬 (독립 cargo workspace)
│   │   └── src/{main,sequence,motion,gripper,epics,model,waypoints,config}.rs
│   ├── epics_rs_robot/    # Rust EPICS IOC(robot_ioc) + deploy (기존 유지)
│   └── robot_gui/         # EPICS 기반 GUI (silx/PyQt, conda)
├── model/                 # 정적 URDF/SRDF/메쉬 (ur3e + ur5e URSim용)
├── config/                # sequencer.yaml, sequencer_ursim.yaml, taught_waypoints.yaml
├── resources/urscript/    # external_control.urscript, RTDE recipe
├── deploy/ur_monitor_ioc/ # 읽기 전용 UR 모니터링 IOC
├── db/robot.db            # EPICS IOC 데이터베이스
├── desktop/ scripts/      # 데스크톱 런처
└── doc/
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

`sequencer.yaml`의 `vision.enabled: true`면 Normal 모드의 4개 하강
(스텝 3/9/14/20) 직전에 look-then-move 보정이 걸립니다: 시퀀서가
Req(id)/Kind로 측정을 요청하고, 비전 노드가 DX/DY/DZ(mm, TCP-로컬)를
쓰고 Done에 id를 에코. deadband(`min_correction`) 미만은 무시,
`max_correction` 초과·Valid=0·타임아웃은 시퀀스 정지(StartStep 재개는
기존과 동일). 스텝 5/16 후 파지 편차(Grip Offset)를 측정해 다음 놓기
보정에 합산하고, 스텝 11/22 후 안착 검사(Seated/Tilt) — 불합격이면 정지.
`observe_only: true`는 Phase C 관찰 모드(측정·로그만, 이동/정지 없음).
캘리브레이션 모드에는 훅이 없습니다(티칭 오차를 가리므로). 카메라 없는
리허설은 `vision_sim` 바이너리가 비전 노드를 대신합니다
(`vision_sim --dx 0.8 --grip-dx 0.3` 등, src/bin/vision_sim.rs).

## 그리퍼 통신

robotiq-hande(Rust)가 UR 툴 통신 포워더 TCP 54321에 직접 Modbus RTU —
socat / 가상 tty 없음. URSim에는 툴 장치가 없으므로 리허설 설정은
`gripper.mode: "simulated"`.
