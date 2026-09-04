# Channel Access 클라이언트 인터페이스 — 2026-09-04

`CLAUDE.md`는 **무엇이 있는지**를 적는다(PV 표, 모드, 스텝 번호). 이
문서는 **GUI 없이 CA로만 로봇을 부리는 프로그램**이 쓸 수 있게 그
문법을 모은다 — 어떤 레코드에 어떤 숫자를 어떤 순서로 쓰고, 무엇을
읽어 끝났는지·거절됐는지를 판단하는가.

대상은 빔라인 측정 프로그램이다. 장비 앞에 선 사람은 GUI를 쓰고, 이
문서의 절반(티칭·jog·hold)은 애초에 원격에서 할 일이 아니다.

---

## 0. 전제

- **IOC가 `db/robot.db`의 현재 판을 서빙해야 한다.** `Robot:Cmd`·
  `Robot:CmdArg`·`Robot:CmdArg2`는 2026-09-04에 들어갔다. 없는 IOC에서는
  `caput`이 `Channel connect timed out`으로 떨어지고, 데몬은 명령을 아예
  보지 못한다(채널이 optional이라 로그에 한 줄 남기고 넘어간다).
  §7의 옛 방식은 그대로 유효하다.
- **`robot-sequencer` 데몬이 떠 있어야 한다.** IOC만으로는 레코드에 값이
  들어갈 뿐 아무것도 움직이지 않는다. §4로 살아 있는지 먼저 본다.
- **CA 환경변수를 좁히지 말 것.** 이 호스트에는 5064를 공유하는 CA 서버가
  여럿이다(robot_ioc, ur-monitor-ioc, D405 IOC). `EPICS_CA_ADDR_LIST=127.0.0.1`
  + `AUTO_ADDR_LIST=NO`로 유니캐스트 search를 쓰면 커널이 그중 **하나**
  에만 배달해서 엉뚱한 IOC가 받고 `Robot:*`를 못 찾는다. 브로드캐스트
  search를 그대로 두거나, 고정이 필요하면
  `EPICS_CA_NAME_SERVERS=127.0.0.1:5064`(TCP 직결)를 쓴다.

---

## 1. 명령 한 줄 — `Robot:Cmd`

런에 필요한 값은 레코드 넷(`CalibMode`·`StartStep`·`Holder`·`MapSource`)에
있지만 그중 셋은 데몬의 어휘다 — 여덟 모드 중 몇 번인지, 24스텝
스크립트의 몇 번째에서 시작하는지. `Robot:Cmd`는 errand를 이름으로 받고
번호는 데몬이 채운다.

| `Robot:Cmd` | 명령 | `Robot:CmdArg` | `Robot:CmdArg2` |
|---|---|---|---|
| 0 | None — 대기. 데몬이 명령을 집으면 되돌려 놓는 값 | — | — |
| 1 | Mount — 홀더의 퍽을 스테이지로 | 홀더 **1–10** | 안 씀 |
| 2 | Unmount — 스테이지에 **남은** 퍽을 홀더로 | 홀더 **1–10** | 안 씀 |
| 3 | Divert — **이미 든** 퍽을 홀더로 | 홀더 **1–10** | 안 씀 |
| 4 | Move — 홀더에서 홀더로 | 목적지 **1–10** | 출발 **1–10**, 목적지와 달라야 함 |
| 5 | Recover — 보호정지 해제 + 스탠바이 복귀 | 안 씀 | 안 씀 |
| 6 | Grip Null — 그 시트의 티칭 자세 보정 | 시트 **0–10** (0=스테이지) | 소스 **0–10** (0=그 자리 퍽) |

"안 씀"은 무시된다는 뜻이다. 남아 있던 값이 새어 들어가지 않도록 데몬이
0으로 채워 넣는다. Recover만은 `Robot:Holder`에 이미 있는 값을 쓴다 —
멈춘 런의 standby로 돌아가는 것이 일이라 자기 시트가 없다.

데몬이 채우는 값:

| 명령 | CalibMode | StartStep | Holder | MapSource |
|---|---|---|---|---|
| Mount | 0 Normal | 0 | CmdArg | 0 |
| Unmount | 0 Normal | **13** | CmdArg | 0 |
| Divert | 0 Normal | **18** | CmdArg | 0 |
| Move | 7 Holder Transfer | 0 | CmdArg | CmdArg2 |
| Recover | 4 Recover | 0 | (그대로) | 0 |
| Grip Null | 6 Grip Null | 0 | CmdArg | CmdArg2 |

### 쓰는 순서

**인자를 먼저, `Robot:Cmd`를 마지막에.** 데몬은 `Cmd`가 0이 아닌 것을 본
그 패스에서 인자 둘을 읽는다. `Cmd`를 먼저 쓰면 직전 런의 인자가 읽힐 수
있다.

```bash
caput Robot:CmdArg 3 ; caput Robot:Cmd 1                          # 홀더 3 마운트
caput Robot:CmdArg 3 ; caput Robot:Cmd 2                          # 홀더 3으로 언마운트
caput Robot:CmdArg 5 ; caput Robot:Cmd 3                          # 든 퍽을 홀더 5에
caput Robot:CmdArg 7 ; caput Robot:CmdArg2 2 ; caput Robot:Cmd 4  # 홀더 2 → 홀더 7
caput Robot:Cmd 5                                                 # 복구
caput Robot:CmdArg 0 ; caput Robot:CmdArg2 4 ; caput Robot:Cmd 6  # 홀더 4 퍽으로 스테이지 널
```

### 언제 읽히는가

데몬은 **idle 트리거 대기에서만** 이 레코드를 읽는다(100 ms 주기). 읽는
즉시 0으로 되돌리므로 한 번 쓰면 한 번 돈다.

팔이 움직이는 동안, 또는 hold·측정 대기에 서 있는 동안 쓴 명령은
**실행되지 않는다**. 그 대기는 이미 파라미터가 정해진 런 안에 서 있고,
거기서 시작하는 Mount를 요청한 사람은 없다. 그 값은 다음 대기 진입에서
버려지며 데몬 로그에 이름이 남는다(`Dropped Cmd …`). 그러니
**`Robot:State`가 0(Idle)일 때만 보낸다**(§4).

확장 결과는 네 레코드에 되쓴다. 그것이 런의 상태이기 때문이다 — 재개
경로가 `StartStep`·`Holder`를 읽고, autosave가 IOC 재시작 너머로 그걸
나르고, GUI 둘이 그걸로 "무엇이 도는지"를 말한다.

`Robot:Cmd`는 **autosave 대상이 아니다.** 저장된 명령이 IOC 재시작으로
되살아나면 아무도 안 본 사이에 팔이 움직인다.

---

## 2. 거절 — 두 단계

### 2.1 명령 자체가 말이 안 될 때

아무것도 쓰지 않고 거절한다. `Robot:Cmd`만 0으로 돌아가고
`Holder`·`StartStep`·`CalibMode`·`MapSource`는 그대로다. 이유는
`Robot:Status`(39자)에 남는다.

| 상황 | `Robot:Status` |
|---|---|
| Mount/Unmount/Divert의 CmdArg가 0이거나 11 이상 | `Cmd Mount: seat must be 1-10` (명령 이름은 그때 것) |
| Move의 CmdArg2가 1–10 밖 | `Cmd Move: source must be 1-10` |
| Move의 출발 = 목적지 | `Cmd Move: source is the target` |
| Grip Null의 CmdArg가 0–10 밖 | `Cmd Grip Null: seat must be 0-10` |
| Grip Null의 CmdArg2가 0–10 밖 | `Cmd Grip Null: source must be 0-10` |
| `Robot:Cmd`가 7 이상이거나 음수 | `Cmd: no such command` |

### 2.2 명령은 맞지만 런을 시작할 수 없을 때

명령이 확장된 뒤, 팔이 움직이기 **전에** 걸리는 검사가 있다. 역시
아무것도 움직이지 않고 `CurrentStep`·`StartStep`도 그대로다.

| `Robot:Status` | 뜻 |
|---|---|
| `not started: the fingers hold a puck` | 핑거가 퍽을 물고 있는데 이 런은 집기로 시작한다. Divert(3)로 먼저 내려놓는다 |
| `not started: nothing in the fingers` | 핑거가 비었는데 이 런은 놓기로 시작한다. Divert를 빈 손으로 보낸 경우 |
| `not started: fingers shut on nothing` | 핑거가 빈 채로 닫혀 있다. 열어야 한다(GUI Gripper: Open) |

런이 시작한 뒤 멈추면 `Robot:Status`가 `STOP: …`로 시작한다. 흔한 것:

| `STOP: …` | 뜻 |
|---|---|
| `STOP: seat check @…: the seat r…` | 시트가 스텝이 가정한 상태가 아니다(집으러 갔는데 비었거나, 놓으러 갔는데 찼다) |
| `STOP: the fingers hold a puck bef…` | 런 도중에 핑거 상태가 스텝과 어긋났다 |
| `STOP: open_gripper_final: the fin…` | 열기 명령이 먹지 않았다(Hand-E 활성화 상실 의심) |

전문은 데몬 로그에 있다. `stringin`이 39자라 잘리지만 **앞에서부터**
남으므로 어떤 검사가 거절했는지는 항상 보인다.

---

## 3. 마운트 → 측정 → 반납 한 판

**Mount 한 번이 왕복 전부다.** 스텝 0-12가 퍽을 스테이지에 올리고 측정
대기에 서고, `Robot:Wait`에 1을 쓰면 그 **같은 런**이 13-23으로 퍽을 원래
홀더에 되돌린다. 반납을 위해 명령을 다시 보내지 않는다.

```bash
# 1) 데몬이 idle인지 확인
[ "$(caget -t Robot:State)" = "0" ] || exit 1

# 2) 홀더 3 마운트
caput Robot:CmdArg 3
caput Robot:Cmd 1

# 3) 측정 대기까지 기다린다: State=2, Loaded=1
#    (거절이면 State는 0에 머물고 Status가 이유를 말한다)
while [ "$(caget -t Robot:State)" != "2" ]; do
    case "$(caget -t -s Robot:Status)" in
        "Cmd "*|"not started:"*|"STOP: "*) echo "거절/정지: $(caget -t -s Robot:Status)"; exit 1;;
    esac
    sleep 1
done

# 4) 측정한다 … 끝나면 계속 진행 → 팔이 퍽을 홀더 3에 되돌린다
caput Robot:Wait 1

# 5) 반납까지 끝나면 State=0, Status=idle - waiting for a trigger
while [ "$(caget -t Robot:State)" != "0" ]; do sleep 1; done
```

### 측정 대기에서는 `Robot:Cmd`가 안 먹는다

여기서 `Robot:Cmd`에 2(Unmount)를 써도 **아무 일도 일어나지 않는다.**
명령은 idle 트리거 대기에서만 읽히고(§1), 측정 대기는 이미 파라미터가
정해진 런 **안에** 서 있다. 그 자리에서 읽는 것은 `Robot:Wait` 하나다.

```bash
caput Robot:Wait 1    # 계속 — 13-23이 퍽을 홀더로 되돌린다
caput Robot:Wait 2    # 건너뛰기 — 런이 여기서 끝나고 퍽은 스테이지에 남는다
```

써 둔 2는 다음 대기가 열릴 때 버려지고 데몬 로그에 이름이 남는다
(`Dropped Cmd …`). 레코드에 값이 남아 다음 idle에서 뒤늦게 도는 일은
없다.

**Unmount(2)는 그 런이 없어졌을 때** 쓴다 — `Wait=2`로 건너뛰어 퍽을
스테이지에 두고 끝냈거나, 데몬이 죽어 다시 떴거나, `STOP:`으로 멈춘
런을 접었을 때. StartStep 13으로 들어가 스테이지 standby까지 계획해서
간 뒤 13-23으로 퍽을 홀더에 넣는다.

### `Robot:Wait`을 미리 쓰지 말 것

대기는 **열리면서** 명령 레코드를 비운다(`Trigger`·`Wait`·jog·
`Jog:Apply`·`Gripper`). 대기가 열리기 전에 써 둔 1은 그 비움에 지워진다.
`State=2`를 본 뒤에 쓴다.

읽기 규칙은 비대칭이다 — **0만이 "계속 기다려라"이고 2만이 Skip이며,
1도 그 밖의 값도 읽기 실패도 전부 Continue다.** 측정이 끝나지 않았는데
0이 아닌 값이 들어가면 팔이 움직인다.

## 4. 읽는 쪽 — 지금 무슨 일이 일어나는가

| PV | 타입 | 값 |
|---|---|---|
| `Robot:State` | longin | 0=Idle 1=Running 2=MeasWait 3=Paused 4=Hold |
| `Robot:Alive` | longin | 서비스 패스마다 +1. **Running 중에는 멈춘다** |
| `Robot:Status` | stringin | 지금 하는 일, 또는 멈춘 이유 (39자) |
| `Robot:CurrentStep` | longin | 실행 중인 스텝 (0-23) |
| `Robot:Loaded` | bi | 1 = 측정 대기 중(스테이지에 시료가 있다) |
| `Robot:Seat:Stage`, `Robot:Seat:H1`..`H10` | longin | 0=모름 1=빔 2=참 |
| `Robot:Seat:Msg` | stringin | 마지막 시트 검사 한 줄 |
| `Robot:Gripper_RBV` | bi | 0=Close 1=Open |

**데몬이 살아 있는지**는 `State`와 `Alive` 둘로 판단한다:

- `State`가 0/2/3/4(서 있는 상태)인데 `Alive`가 2초 넘게 안 변하면
  데몬이 안 듣는 것이다. 명령을 보내도 소용없다.
- `State=1`(Running)이면 `Alive`가 멈춰 있는 것이 **정상**이다. 일하는
  중이고 명령을 읽지 않는다.

명령을 보내도 되는 조건은 하나다: **`State=0` 이고 `Alive`가 움직인다.**

**시트 레코드는 관측이지 추측이 아니다.** 아무도 가보지 않은 시트는
0(모름)으로 남는다. IOC 재시작 뒤에는 전부 0이다(autosave 대상이 아니다 —
복원된 "참"은 그 사이 손으로 옮긴 퍽을 못 본다).

---

## 5. 명령으로 감싸지 않은 것들

레코드 하나로 끝나는 일은 그대로 쓴다.

| 일 | 쓰기 |
|---|---|
| 측정 계속 | `Robot:Wait` = 1 |
| 남은 스텝 건너뛰기 | `Robot:Wait` = 2 |
| 일시정지 / 해제 | `Robot:Stop` = 1 / 0 |
| 특정 스텝에서 정지 | `Robot:PauseStep` = 스텝 번호 |
| 시트 카메라 검사 끄기/켜기 | `Robot:SeatCheck` = 0 / 1 |

`Stop`·`PauseStep`·`SeatCheck`는 "지금부터 이렇게 두라"는 **레벨**이라
대기가 열려도 지워지지 않는다. `Wait`·`Trigger`·`Cmd`·jog·`Gripper`는
"한 번 하라"는 **one-shot**이라 대기가 열릴 때 비워진다.

`Robot:Stop`은 **스텝 경계에서** 걸린다(다음 스텝 앞에서 선다). 궤적
중간에 팔을 세우지 않는다 — 비상정지는 펜던트의 몫이다. 멈춰 있는 동안
`Robot:Status`가 `paused by Robot:Stop before step N`이라고 말한다.

티칭 모드(`CalibMode` 1 Holder Calib, 2 Sample Holder Calib, 3 Hand-Eye,
5 Seat Probe)는 명령에 넣지 않았다. jog hold에 서서 사람이 손으로 맞추는
일이라 원격 한 줄로 시작할 것이 아니다.

---

## 6. 걸렸을 때

멈춘 런은 `CurrentStep`을 남긴다(불변식: `>0`이면 중단된 시퀀스, `0`이면
idle). 이유는 `Robot:Status`에 있다.

| 증상 | 답 |
|---|---|
| 보호정지 / 충격 감지 | `Robot:Cmd` = 5 (Recover). unlock → 프로그램 재전송 → 스탠바이가 한 번에 된다. 데몬 재시작은 **금지** — Hand-E 재활성화가 파지를 푼다 |
| 스테이지가 차서 스텝 8 앞에서 멈춤 | 든 퍽을 다른 홀더에: `CmdArg`=그 홀더, `Cmd`=3 (Divert) |
| 원래 홀더가 차서 스텝 19 앞에서 멈춤 | 같음 — `Cmd`=3 |
| 스테이지에 퍽이 남아 Normal이 스텝 7에서 멈춤 | `CmdArg`=넣을 홀더, `Cmd`=2 (Unmount) |
| 퍽을 든 채 Mount를 눌러 거절됨 | 먼저 `Cmd`=3으로 내려놓는다 |

**퍽을 든 채로 Mount(1)를 보내면 거절한다**(§2.2). 스텝 0의 열기가 선
자리에서 퍽을 떨구기 때문이다.

중간 스텝에서 직접 재개해야 하면 옛 방식(§7)으로 `StartStep`을 직접 쓴다.
명령 여섯 개는 사람이 쓰는 재개 지점(0·13·18)만 덮는다.

---

## 7. 옛 방식 — 레코드 넷 + 트리거

`Robot:Cmd`가 없는 IOC에서도, 있는 IOC에서도 그대로 동작한다. 명령이
덮지 않는 재개 지점이 필요하면 이쪽을 쓴다.

```bash
caput Robot:Holder 3
caput Robot:CalibMode 0
caput Robot:StartStep 0
caput Robot:Trigger 1
```

`Robot:Trigger`도 one-shot이라 데몬이 집으면서 0으로 되돌린다. 네 값은
**트리거를 집는 그 순간에 한꺼번에** 읽힌다 — 그러니 트리거를 마지막에
쓴다. 두 사람이 각자 준비하다 섞이는 것을 완전히 막지는 못한다(CA에
원자성이 없다). 창이 네 번의 get으로 좁혀져 있을 뿐이다.

`CalibMode` 값: 0 Normal, 1 Holder Calib, 2 Sample Holder Calib,
3 Hand-Eye Calib, 4 Recover, 5 Seat Probe, 6 Grip Null, 7 Holder Transfer.

`StartStep` 재개 지점: 0 처음부터, 13 스테이지에서 회수, 18 든 퍽을 홀더에.

---

## 8. 코드 기준

`db/robot.db`(레코드 셋), `src/robot_sequencer/src/sequence.rs`
(`Command::expand`, `Sequencer::take_command`),
`src/robot_sequencer/src/epics.rs`(채널과 접근자),
`src/robot_sequencer/src/config.rs`(PV 이름 기본값).

명령의 확장 표와 거절 문구는 `sequence.rs`의 테스트
`a_command_expands_to_the_records_an_operator_would_have_set`,
`a_command_refuses_a_seat_its_errand_has_no_use_for`,
`only_the_six_command_codes_name_a_command`가 고정한다. 이 문서의 §1·§2.1
표가 바뀌면 그 테스트가 먼저 깨진다.
