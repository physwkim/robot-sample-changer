# 런 상태 발행과 중단 복구 — 2026-09-03

`CLAUDE.md`는 **무엇이 있는지**를 적는다(PV 표, 모드, 스텝 번호). 이
문서는 2026-09-03에 넣은 것들이 **어떤 질문에 답하려고** 들어갔는지,
그리고 멈춘 런 앞에 선 사람이 실제로 무엇을 눌러야 하는지를 적는다.

코드 기준은 `0547706..0be30c1`, 15 커밋. 전부 이 날짜다.

---

## 1. 출발점 — 멈춘 팔 앞에서 알 수 없던 것

시트 점유 검사(`seat_check`, 2026-08-19)가 들어간 뒤로 런은 **정당한
이유로 멈추기 시작했다**. 스테이지가 차 있으면 스텝 7에서, 넣으려는
홀더가 차 있으면 스텝 18에서 선다. 그게 맞다. 문제는 그 다음이었다:

- **왜 멈췄는지**가 데몬 터미널에만 있었다. 장비 앞에 선 사람은 거기를
  보고 있지 않다.
- **퍽이 어디 있는지**를 화면이 몰랐다. 검사는 방금 봤는데, 그 판정이
  아무 데도 남지 않고 로그로 흘러갔다.
- **버튼이 눌리는 근거**가 런 잔해였다. `CurrentStep == 12`로 Continue를
  열면, 스텝 12에서 죽은 데몬 위에서도, autosave가 12를 복원한 IOC
  위에서도 버튼이 열린다 — 아무도 안 읽는 `Wait`에 쓰게 된다.
- **손에 퍽을 든 채 멈추면** 나갈 길이 Advanced 카드밖에 없었다.

오늘 넣은 것은 전부 이 네 줄에 대한 답이다.

---

## 2. 데몬이 발행하는 것

### 2.1 `Robot:State` / `Robot:Alive` — 버튼의 유일한 게이트

데몬은 **서비스 패스**(`service_hold`)에서만 운전자 명령을 읽는다 —
`Robot:Gripper`, jog, `Jog:Apply`, `Wait`, 두 번째 `Trigger`. 이 패스는
멈춰 서 있는 루프에서만 돌고 팔이 움직이는 동안에는 돌지 않는다.
그래서 GUI 컨트롤은 **`State`(어느 루프인지) + `Alive`(아직 그 말을
하고 있는지) 둘로만** 열린다. `Alive`가 2초(서비스 패스 20회) 멈추면
그 컨트롤을 회색 처리한다.

`State=1`(Running)일 때 `Alive`가 멈추는 것은 정상이다. 그래서 데몬은
**막히는 일 앞뒤로 Running을 찍는다**(`while_moving`) — 스텝 실행뿐
아니라 서비스 패스 안의 jog 모션과 그리퍼 명령까지. 덕분에 규칙이
하나로 줄어든다: *Running이면 조용한 게 정상, 서 있는 상태인데 조용하면
죽은 것.*

쓰는 주인은 `set_state` 하나뿐이고, 항상 현재 beat와 같이 쓴다.
(`8b899a5`)

### 2.2 `Robot:Status` — 왜 서 있는지

`State`가 숫자로 "어느 루프"를 말한다면 이건 **문장**이다.

- 평상시: `step 9: sample_holder_on_position`,
  `measuring - Wait=1 to continue`, `hold @holder 3: trigger to return`
- 멈추면: `STOP: seat check @the stage: the seat r…`,
  `not started: fingers shut on nothing`

두 가지가 설계의 전부다.

1. **멈춘 이유는 다음 트리거 대기에서도 그대로 남는다.** 이유를 쓰고
   바로 `Idle`로 덮으면 "아무 일 없음"이 되는데, 실패 직후에 그건
   유일하게 틀린 답이다. 데몬은 트리거 대기에 들어갈 때마다 자기가 들고
   있는 `idle_status`를 다시 쓴다.
2. **서비스 패스마다 다시 쓴다.** 대기 중에 IOC를 재시작하면 레코드가
   db 기본값(`no daemon`)으로 돌아온다. 바뀔 때만 쓰는 데몬이면 살아서
   서 있는 데몬 위에 "no daemon"이 그대로 남는다.

`stringin`이라 39자다. 에러 문장은 **앞에서부터** 남으므로, 어떤
검사·어떤 이동이 거부했는지가 먼저 오도록 메시지를 쓴다. 전문은 로그에
그대로 있다. 쓰는 주인은 `set_status` 하나뿐이다. (`3899bf4`)

### 2.3 `Robot:Seat:*` — 퍽이 어디 있나

시트 11개(스테이지 + 홀더 1-10)마다 longin 하나: 0=모름, 1=빔, 2=참.
**둘 다 관측이고 추측은 없다.**

- `seat_check` 게이트가 시트에 들어가기 전에 읽은 판정. 스텝이 기대한
  값과 달라 런이 멈추는 경우에도 **본 대로** 쓴다 — 그 시트가 바로
  운전자가 봐야 할 시트라서.
- 시퀀스 자신이 아는 것: 퍽을 들어올린 시트는 빔(스텝 5·16), 놓은
  시트는 참(스텝 10·21). 재개로 **건너뛴 스텝은 쓰지 않는다**. 홀더 간
  이동(모드 7)과 그립 널의 가져오기도 같은 두 지점에서 쓴다.

`Robot:Seat:Msg`는 그 판정 **뒤에 있는 읽음**이다 —
`holder 3: occupied -4.5mm 71%`. 판정이 안 나오는 경우도 여기로 나간다
(`check switched off`, `no camera IOC`, `no depth frame`,
`not in the frame`, `unreadable, 7% valid`). (`7189576`)

### 2.4 셋 다 autosave가 아니다 — 그게 요점이다

`CurrentStep`·`Loaded`·`Jog:Target`은 재개용 마커라 데몬보다 오래
살아남아야 하고, IOC가 autosave에서 복원까지 한다. 오늘 넣은 셋은
반대다:

- `State`/`Alive`: 죽은 데몬의 상태를 복원해 보여주면 버튼이 열린다.
- `Status`: 죽은 데몬의 상태를 복원해 보여주는 건 `State`가 막으려는
  바로 그 거짓말이다.
- `Seat:*`: IOC 재시작 뒤에는 아무도 안 본 게 맞다. 복원된 "참"은 그
  사이 손으로 옮긴 퍽을 못 본다.

---

## 3. GUI가 그것으로 하는 것

- **State 카드 `Daemon` 줄** — 듣고 있나(`State`+`Alive`).
- **State 카드 `Doing` 줄** — 무엇을 하고 있나(`Robot:Status`). 멈춘 런
  뒤에는 화면에서 이유를 말하는 유일한 자리다.
- **Seats 카드** — `Robot:Seat:*` 열한 개를 칩으로. 초록 = 퍽, 회색 =
  빔, 어두움 = 아직 아무도 안 봄. 레코드가 아예 없어도 같은 어두움이고,
  툴팁이 IOC 재시작을 지목한다. 칩 아래 `last check —` 줄이
  `Robot:Seat:Msg`.
- **모든 컨트롤의 게이트가 `State`+`Alive`로 통일됐다.** 런 값
  (`CurrentStep`, `Jog:Target`)은 라벨(어느 시트인지)에만 쓰고 게이트에는
  쓰지 않는다. (`src/robot_gui_rs/src/daemon.rs`)

---

## 4. 런이 시작하기 전에 거부하는 것

### 4.1 `empty_close` — 빈 손으로 시트에 들어가지 않는다

시트에 들어가는 모드의 모든 다리는 두 가지 중 하나로 시작한다: 집기(
핑거가 열려 있어야 함) 또는 나르기(퍽을 물고 있어야 함). **아무것도 안
물고 닫혀 있는 핑거**는 둘 다 아니고, 다음 스텝들은 그 상태로 시트에
들어간다. 그렇게 되는 흔한 경로 둘 — `StartStep` 재개가 스텝 0의 열기를
건너뛰는 것, GUI의 수동 Close.

**런의 첫 모션 전에 거부**하고 고치지 않는다. 핑거를 여는 건 운전자의
판단이다(무엇을 물고 있는지 데몬은 모른다). 팔을 움직이며 시작하는
런이야말로 이 검사가 막으려는 것이다. (`809ca97`)

### 4.2 `Robot:SeatCheck` — 끄는 스위치는 사람 손에

`sequencer.yaml`의 `seat_check.enabled`는 검사가 **존재하는지**를 정하고
(핸드아이 파일을 읽는다), 이 레코드는 그게 **도는지**를 정한다. 데몬이
게이트마다 한 번 읽으므로 런 도중에 꺼도 다음 시트부터 듣는다.

읽기가 실패하면(레코드 없음, 타임아웃) **켜짐**으로 답한다. autosave에
넣지 않은 것도 같은 이유다 — 꺼둔 안전 검사가 IOC 재시작으로 조용히
되살아나거나 조용히 꺼져 있으면 안 된다. 레코드는 On으로 올라온다.
(`491d76f`)

### 4.3 `resume_approach` — 재개는 그 다리의 standby부터

`StartStep`으로 다리 중간에 들어가면 첫 카테시안 스텝은 팔이 지금 어디
있든 직선으로 간다. 재개 전에 그 다리의 standby까지 **계획 이동**을 먼저
넣는다. 다리별 매핑은 `LegStandby::for_resume`:

| StartStep | 접근 |
|-----------|------|
| 2-6 | 랙 standby (집기) |
| 8-17 | 스테이지 standby (놓기 + 집기, 두 다리가 같은 standby) |
| 19-23 | 랙 standby (놓기) |
| 0·1·7·18 | 없음 — 그 스텝 자신이 standby로 가는 계획 이동이다 |

(`b75e93a`, 그리고 계획 실패 메시지가 어느 끝인지 말하도록 `ee36bb6`)

---

## 5. 중단된 런 — 어디서 멈췄나, 무엇을 누르나

`Robot:Status`가 어디서 멈췄는지 말한다. 팔의 상태는 그것으로 결정된다.
게이트 넷은 스텝 **1·7·12·18을 마친 직후**에 서므로 `CurrentStep`은 그
번호로 남는다(다음 스텝 2·8·13·19는 실행되지 않았다).

| 멈춘 곳 | 팔의 상태 | 답 |
|---------|-----------|-----|
| 스텝 1 뒤 (`seat check @holder N: empty`) | 빈 손, 랙 standby | 퍽이 있는 홀더로 `Holder`를 고쳐 다시 트리거 |
| 스텝 7 뒤 (`seat check @the stage: occupied`) | **퍽을 문 채** 스테이지 standby | ① 빈 홀더로 **Put carried puck in holder** ② 그 다음 **Return from stage**로 스테이지를 비움 |
| 스텝 12 뒤 measurement wait | 빈 손, 스테이지 standby | Continue(회수) 또는 Abort |
| 스텝 12 뒤 (`seat check @the stage: empty`) | 빈 손, 스테이지 standby | 스테이지에 퍽이 없다 — 어디로 갔는지 확인 |
| 스텝 18 뒤 (`seat check @holder N: occupied`) | **퍽을 문 채** 랙 standby | 다른 빈 홀더로 **Put carried puck in holder** |
| 보호정지(충격) | 어디든 | `CalibMode=4`(Recover) 트리거 — unlock → 프로그램 재전송 → 스탠바이 |

### 5.1 "Put carried puck in holder" (`Action::Divert`)

Sample 카드, "Return from stage" 아래. 위 `Holder` 값을 목적지로 삼아
`Holder=N`, `CalibMode=0`, `StartStep=18`, `PauseStep=0`, `Wait=0`,
`Trigger=1`을 쓴다.

스텝 18이 그 홀더 standby까지 계획해서 가고, 19-23이 놓는다. **집는
다리(0-6, 13-17)는 전부 건너뛰므로 손에 든 퍽을 놓지 않는다.** 새
홀더가 비었는지는 스텝 18의 `seat_check`가 다시 본다. 원래 홀더로
되돌리는 것도 같은 버튼이다 — 스텝 5에서 비웠으므로 그 시트는 비어
있다. (`0be30c1`)

### 5.2 순서가 반대면 안 된다

**퍽을 든 채 `StartStep=13`(Return)을 누르지 말 것.** 스텝 12 뒤의
`seat_gate`는 "스테이지에 퍽 있음"을 기대하므로 통과하고, 스텝 13-14가
퍽을 문 핑거로 스테이지 퍽 위로 내려간다. `empty_close`는 "빈 손으로 닫힘"만
막지 이건 못 막는다. 손에 든 퍽을 먼저 내려놓는다(§5.1).

### 5.3 구 Python GUI도 같은 값을 쓴다

`_return`이 아직 `StartStep=7`을 쓰고 있었다 — 회수 다리의 진입점이 7에서
13으로 바뀐 뒤로 스테이지에 놓는 다리를 다시 실행하는(시트에 내려가
빈손으로 여는) 값이었다. 13으로 고쳤고, "wait에서 Continue를 누르라"던
안내 문구도 같이 고쳤다(그 errand는 wait에 닿지 않는다). (`292477f`)

---

## 6. 시트 체크 창을 시트마다 옮긴다

창은 시트 자세를 따라가지만, **퍽 윗면이 그 자세 어디에 보이는지는
시트마다 다르다**. 창의 툴 y −6…+2 mm는 랙에서 잰 값이고, 스테이지를
옮겨 트림으로 다시 티칭한 뒤로는 스테이지 퍽 윗면이 창 맨 아랫줄에 걸려
중앙값이 50 mm 뒤 배경을 읽었다.

그래서 시트별 **창 바이어스**를 둔다(`seat_check.rack_window_bias_mm` /
`stage_window_bias_mm`, 시트의 툴 프레임 mm). 옮기는 것은 **샘플링 창
뿐**이고 판정 기준인 파지점도, 팔이 가는 자세도 그대로다 — 트림은 팔이
가는 곳이라 창을 맞추려고 건드리면 안 된다.

앉은 퍽으로 실측(스테이지):

| 바이어스 | 판정값 |
|----------|--------|
| +0 mm | +53.1 (비움) |
| +2 mm | +43.2 (비움) |
| +4 mm | −8.2 |
| +6 mm | −7.9 |
| +8 mm | −7.3 |
| +10 mm | −7.3 |

가장자리가 +3이고 플래토가 +10 너머까지 가므로 **+8 mm**(유효 픽셀 74%),
랙은 0이다. `seat_scan <cfg> stage 1 bias:0,8,0`이 라이브 프레임으로
다시 잰다.

게이트가 자기가 검사하는 시트를 라벨 문자열이 아니라 `Seat` 값으로
받게 바꾼 것이 이 바이어스의 전제다(`0beb79f`, `f23e6a8`).

---

## 7. 스테이지를 옮겼다 (2026-09-03)

세 곳이 같이 움직여야 한다 — 시트 트림, 충돌 씬, 시트 체크 창.

1. **트림** — `sample_holder_on_position_{x,y,z}_offset`. jog + Apply로
   쓴다(툴 프레임 mm). 이번 값 (−8.4246, −1.5, +4.4842) mm.
2. **충돌 씬** — `sequencer.yaml`의 `scene.objects` 40개가 전부 같은
   `position`을 쓰는 스테이지 한 덩어리. 트림 변화량을 **모델 좌표로
   돌려서** 그만큼 더한다. 위 트림은 모델 좌표로
   [−0.00431, −0.00849, +0.00162] m였다
   (`seat_scan config/sequencer.yaml stage 1 <dx,dy,dz mm>`가 환산을
   찍는다).
3. **시트 체크 창** — `stage_window_bias_mm` (§6).

트림은 한 점이 어디로 갔는지만 말하므로 **스테이지가 돌아갔다면 rpy는
트림으로 알 수 없다.** 씬과 창은 둘 다 `sequencer.yaml`이라 **데몬
재시작**이 필요하다(트림은 트리거마다 다시 읽는다). (`a0c10d3`)

랙 깊이(툴 y) 트림 둘은 0으로 되돌리고 h5·h9·h10을 다시 썼다. 깊이는
이제 어느 시트에서도 그립 널이 조향하지 않으므로, 조향하던 시절에 남은
그 두 값은 아무도 갱신해 주지 않는 값이었다. 측방·z 변화는
0.006-0.053 mm로 재티칭이 아니라 그립 널 스케일이다. (`765d50c`)

---

## 8. 그립 널의 대상이 스테이지까지 (`CalibMode=6`)

전에는 가져오기가 랙↔랙이었고, 스테이지는 "이미 앉아 있는 퍽"만 쓸 수
있었다. 즉 스테이지를 널하려면 Normal 런을 먼저 한 번 돌려야 했다.

이유는 기능이 아니라 코드였다 — 놓는 다리가 랙용으로 쓰여 있었다.
`carry_puck`에 목적지 `Seat`를 주면 스텝 번호 표가 갈린다:

| 목적지 | 놓는 스텝 |
|--------|-----------|
| 스테이지 | 7-12 |
| 랙 홀더 | 18-23 |

이제 대상은 두 시트 다 되고, `MapSource`의 소스는 여전히 랙 홀더뿐이다
— `MapSource=0`이 "제자리 퍽"에 이미 쓰이고 있어서 스테이지를 가리킬
번호가 없다. 널이 끝나면 퍽은 **대상 시트에 남는다**. 스테이지에 널했다면
거기 놓인 채이므로 되돌리는 건 Return이다. (`c99c32a`)

---

## 9. 그 밖에

`Robot:Loaded`를 내리는 주인을 회수 다리 하나로 모았다. 스텝 12 뒤
measurement wait에서 1이 서고, **wait 지점을 지나는 모든 경로에서** 0이
된다 — continue, skip, 그리고 wait 자체가 없는 회수
errand(`StartStep=13`)까지. 회수 errand가 0을 쓰는 이유는 그 1을 세운
마운트 런이 wait에서 끝났기 때문이다. (`5d1ee1b`)

---

## 10. 커밋

```
ee36bb6 motion: name which end of a planned move is invalid
b75e93a seq: plan to a resumed leg's standby in resume_approach
809ca97 seq: check empty_close before a seat-entering run starts
5d1ee1b seq: write_loaded(0) past the measurement wait, not inside it
8b899a5 seq: publish Robot:State and Robot:Alive from a single set_state
491d76f seq: add Robot:SeatCheck, read at every seat gate
0beb79f seq: give seat_gate the Seat it is checking, not a label
f23e6a8 seq: bias the seat-check window per seat, +8 mm at the stage
a0c10d3 config: move the stage scene and its seat trims to where it now stands
765d50c config: zero the rack depth trims and refresh h5, h9 and h10
c99c32a grip null: let the target seat be the stage, fetch and all
3899bf4 seq: add Robot:Status, the daemon's own account of itself
7189576 seq: add Robot:Seat:*, where the pucks are as far as anything looked
292477f gui: enter the return errand at step 13 in the Python GUI too
0be30c1 gui: add Action::Divert, place a carried puck in another holder
```

IOC db에 레코드 셋(`Robot:Status`, `Robot:Seat:*` 열둘)이 추가됐으므로
**robot_ioc 재시작이 필요하다**. 데몬은 레코드가 없으면 조용히 발행을
건너뛰므로 순서는 상관없지만, 재시작 전까지 GUI의 Doing 줄과 Seats 칩은
비어 있다.
