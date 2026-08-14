# 카메라 보정 구현 계획 — eye-in-hand D405

기존 `vision_inspection_plan.md` / `vision_camera_setup.md`를 **대체하지 않고
전제를 정정한다.** 두 문서는 외부 고정 카메라를 전제로 쓰였고, 실물은 그것이
아니다 (§0). 측정 원리·태그 운용·안착 판정 논의는 그대로 유효하므로 계속
참조한다. 바뀌는 것은 **캘리브레이션 형식과 측정 흐름**이다.

---

## 0. 전제 정정 — 문서와 실물의 불일치

`vision_inspection_plan.md:206`은 이렇게 확정하고 있다:

> **확정 사항 (Rev 3): 외부 고정 카메라 1대 + AprilTag(tag36h11)를 쓴다.**

그리고 `:217`에서 eye-in-hand(M1)를 "손목 D405를 병용할 때만 필요. 외부
카메라만 쓰면 **불필요**"로, `:222`에서 "손목에 카메라가 없으므로"라고 적었다.

**실측 결과 이 전제가 성립하지 않는다.**

| 항목 | 문서 전제 | 실측 |
|---|---|---|
| 카메라 대수 | 외부 고정 1대 | **D405 1대뿐** (`lsusb`: `8086:0b5b`, serial `315122272475`) |
| 장착 | 브레드보드 고정 | **손목 장착** (사용자 확인) |
| 캘리브레이션 | eye-to-hand → `T_base_cam` | **eye-in-hand → `T_ee_cam`** |
| 외부 카메라 | 존재 전제 | **존재하지 않음** |

문서는 8/12 작성, AprilTag 시트(`doc/apriltags/`, 바닥 100 mm 태그 + 홀더
10 mm 태그)는 8/13 작성이다. 태그 구성 자체가 이미 eye-in-hand 배치다 —
**고정 태그 + 움직이는 카메라**. 즉 실제 작업은 이미 eye-in-hand로 갔고
문서만 남아 있다.

**이 계획은 eye-in-hand를 기준으로 한다.** 외부 카메라가 나중에 추가되면
§6에 적은 대로 이 계획 위에 얹는다 (버리는 작업 없음).

### 0.1 전제가 바뀌면서 좋아지는 것

문서가 걱정하던 두 문제가 **사라진다**:

- **로봇 팔 가림** (`vision_camera_setup.md:142`, "30° 비스듬") — 카메라가
  손에 있으므로 팔이 시야를 가릴 일이 없다. 마운트 각도 타협이 불필요하다.
- **랙과 측정 홀더를 한 시야에** (`:529`) — 카메라가 따라가므로 한 대로
  둘 다 본다. 배치 제약이 없다.

그리고 결정적으로:

**기존 `Robot:Vision:DX/DY/DZ` 훅이 eye-in-hand 출력과 정확히 맞는다.**

```rust
// sequence.rs:505 — 보정은 TCP-local 프레임
self.model.apply_cartesian_offset(base, offset, /* z_global */ false, label)
```

`corrected()`는 티칭 웨이포인트의 FK 포즈에 **TCP 로컬**로 offset을 곱한다.
eye-in-hand 카메라의 측정값은 툴에 붙어 있으므로 그대로 TCP-local이다.
외부 고정 카메라였다면 base 경유 변환이 필요했고, 그 변환에
`T_base_cam` 오차가 통째로 실렸을 것이다. **훅을 고칠 필요가 없다.**

---

## 1. 현재 상태 인벤토리

### 1.1 이미 있는 것 (재사용)

| 항목 | 위치 | 상태 |
|---|---|---|
| D405 EPICS IOC | `d405-ioc.service` (epics-rs-iocs) | **가동 중.** `RS405:` prefix |
| 컬러 스트림 | `RS405:image1:` | 640×480, RGB1/UInt8, 15 fps |
| 깊이 스트림 | `RS405:depth1:` | 640×480, depth unit **0.0001 m** (D405 고유) |
| PVA 서버 | port 5085 | 이미지 전송 경로 |
| 시퀀서 비전 훅 | `sequence.rs:195-276` | 4종 8지점 배선 완료, `enabled: false` |
| 핸드셰이크 프로토콜 | `Robot:Vision:Req/Kind/Done/Valid/DX/DY/DZ/Quality/Seated/Tilt` | IOC 서빙 중 |
| 보정 적용 | `sequence.rs:496` `corrected()` → `model.rs:133` | TCP-local, IK 재해 |
| 관찰 모드 | `config.rs:196` `observe_only` | 측정·로그만, 이동 없음 |
| 리허설 대역 | `src/bin/vision_sim.rs` | 카메라 없이 프로토콜 검증 |
| AprilTag 시트 | `doc/apriltags/` | id 0 @100 mm, id 1–11 @10 mm, 시험띠 100–104 @10~30 mm |
| 구 hand-eye 코드 | `git show main:src/realsense_service/.../hand_eye_calibration_node.py` | ROS2, 574줄. **수식은 이식 가능** |

### 1.2 없는 것 (만들어야 함)

| 항목 | 왜 필요 | 난이도 |
|---|---|---|
| **카메라 내부 파라미터 공급** | 없으면 거리가 전부 틀림 | 소 (§2.1) |
| **비전 노드** | 저장소에 구현이 0. 문서와 태그 생성기뿐 | 중 |
| **`T_ee_cam`** | 미측정 | 중 (§3) |
| **URDF 카메라 링크** | `model/urdf/*.urdf`에 camera 없음 | 소 |
| **scene 런타임 갱신 경로** | `load_scene_assets`는 connect 전용 (`bringup.rs:46`) | 중 (§5) |

### 1.3 구 hand-eye 코드의 함정 (이식 전 반드시 고칠 것)

```python
# hand_eye_calibration_node.py:80-85
self.camera_matrix = np.array([...])   # D405 값 하드코딩
self.dist_coeffs   = np.zeros(5)       # "RealSense는 왜곡 보정된 이미지 제공"
```

- **하드코딩 intrinsics** — 개체마다 다르다. 실제 장치에서 읽어야 한다.
- **`invert_gripper2base`** (`:463`) — eye-to-hand용 역변환 플래그.
  **eye-in-hand에서는 `False`**. 켜면 조용히 틀린 답이 나온다.
- ArUco `DICT_6X6_250` / `marker_size 0.05` — 새 태그는 tag36h11이다.

---

## 2. 좌표계와 미지수

```
        T_base_ee        FK로 계산 (관절각 → 기지)
   base ──────────► ee ──────────► cam ──────────► tag
                    T_ee_cam        T_cam_tag
                    ▲               ▲
                    │               └── 매 프레임 측정 (solvePnP)
                    └── 미지수. 1회 캘리브레이션 (§3)
```

**보정에 실제로 필요한 것은 절대 포즈가 아니다.** 티칭된 `above` 자세에서
태그를 보고, "기대 위치 대비 얼마나 어긋났나"만 구한다:

```
d_cam   = T_cam_tag(측정) 의 위치 − T_cam_tag(기준, 티칭 시 저장) 의 위치
d_tcp   = R_ee_cam · d_cam                    ← 회전만 쓴다
→ Robot:Vision:DX/DY/DZ  (mm, TCP-local)
```

**`T_ee_cam`의 병진 성분은 1차적으로 상쇄된다** — 같은 카메라가 기준과 측정을
모두 찍기 때문이다. 회전 `R_ee_cam`만 정확하면 된다. 이것이 eye-in-hand 상대
측정이 절대 캘리브레이션 정확도에 둔감한 이유이고, `max_correction: 3.0` mm
같은 좁은 한계를 걸 수 있는 근거다.

> 이 성질은 §5(지그 로컬라이제이션)에서는 성립하지 않는다. 거기서는 절대
> `T_base_rack`이 필요하고 `T_ee_cam`의 병진 오차가 그대로 실린다. **그래서
> §4까지와 §5 이후는 요구 정확도가 다르다.**

---

## 3. Phase 0 — 측정 기반 (선행 필수)

이 단계가 끝나기 전에는 그 어떤 보정도 의미가 없다.

### 3.1 내부 파라미터 확보

**현재 얻을 수 없다.** IOC가 intrinsics를 PV로 내보내지 않는다:

```
RS405:cam1:RSFx_RBV   → Channel connect timed out: not found
RS405:cam1:RSDepthUnits_RBV → 0.0001      (이건 있음)
```

드라이버는 값을 갖고 있다 (`epics-rs-iocs/drivers/d435i/src/types.rs`에
intrinsics 타입 존재). 세 선택지:

| 안 | 방법 | 평가 |
|---|---|---|
| **A** | IOC에 `RSFx/RSFy/RSPpx/RSPpy/RSCoeff` PV 추가 | **권장.** 드라이버가 이미 보유. 변경 국소적. 장치 교체 시 자동 추종 |
| B | 비전 노드가 pyrealsense2로 직접 조회 | 카메라를 IOC가 배타 점유 중이라 충돌 |
| C | 체커보드로 독립 캘리브레이션 | 공장값보다 정확할 수 있으나 공수 큼. A 검증용으로 유용 |

**A로 진행하고, C를 1회 교차검증으로 쓴다.** 두 값이 크게 다르면 A를 믿지
않는다.

⚠ `dist_coeffs = zeros(5)`를 그대로 두지 말 것. D405가 왜곡 보정된 프레임을
주는지 실제 계수로 확인하고, 0이 아니면 반영한다.

### 3.2 태그 검출 파이프라인

```
RS405:image1:  ─(CA 또는 PVA)─►  비전 노드  ─► tag36h11 검출 ─► solvePnP
                                     │                              │
                                     └──────── Robot:Vision:* ◄─────┘
```

구현 언어는 Python (기존 `robot_gui`가 이미 pyepics 기반, `robot_gui` conda
env 재사용 가능). `pupil-apriltags` 또는 `cv2.aruco.DICT_APRILTAG_36h11`.

### 3.3 태그 크기 결정 — 시험띠로 실측

10 mm 태그가 실제로 검출되는지는 **계산이 아니라 측정으로** 정한다.
`doc/apriltags/`의 시험띠 (id 100–104, 10/15/20/25/30 mm)가 이 용도다.

개략 계산으로 예상되는 문제 (실측으로 확정할 것):

IOC의 실측 intrinsics(fx 393.284, cx 321.745, fy 392.673, cy 246.323)로
계산한 값이다. 사양서의 87°가 아니라 **78.3° × 62.8°** 이다:

```
HFOV = atan(cx/fx) + atan((640-cx)/fx) = 39.28° + 38.98° = 78.3°
VFOV = atan(cy/fy) + atan((480-cy)/fy) = 32.09° + 30.75° = 62.8°

640×480, 작업거리 150 mm
→ 시야 폭 = 150·(tan39.28° + tan38.98°) ≈ 244 mm → 0.38 mm/px
→ 10 mm 태그 ≈ 26 px
```

tag36h11은 테두리 포함 10×10 셀이라 26 px면 셀당 2.6 px — **경계선상**이다.
대응책 두 가지:

- **IOC 해상도를 1280×720으로 올린다** (st.d405.cmd:47이 지원 명시)
- 또는 더 큰 태그를 쓴다 (시험띠가 답을 준다)

실제로는 10 mm 태그를 붙일 자리가 없어 **100 mm 한 장을 바닥에 고정**했고,
작업거리 ≈ 290 mm에서 0.74 mm/px로 관측된다.

### Phase 0 완료 기준 (검증 가능)

- [ ] `caget RS405:cam1:RSFx_RBV` 등이 값을 반환한다
- [ ] 체커보드 독립 캘리브레이션 결과와 fx, fy가 **3% 이내** 일치
- [ ] 고정 태그를 고정 거리에서 300 프레임 관측 시 위치 표준편차
      **σ < 0.1 mm**, 검출 실패율 **0%**
- [ ] 시험띠로 "작업거리에서 안정 검출되는 최소 태그 크기"를 표로 확정

---

## 4. Phase 1 — hand-eye 캘리브레이션 (`T_ee_cam`), 평생 1회

### 4.1 절차 — 데몬의 `CalibMode = 3`

수집은 별도 도구가 아니라 **`robot-sequencer` 데몬 자체**가 한다. 로봇은
RTDE 클라이언트를 하나만 받으므로, 별도 바이너리로 수집하려면 프로덕션
데몬을 내려야 하고 — 카메라를 보정하려고 그 카메라에 의존하는 데몬을
죽이는 것은 순서가 거꾸로다.

```bash
caput Robot:CalibMode 3        # Hand-Eye Calib
caput Robot:Trigger 1          # 1회차: aiming hold (jog 가능, 1 Hz 검출 로그)
# JogX/Y/Z로 태그를 화면 중앙에 맞춘다
caput Robot:Trigger 1          # 2회차: 수집 시작
```

트리거가 2회인 것은 다른 캘리브레이션 모드와 같은 이유다. idle 트리거
대기(`wait_for_trigger(false)`)는 jog를 서비스하지 않는다 — 그 자리는 티칭된
standby 자세이고 거기서 jog를 허용하면 시퀀스의 시작점이 움직인다. 조준은
모드가 정해진 뒤의 hold에서 한다. hold 중에는 검출기가 이미 떠 있으므로
1초마다 태그 위치를 로그로 되돌려준다 — "화면에 보인다"와 "검출기가 풀 수
있다"는 다른 주장이고, 수집을 성립시키는 것은 후자다.

바닥 고정 태그(**id 0, 100 mm**)를 브레드보드에 볼트 고정한다. 데몬은
2회차 트리거 시점의 자세를 home으로 잡고, 툴 자체 축(x/y/z) 둘레로 회전한
12자세를 돌며 각 자세에서:

1. 관절각 기록 → FK로 `T_base_ee`
2. 태그 관측 → solvePnP → `T_cam_tag`

자세마다 home으로 복귀하므로 검출 실패가 누적되지 않고, 모든 이동은
시퀀스와 같은 충돌 씬을 통과하는 계획 이동이다. 결과는
`handeye.out_dir/samples.yaml`.

**공전(orbit)이 아니라 제자리 회전인 이유.** `calibrateHandEye`가 원하는
것은 회전 다양성이지 병진이 아니고, 셀이 좁다. 290 mm에서 100 mm 태그는
78°×63° 시야 안에서 약 ±10°만 차지하므로 툴은 태그가 프레임을 벗어나기
전까지 수십 도를 돌 수 있다 — 그리고 툴 회전은 카메라를 마운트 오프셋
만큼(수십 mm) 옮길 뿐이어서, 같은 각도의 공전이 요구하는 ~100 mm 이동과
비교가 안 된다.

수집 자세 수(12) × 3축은 고정이고, 각도만 `handeye.angle_deg`로 조절한다.
축이 3개인 것은 필수다 — 회전축을 공유하는 두 상대운동은 AX = XB에서
축퇴(degenerate)이고, 이것이 "그럴듯한 오답"이 나오는 전형적 경로다.

```python
# eye-in-hand: 반환값이 곧 T_ee_cam
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    R_gripper2base, t_gripper2base,     # invert 하지 않는다
    R_target2cam,   t_target2cam,
    method=cv2.CALIB_HAND_EYE_TSAI)
```

⚠ **`invert_gripper2base = False`.** 구 코드의 이 플래그는 eye-to-hand용이다
(`hand_eye_calibration_node.py:463`). eye-in-hand에서 켜면 결과가 조용히
틀린다 — 수렴은 하되 값이 틀리므로 residual만으로는 못 잡는다.

**회전 다양성이 정확도를 지배한다.** 위치만 옮기고 자세를 안 바꾸면
`calibrateHandEye`는 병조건이 된다. 태그를 화면 중앙에 맞춰 놓으면 이 셀
에서도 ±29°/±21°까지는 실측으로 확보된다.

> 여기서 §0.1의 level-tool 제약(`scene.rs:49`, ±3°)이 걸린다. 캘리브레이션
> 자세는 정확히 그 제약이 금지하는 툴 회전이므로, 수집 동안만 제약을 끈다.
> 끄고 켜는 것은 `Motion::suspend_level_tool`이 돌려주는 가드가 하며,
> 복원은 실패한 이동에서 빠져나가는 `?` 경로까지 포함해 Drop이 책임진다 —
> 데몬은 수집 뒤에도 계속 돌기 때문에, 한 번의 복원 누락이 다음 프로덕션
> 시퀀스를 무제약으로 계획하게 만든다.

### 4.2 검증 — residual만 믿지 않는다

`calibrateHandEye`는 틀린 입력에도 답을 낸다. **독립 검증**이 필수다.
`tools/handeye/solve.py`가 이 둘을 자동으로 낸다:

- 여러 방법(Tsai / Park / Horaud / Daniilidis) 결과가 서로 **2 mm, 1° 이내**
  로 일치하는지 — 흩어지면 데이터가 나쁜 것이다
- leave-one-out: 한 자세를 빼고 적합한 뒤, 뺀 자세로 `T_base_tag`를 예측해
  나머지의 평균과 비교. 적합에 쓴 자세의 residual과 달리 이것은 실제
  측정이 지게 될 오차다

4점 태그의 reprojection error는 **정확도 지표가 아니다** — 미지수와 관측
수가 거의 같아 언제나 작게 나온다.

### Phase 1 완료 기준

- [ ] 검증셋 5자세에서 예측-실측 위치 오차 **< 2 mm**, 회전 오차 **< 1°**
- [ ] 4개 방법의 해가 서로 2 mm / 1° 이내
- [ ] `T_ee_cam`을 YAML로 저장 + 측정 일자·자세 수·residual 기록
- [ ] URDF에 카메라 링크 추가 (측정된 `T_ee_cam`으로) — scene/충돌 검사에서
      카메라 몸체가 고려되도록

---

## 5. Phase 2 — 관찰 모드 (이동 없음)

**코드 변경 없이** 이미 가능하다. `config.rs:196`의 `observe_only`가 정확히
이것이다 — 4개 훅에서 측정하고 로그만 남기며, 이동도 정지도 하지 않는다.

```yaml
vision:
  enabled: true
  observe_only: true      # 측정만
```

기존 시퀀스를 N회 돌리며 8개 측정 지점의 분포를 모은다.

### Phase 2 완료 기준

- [ ] 20 사이클 무중단, 검출 실패 0
- [ ] 8지점 각각에서 |d| 분포의 σ **< 0.2 mm**
- [ ] 반복 측정 편차가 `min_correction: 0.05` mm보다 크고
      `max_correction: 3.0` mm보다 충분히 작은지 확인 →
      **이 데이터로 두 임계값을 확정한다** (지금 값은 추정치)
- [ ] 이상치가 나온 지점·조명·자세를 기록

---

## 6. Phase 3 — 미세 보정 활성화

`observe_only: false`. 훅을 **한 번에 하나씩** 켠다:

```
pick_align → grip_offset → place_align → seating_check
```

각 단계에서 실패 시 즉시 이전 상태로 되돌린다. 실패 정의: 시퀀스 정지,
파지 실패, 안착 불량 중 하나라도 발생.

### Phase 3 완료 기준

- [ ] 훅 4개 전부 활성 상태로 50 사이클 무중단
- [ ] 보정 없이 돌린 대조군 대비 안착 실패율 감소를 수치로 제시
- [ ] `max_correction` 초과로 정지한 사례가 **전부 실제 이상**이었는지 확인
      (오검출로 인한 헛정지가 있으면 임계값 재조정)

---

## 7. Phase 4 — 지그 로컬라이제이션 → scene 갱신

여기서부터 **절대 정확도가 필요하다** (§2 각주). `T_ee_cam`의 병진 오차가
그대로 실리므로 Phase 1의 품질이 직접 드러난다.

### 7.1 구조

랙/측정 홀더의 10 mm 태그(id 1–11)를 관측해 `T_base_rack`을 구하고,
`SceneAsset.pose`에 주입한다.

**현재 갱신 경로가 없다.** `load_scene_assets`(`scene.rs:97`)는
`bringup.rs:46`의 connect 시점 전용이고, `ParryCollisionEnv`는
`scene_with_assets`(`scene.rs:126`)에서 매번 새로 만들어진다. 다행히
`SceneAsset.pose`가 단일 필드라 주입점 추가는 국소적이다.

### 7.2 안전장치 (문서 §5.3의 승인 흐름 유지)

```
Robot:RelocalizeReq=1 → 측정 3회 반복 → σ 검증
   → Robot:RelocalizeDelta 보고 (저장값 대비 이동량)
   → 한계 초과면 자동 거부
   → 운영자가 Robot:RelocalizeCommit=1 로 승인해야 반영
```

**자동 커밋 금지.** 태그 오검출로 잘못된 지그 위치를 커밋하면 다음 동작이
충돌이다.

### Phase 4 완료 기준

- [ ] 랙을 의도적으로 5 mm 옮긴 뒤 측정한 `RelocalizeDelta`가 실측 이동량과
      **1 mm 이내** 일치
- [ ] 3회 반복 측정 σ < 0.5 mm
- [ ] 오검출을 인위적으로 주입(태그 가림·오조명)했을 때 자동 거부가 동작
- [ ] scene 갱신 후 플래닝이 정상 동작 (기존 4개 티칭 포즈가 충돌로 읽히지
      않을 것 — **VHACD 근사가 이미 `holder1_on_position`을 충돌로 읽는
      기존 한계가 있으므로**, 갱신 전후 비교로 새 회귀만 판정한다)

---

## 8. Phase 5 — frame-relative 웨이포인트 (최대 위험, 선택)

`vision_inspection_plan.md` §5 그대로. **이 계획의 필수 경로가 아니다** —
Phase 3까지로 "카메라 보정"은 완성된다. Phase 5는 *재설치 대응*이 목적이다.

핵심 위험은 변하지 않았다: **IK 분지 선택.** UR3e는 한 포즈에 해가 최대 8개고
카메라는 어느 분지인지 말해주지 못한다. 기존 티칭 관절값을 seed로 유지해야
한다. `waypoint_mode: joint | frame_relative` fallback 필수.

---

## 9. 순서 요약과 의존성

```
Phase 0 (intrinsics + 검출 + 태그크기)   ← 선행 필수, 이것 없이는 전부 무의미
   │
Phase 1 (T_ee_cam, 1회)                  ← 회전 다양성이 품질을 지배
   │
   ├─► Phase 2 (관찰) ─► Phase 3 (보정 활성)   ★ "카메라 보정"의 완성점
   │
   └─► Phase 4 (scene 갱신) ─► Phase 5 (frame-relative, 선택)
```

Phase 3까지가 사용자가 요청한 "카메라로 보정"이다. Phase 4/5는 재설치
자동 대응이라는 **다른 목적**이며, 요구 정확도도 다르다 (§2 각주).

---

## 10. 즉시 착수 가능한 것 / 막혀 있는 것

| | 항목 | 상태 |
|---|---|---|
| ✅ | 바닥 100 mm 태그 볼트 고정 | 완료 (id 0) |
| ✅ | intrinsics PV 서빙 + 실측 (§3.1) | 완료 |
| ✅ | 검출 파이프라인 (`tools/handeye/detector.py`) | 완료, 재현성 검증됨 |
| ✅ | hand-eye 수집 (`CalibMode = 3`) + 해석 (`solve.py`) | 구현 완료, **실행 미수행** |
| ✅ | 태그 시험띠 인쇄·부착 → 최소 크기 실측 | 지금 가능 (100 mm로 우회 중) |
| ✅ | IOC 해상도 1280×720 전환 시험 | 지금 가능 |
| ⛔ | 비전 노드 구현 | `T_ee_cam`(Phase 1 실행) 선행 |

---

## 11. 미확정 — 실측해야 정해지는 값

문서가 추정치로 적고 있으나 **아직 측정되지 않은** 것들.

- 작업거리에서 안정 검출되는 최소 태그 크기 (10 mm로 되는가) — 시험띠 미실행,
  현재는 100 mm 한 장으로 우회
- `min_correction` / `max_correction`의 근거 있는 값 (현재 0.05 / 3.0 mm는 추정)
- D405 최소 작업거리(사양 ~70 mm)와 티칭 `above` 자세의 실제 거리 관계
- 640×480 15 fps로 충분한지, 아니면 해상도·프레임률 상향이 필요한지

확정된 것(더는 추정이 아님):

- intrinsics: fx 393.284 / fy 392.673 / cx 321.745 / cy 246.323 → HFOV 78.3°
  (사양값 87°는 **틀렸다**)
- 왜곡 모델 규약: IOC가 보고하는 `BrownConradyInverse`는 **OpenCV 정방향
  Brown-Conrady와 같다**. 이름으로 추정하지 않고 `librealsense2.so`에 직접
  물었다 — 같은 계수로 `rs2_project_point_to_pixel`
  (`RS2_DISTORTION_INVERSE_BROWN_CONRADY`)과 `cv2.projectPoints`가 프레임
  모서리에서 **0.022 px** 이내로 일치한다(잔차는 librealsense의 접선항 적용
  순서 차이). "Inverse"는 OpenCV 기준이 아니라 `MODIFIED_BROWN_CONRADY`의
  역투영 기준 이름이다. 따라서 계수를 그대로 solvePnP에 넣는 것이 맞고,
  버리면 모서리에서 5.8 px = 290 mm에서 4.2 mm가 틀어진다.
  재현: `tools/handeye/check_distortion_model.py`
- 검출 재현성: 정지 태그 20회 연속 검출에서 σ = (0.015, 0.008, 0.056) mm,
  20/20 서로 다른 프레임. 단 이는 아래 두 버그를 고친 뒤의 값이다 —
  pyepics가 `use_monitor=False` 없이는 캐시를 돌려주는 문제, 그리고
  cv2 기본값 `CORNER_REFINE_NONE`이 코너를 정수 픽셀로 양자화하는 문제.
  둘 다 "완벽한 재현성"으로 위장하며, 그대로 뒀다면 `T_ee_cam`에 0.74 mm급
  오차가 조용히 실렸을 것이다.
