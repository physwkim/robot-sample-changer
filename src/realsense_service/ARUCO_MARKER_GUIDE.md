# ArUco 마커 생성 및 사용 가이드

Hand-Eye Calibration에 사용할 ArUco 마커를 생성하고 인쇄하는 방법을 설명합니다.

## ArUco 마커 생성

### 1. 단일 마커 생성

```bash
cd ~/ws/realsense_service/examples

# 기본 설정 (ID: 0, 크기: 50mm)
python3 generate_aruco_marker.py

# 커스텀 설정
python3 generate_aruco_marker.py \
  --id 0 \
  --size 50 \
  --dict DICT_6X6_250 \
  --output my_marker.pdf
```

**파라미터 설명:**
- `--id`: 마커 ID (0-249)
- `--size`: 마커 크기 (mm 단위, 검은색 영역만)
- `--dict`: ArUco dictionary 타입
- `--output`: 출력 PDF 파일명

### 2. 여러 마커 한 번에 생성 (2x2 그리드)

```bash
python3 generate_aruco_marker.py \
  --multiple \
  --id 0 \
  --count 4 \
  --size 50 \
  --output markers_sheet.pdf
```

**권장 설정:**
- ID 0부터 시작
- 4개 마커 (ID: 0, 1, 2, 3)
- 크기: 50mm (D405 카메라에 최적)

## 마커 크기 선택 가이드

### RealSense D405 사용 시

| 마커 크기 | 캘리브레이션 거리 | 권장 용도 |
|----------|----------------|----------|
| 30mm | 10-30cm | 매우 가까운 거리 |
| **50mm** | **20-50cm** | **일반적인 캘리브레이션 (권장)** |
| 100mm | 40-100cm | 넓은 작업 공간 |

### 크기 결정 요소:

1. **카메라-마커 거리**
   - 마커가 이미지의 10-50%를 차지해야 함
   - 너무 가까우면 마커가 잘림
   - 너무 멀면 감지 정확도 저하

2. **작업 공간**
   - 로봇 작업 거리와 비슷한 범위에서 캘리브레이션
   - 예: 30cm 거리 작업 → 50mm 마커 사용

## 마커 인쇄 방법

### 1. PDF 열기
```bash
# PDF 뷰어로 열기
evince aruco_marker_id0_50mm.pdf
# 또는
xdg-open aruco_marker_id0_50mm.pdf
```

### 2. 인쇄 설정 (중요!)

**필수 설정:**
- ✅ **페이지 배율: 실제 크기 (100%)**
- ✅ **용지: A4 흰색 용지**
- ✅ **인쇄 품질: 고품질**
- ❌ 페이지에 맞춤 (비율 변경됨!)
- ❌ 자동 회전/크기 조정 (비활성화)

**인쇄 대화상자 예시:**
```
페이지 크기 조정: 없음
배율: 100%
자동 회전: 꺼짐
```

### 3. 인쇄 후 검증

**반드시 자로 측정하세요!**

```bash
# 50mm 마커의 경우
검은색 정사각형 영역 = 정확히 50mm ± 0.5mm
```

측정이 정확하지 않으면:
- 프린터 설정에서 "실제 크기" 확인
- 다시 인쇄
- 캘리브레이션 정확도에 직접 영향!

### 4. 마커 준비

1. **자르기**
   - 가위로 흰색 테두리 포함하여 자르기
   - 깔끔하게 직선으로 자르기

2. **부착**
   - 평평하고 단단한 표면에 부착
   - 플라스틱 판, 나무판, 또는 두꺼운 종이
   - 양면테이프 또는 풀 사용
   - **구겨지거나 휘어지지 않도록 주의!**

3. **보관**
   - 평평한 곳에 보관
   - 손상 방지

## 캘리브레이션에서 사용

### Launch 파일에 파라미터 설정

```bash
ros2 launch realsense_service hand_eye_calibration.launch.py \
  marker_type:=aruco \
  marker_size:=0.05 \
  aruco_dict:=DICT_6X6_250 \
  marker_id:=0
```

**중요:**
- `marker_size`는 **meter 단위**로 입력!
  - 50mm → 0.05
  - 30mm → 0.03
  - 100mm → 0.10

### Python 코드에서 사용

```python
# hand_eye_calibration_node.py 파라미터
self.declare_parameter('marker_type', 'aruco')
self.declare_parameter('marker_size', 0.05)  # 50mm = 0.05m
self.declare_parameter('aruco_dict', 'DICT_6X6_250')
self.declare_parameter('marker_id', 0)
```

## 지원하는 ArUco Dictionary

| Dictionary | 마커 개수 | 마커 크기 | 권장 용도 |
|-----------|----------|----------|----------|
| DICT_4X4_50 | 50 | 4x4 bits | 매우 가까운 거리 |
| DICT_5X5_100 | 100 | 5x5 bits | 가까운 거리 |
| **DICT_6X6_250** | **250** | **6x6 bits** | **일반적인 사용 (권장)** |
| DICT_7X7_1000 | 1000 | 7x7 bits | 많은 마커 필요 시 |

**DICT_6X6_250 권장 이유:**
- 충분한 마커 개수 (250개)
- 적절한 복잡도 (감지율 높음)
- 중거리 감지에 최적

## 캘리브레이션 팁

### 1. 마커 배치

```
카메라 ----[20-50cm]---- 마커 (평평하게 부착)
                          ↓
                        단단한 표면
```

- 평평하고 흔들리지 않는 곳에 고정
- 조명이 균일한 곳
- 반사광이나 그림자 최소화

### 2. 샘플 수집 시

- 다양한 각도에서 10-20개 샘플 수집
- 마커 전체가 항상 이미지 안에 보여야 함
- 마커가 흐리거나 왜곡되면 거리 조정

### 3. 문제 해결

**마커가 감지되지 않을 때:**
- [ ] 마커 크기가 적절한가? (이미지의 10-50%)
- [ ] 마커가 평평한가? (구겨지거나 휘어지지 않음)
- [ ] 조명이 충분한가? (너무 어둡거나 밝지 않음)
- [ ] Dictionary 타입이 일치하는가?
- [ ] Marker ID가 올바른가?

**감지는 되지만 정확도가 낮을 때:**
- [ ] 마커를 실제 크기로 인쇄했는가? (자로 측정)
- [ ] 마커가 손상되지 않았는가?
- [ ] 카메라 렌즈가 깨끗한가?

## 예제 파일

이미 생성된 마커 파일:

```bash
cd ~/ws/realsense_service/examples

# 단일 마커 (ID: 0, 50mm)
aruco_marker_id0_50mm.pdf

# 여러 마커 (ID: 0-3, 50mm)
aruco_markers_sheet.pdf
```

## 참고 자료

- [OpenCV ArUco Documentation](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [ArUco Marker Dictionary](https://docs.opencv.org/4.x/d9/d6a/group__aruco.html)
- `HAND_EYE_CALIBRATION_GUIDE.md` - 전체 캘리브레이션 절차
