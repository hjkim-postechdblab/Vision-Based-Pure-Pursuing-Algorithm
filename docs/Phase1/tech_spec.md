# Phase 1 Tech Spec: UniSannio Vision-Based Path Following Algorithm

> **목적:** IFAC2020 우승 알고리즘(UniSannio)의 핵심 로직을 **재현 가능한 수준으로** 명세하고,
> 2026 대회 모델의 Flight Control System 내부에 이식하기 위한 **구현 명세서**.
>
> **대상 독자:** MATLAB/Simulink 초보자 (Phase 0 학습 완료 직후 수준)
>
> **문서 버전:** v2.0 (v1.0 대비 리뷰 반영 개정)

---

## 0. Scope 및 기본 원칙

### 0.1 Scope

**포함 (In-Scope):**
- UniSannio 2020 알고리즘의 충실한 baseline 재현
- 2026 모델 Flight Control System 내부의 IPS / PP 블록 구현
- Simple 트랙 + 기본 날씨에서의 동작 검증

**제외 (Out-of-Scope):**
- Adaptive thresholding (Phase 2)
- 교차 트랙 분기 판별 로직 (Phase 2)
- Yaw 안정화 (Phase 2)
- 학습 기반 개선 (Phase 3+)
- 해상도 정규화 (Phase 2에서 검토)

### 0.2 참조 자료 우선순위

논문-코드-대회규격 간 불일치 발생 시 아래 순서를 따른다:

| 우선순위 | 자료 | 이유 |
|---------|------|------|
| 1순위 | UniSannio 코드 (`variables.m`, `.slx`) | 실제 동작하는 구현체 |
| 2순위 | 논문 (Terlizzi et al., ICUAS 2021) | 설계 의도 설명 |
| 3순위 | 대회 규격 (2026 Rules and Guidelines) | 인터페이스 제약 |

불일치를 발견한 경우, 부록 D의 Deviation Log에 기록한다.

### 0.3 구현자 준수 규칙

아래 항목은 Phase 1 baseline 구현 중 **임의 변경 금지**이다:

- VTP 산출 방식 (mean 기반, Section 3.4.4에서 확정)
- State 전이 조건 (Section 2에서 확정)
- 파라미터 값 (Section 8의 표에서 확정, 2026 환경 검증 후에만 변경)
- IPS/PP 블록 간 인터페이스 (Section 1.3에서 확정)
- MATLAB Function 블록 내 코드 생성 불가 문법 사용 금지

---

## 1. 시스템 아키텍처

### 1.1 전체 구조

```
┌──────────────────────────────────────────────────────────┐
│                  Flight Control System                    │
│                                                          │
│  ┌───────────────────┐    ┌────────────────────────┐     │
│  │ Image Processing   │───▶│    Path Planning        │     │
│  │   System (IPS)     │    │       (PP)              │     │
│  │   @ 5 Hz           │    │   @ 200 Hz              │     │
│  └───────────────────┘    └────────────────────────┘     │
│        ▲                            │                     │
│        │ IMG (RGB frame)            │ xw, yw, zw          │
│        │                            ▼                     │
│  ──────┴────────            ────────┴──────────           │
│  카메라 센서 입력            저수준 제어기로 출력            │
│  (MathWorks 제공)           (MathWorks 제공)              │
└──────────────────────────────────────────────────────────┘
```

**참가자가 구현하는 영역:** IPS 블록 내부 + PP 블록 내부만 해당.
카메라 센서, 드론 동역학, 저수준 제어기는 MathWorks가 제공하며 수정 불가.

### 1.2 좌표계 정의

이 문서에서 사용하는 3가지 좌표계를 명확히 구분한다:

**a) 이미지 프레임 좌표계 (Image Frame)**
```
    열(col) m = 1 ──────────────▶ m = W (160)
    │
행  │   ┌─────────────────────┐
(row)│   │                     │
n=1 │   │    (n, m)            │
    │   │                     │
    │   │         ● CoM       │
    │   │      (60, 80)       │
    ▼   │                     │
n=H │   └─────────────────────┘
(120)
```
- 행(row, n): 위에서 아래로 증가 (1 ~ 120)
- 열(col, m): 왼쪽에서 오른쪽으로 증가 (1 ~ 160)
- 원점: 좌상단 (1, 1)
- CoM(Center of Mass): (COG_X, COG_Y) = (60, 80)

**b) 월드 좌표계 (World Frame, NED)**
- x: 북쪽(North) 방향, 양수
- y: 동쪽(East) 방향, 양수
- z: 아래쪽(Down) 방향, 양수 → **고도 -1m은 지면 위 1m**

**c) 이미지↔월드 변환**
- PP에서 `x_{k+1} = x_k + α × ex`를 적용할 때,
  이미지의 행 방향 오차(ex)가 월드의 어떤 축에 대응하는지는
  **카메라 장착 방향에 의존**한다. 이는 2026 모델 설치 후 확인 필요.
- UniSannio 원본에서는 하방 카메라가 NED x-y와 정렬되어 있다고 가정.

### 1.3 블록 인터페이스 명세

#### IPS 블록

| 방향 | 신호명 | 데이터 타입 | 크기 | 샘플 타임 | 설명 |
|------|--------|-----------|------|----------|------|
| 입력 | IMG | uint8 | H×W×3 | T_IPS (0.2s) | RGB 카메라 프레임 |
| 출력 | ex | double | 1×1 | T_IPS | 행 방향 오차 [pixel] |
| 출력 | ey | double | 1×1 | T_IPS | 열 방향 오차 [pixel] |
| 출력 | Flag_VTP | boolean | 1×1 | T_IPS | 트랙 감지 여부 |
| 출력 | Flag_marker | boolean | 1×1 | T_IPS | 마커 감지 여부 |

내부 상태 (블록 내 persistent 변수):

| 변수명 | 타입 | 초기값 | 설명 |
|--------|------|--------|------|
| theta_prev | double | NaN | 이전 VTP 방향각 [rad]. NaN은 "초기화 전" 표시 |
| ex_prev | double | 0 | 이전 프레임의 ex (미분 계산용) |
| ey_prev | double | 0 | 이전 프레임의 ey (미분 계산용) |

#### PP 블록

| 방향 | 신호명 | 데이터 타입 | 크기 | 샘플 타임 | 설명 |
|------|--------|-----------|------|----------|------|
| 입력 | ex | double | 1×1 | T_IPS에서 hold | 행 방향 오차 [pixel] |
| 입력 | ey | double | 1×1 | T_IPS에서 hold | 열 방향 오차 [pixel] |
| 입력 | Flag_VTP | boolean | 1×1 | T_IPS에서 hold | 트랙 감지 여부 |
| 입력 | Flag_marker | boolean | 1×1 | T_IPS에서 hold | 마커 감지 여부 |
| 입력 | current_z | double | 1×1 | T_PP (0.005s) | 현재 고도 [m, NED] |
| 출력 | xw | double | 1×1 | T_PP | 위치 명령 x [m] |
| 출력 | yw | double | 1×1 | T_PP | 위치 명령 y [m] |
| 출력 | zw | double | 1×1 | T_PP | 위치 명령 z [m, NED] |

내부 상태 (블록 내 persistent 변수):

| 변수명 | 타입 | 초기값 | 설명 |
|--------|------|--------|------|
| x_prev | double | init.posNED(1) | 이전 x 위치 명령 [m] |
| y_prev | double | init.posNED(2) | 이전 y 위치 명령 [m] |
| mission_state | enum | TAKEOFF | 현재 미션 상태 |
| hover_counter | uint32 | 0 | 호버링 확인 카운터 |
| center_counter | uint32 | 0 | 마커 중심 정렬 확인 카운터 |
| deriv_hold_counter | uint32 | 0 | 미분항 유지 카운터 |
| track_lost_counter | uint32 | 0 | 트랙 유실 카운터 |
| ex_prev_pp | double | 0 | 이전 IPS 갱신 시점의 ex |
| ey_prev_pp | double | 0 | 이전 IPS 갱신 시점의 ey |

### 1.4 두 개의 서로 다른 동작 주기

| 모듈 | 주기 | 주파수 | 설명 |
|------|------|--------|------|
| IPS | T_IPS = 0.2 s | 5 Hz | 카메라 프레임 수신 및 처리 |
| PP | T_PP = 0.005 s | 200 Hz | 위치 명령 갱신 |

PP가 IPS보다 **40배 빠르게** 동작한다. 새로운 비전 데이터가 없는 사이에도
PP는 마지막으로 받은 오차값(zero-order hold)을 기반으로 계속 위치를 갱신한다.
이 cascade 구조가 비전 처리 지연에도 드론이 부드럽게 움직이는 핵심이다.

**코드 근거:**
```matlab
% startVars.m
Ts = 0.005;       % PP 주기: 5ms (200Hz)
VTs = 40 * Ts;    % IPS 주기: 40 × 5ms = 200ms (5Hz)
```

---

## 2. State Machine (상태 기계)

### 2.1 상태 정의

| 상태 | 이름 | 설명 |
|------|------|------|
| S1 | TAKEOFF | 이륙 후 목표 고도까지 상승, 호버링 안정화 대기 |
| S2 | FOLLOWING | 트랙 추종 (IPS의 VTP 기반) |
| S3 | END_MARKER | 착륙 마커 감지됨, 마커 중심 정렬 중 |
| S4 | LANDING | 하강 착륙 |

### 2.2 상태 전이 조건

```
  ┌────┐                          ┌────┐
  │ S1 │──── hovering_ok ────────▶│ S2 │
  │TAKE│     ∧ Flag_VTP           │FOLL│
  │OFF │                          │WING│
  └────┘                          └────┘
                                    │
                         ¬Flag_VTP ∧ Flag_marker
                                    │
                                    ▼
                                  ┌────┐
                                  │ S3 │
                                  │END │
                                  │MARK│
                                  └────┘
                                    │
                               centered_ok
                                    │
                                    ▼
                                  ┌────┐
                                  │ S4 │
                                  │LAND│
                                  └────┘
```

각 전이 조건의 **엄밀한 정의:**

**S1 → S2: `hovering_ok ∧ Flag_VTP`**
```
hovering_ok = (Z_HIGH <= current_z <= Z_LOW)
              ∧ (hover_counter >= HOVER_CONFIRM_FRAMES)

HOVER_CONFIRM_FRAMES = T_IPS / T_PP = 40
(= 호버링 고도 범위 내에서 최소 1 IPS 주기(0.2s) 동안 유지)
```

**S2 → S3: `¬Flag_VTP ∧ Flag_marker`**

이전 v1.0에서는 `Flag_VTP ∧ Flag_marker`로 기술했으나, 이는
"Flag_VTP와 Flag_marker는 상호배타적"이라는 설계 원칙과 **충돌**한다.

실제 동작: IPS에서 Arc Mask로 VTP를 찾지 못하고(`Flag_VTP = false`),
동시에 원형 침식으로 마커가 감지될 때(`Flag_marker = true`) 전이.
즉, **트랙이 끝나서 VTP가 사라지고 마커만 남는 순간**이 전이 시점이다.

**S3 → S4: `centered_ok`**
```
centered_ok = (sqrt(ex^2 + ey^2) <= MAX_ERROR_LANDING)
              ∧ (center_counter >= CENTER_CONFIRM_FRAMES)

MAX_ERROR_LANDING = 5          [pixel]
CENTER_CONFIRM_FRAMES = DELAY_LANDING / T_PP = 4 / 0.005 = 800
(= 마커 중심 오차가 5 pixel 이하인 상태를 4초간 유지)
```

이전 v1.0의 `ex==0 && ey==0` 조건은 이산 이미지에서 현실적으로 불가능하므로
**tolerance 기반으로 수정**한다. `MAX_ERROR_LANDING=5`는 원본 코드에 존재하는 값이다.

### 2.3 예외: 트랙/마커 유실 처리

S2(Following) 상태에서 `Flag_VTP=false ∧ Flag_marker=false`인 경우:

```
track_lost_counter += 1

if track_lost_counter < TRACK_LOST_TIMEOUT:
    action = HOLD_POSITION   (마지막 유효 위치 명령 유지, 고도 유지)
else:
    action = HOLD_POSITION   (baseline에서는 계속 유지. 복구 로직은 Phase 2)

TRACK_LOST_TIMEOUT = 5 / T_IPS = 25 frames (5초)
```

Flag_VTP 또는 Flag_marker가 다시 true가 되면 `track_lost_counter = 0`으로 리셋.

**설계 이유:** baseline에서는 복구 행동(회전 탐색 등)을 구현하지 않는다.
Simple 트랙 + 기본 날씨에서는 유실이 드물며, 유실 빈도 자체가
Phase 2 개선의 필요성을 판단하는 지표가 된다.

---

## 3. Image Processing System (IPS) 상세

### 3.1 Step 1: 채널 변환 (Channel Conversion)

**목적:** RGB 프레임에서 빨간색 트랙 픽셀을 강조하고 배경을 억제.

**수식:**
```
F(n, m) = fR(n, m) - (fG(n, m) / G_B_GAIN) - (fB(n, m) / G_B_GAIN)
```

여기서:
- `fR(n,m)`, `fG(n,m)`, `fB(n,m)`: 픽셀 (n,m)의 Red, Green, Blue 채널 강도값 (uint8, 0~255)
- `G_B_GAIN = 2`: Green과 Blue의 감산 게인. 원본 코드에서 G, B 모두 동일한 값 사용.
- 연산은 **double 변환 후** 수행 (uint8 오버플로 방지)

**코드 근거:** `variables.m`에서 `G_B_GAIN = 2`는 단일 변수로 G, B 모두에 적용됨.
논문 수식(1)의 `G_G`, `G_B`는 둘 다 이 값을 참조.

**동작 원리:**
- 빨간 트랙: fR이 높고 fG, fB가 낮음 → F가 크게 양수
- 회색 배경: fR ≈ fG ≈ fB → F ≈ fR - fR/2 - fR/2 = 0
- 초록/파랑: fR이 낮고 fG 또는 fB가 높음 → F가 음수

**구현 코드:**
```matlab
F = double(IMG(:,:,1)) - double(IMG(:,:,2))/G_B_GAIN - double(IMG(:,:,3))/G_B_GAIN;
```

### 3.2 Step 2: 이진화 (Binarization)

**수식:**
```
Fbin(n, m) = 1   if F(n, m) >= BINARIZER_THRESHOLD
             0   otherwise
```

**파라미터:** `BINARIZER_THRESHOLD = 100`

**구현 코드:**
```matlab
Fbin = F >= BINARIZER_THRESHOLD;   % logical 행렬 (H × W)
```

### 3.3 Step 3: 침식 (Erosion)

두 가지 목적에 따라 **서로 다른 커널**을 사용한다:

| 용도 | 커널 형상 | 파라미터 | 크기 |
|------|----------|---------|------|
| 트랙 추종 | 정사각형 (square) | SQUARE_KERNEL = 8 | 8×8 pixel |
| 마커 감지 | 원형 (disk) | DISK_KERNEL = 11 | 반지름 11 pixel |

**구현 코드:**
```matlab
% 트랙 추종용 침식
se_square = strel('square', SQUARE_KERNEL);
IMG_eroded = imerode(Fbin, se_square);

% 마커 감지용 침식 (별도 경로에서 Fbin에 직접 적용)
se_disk = strel('disk', DISK_KERNEL);
IMG_marker_eroded = imerode(Fbin, se_disk);
```

### 3.4 Step 4: VTP 탐색 (Arc Mask 기반)

UniSannio 알고리즘의 **핵심 독창 기법**.

#### 3.4.1 개념

드론 위치(프레임 중심)에서 일정 거리 앞에 있는 트랙 위의 한 점을
VTP(Virtual Target Point)로 삼아 추종한다.
이것이 Pure Pursuit의 "look-ahead point"에 해당한다.

#### 3.4.2 Arc Mask의 기하학적 정의

Arc Mask는 다음 3가지 조건을 동시에 만족하는 픽셀 집합이다:

```
ArcMask(n, m) = radius_condition(n, m)
                ∧ angle_condition(n, m)
```

**반경 조건:**
```
radius_condition = (MIN_RADIUS_CROWN <= dist(n,m) <= MAX_RADIUS_CROWN)

dist(n, m) = sqrt((n - COG_X)^2 + (m - COG_Y)^2)
```

**각도 조건:**
```
angle_condition = (|angle_diff(n,m)| <= half_fov)

angle(n, m) = atan2(n - COG_X, m - COG_Y)
half_fov = pi / FOV                              (= π/2.9 ≈ 1.08 rad ≈ 62°)
angle_diff = wrap_to_pi(angle - theta_prev)       ([-π, π] 범위로 정규화)
```

**atan2 좌표계 규약:**
- `atan2(Δrow, Δcol)` 사용. 즉 첫 번째 인자가 행 차이, 두 번째가 열 차이.
- angle = 0: 오른쪽(열 증가 방향)
- angle = π/2: 아래쪽(행 증가 방향)
- angle 양수 방향: 시계 방향 (이미지 좌표계에서)

**파라미터:**
```matlab
MIN_RADIUS_CROWN = 27;    % 내경 [pixel]
MAX_RADIUS_CROWN = 28;    % 외경 [pixel] → 두께 = 1 pixel
FOV = 2.9;                % 시야각 = 2π/FOV ≈ 124°
COG_X = 60;               % 프레임 중심 행
COG_Y = 80;               % 프레임 중심 열
```

#### 3.4.3 theta_prev 초기화 규칙

**결정사항:** 초기 상태에서 theta_prev가 정의되지 않은 경우의 처리.

```
if theta_prev == NaN:                      (← 첫 프레임 또는 미초기화)
    angle_condition = true for all pixels   (← 360° 전체 탐색)
else:
    angle_condition = 일반 FOV 제한 적용
```

즉, **첫 유효 VTP 감지 전까지는 FOV 제한 없이 전체 원호를 탐색**한다.
첫 VTP가 감지되면 그 방향으로 theta_prev를 설정하고,
이후부터 directional Arc Mask를 적용한다.

#### 3.4.4 VTP 좌표 추출 (확정)

**결정사항:** Arc Mask 내 양성 픽셀의 **산술 평균(mean)**을 VTP로 사용한다.

```
masked = IMG_eroded AND ArcMask

[vtp_rows, vtp_cols] = find(masked)

if numel(vtp_rows) > 0:
    x_VTP = mean(vtp_rows)       ← 행 좌표 평균
    y_VTP = mean(vtp_cols)       ← 열 좌표 평균
    theta_prev = atan2(x_VTP - COG_X, y_VTP - COG_Y)
    ex = x_VTP - COG_X
    ey = y_VTP - COG_Y
    Flag_VTP = true
else:
    Flag_VTP = false
    ex, ey = 이전 값 hold (PP에서 track_lost_counter 증가)
```

대안(connected component 중심, 밀집 방향 중심 등)은 Phase 2에서 검토.
baseline에서는 **mean만 사용**한다.

#### 3.4.5 구현 코드

```matlab
function [ex, ey, Flag_VTP, theta_next] = findVTP(IMG_eroded, theta_prev)
    persistent arc_distances arc_angles
    if isempty(arc_distances)
        [M, N] = meshgrid(1:160, 1:120);  % col, row 순서 주의
        arc_distances = sqrt((N - COG_X).^2 + (M - COG_Y).^2);
        arc_angles = atan2(N - COG_X, M - COG_Y);
    end

    % 반경 조건
    radius_mask = (arc_distances >= MIN_RADIUS_CROWN) & ...
                  (arc_distances <= MAX_RADIUS_CROWN);

    % 각도 조건
    if isnan(theta_prev)
        % 초기 상태: 360° 전체 탐색
        angle_mask = true(120, 160);
    else
        half_fov = pi / FOV;
        angle_diff = mod(arc_angles - theta_prev + pi, 2*pi) - pi;
        angle_mask = abs(angle_diff) <= half_fov;
    end

    % Arc Mask 적용
    arc_mask = radius_mask & angle_mask;
    masked = IMG_eroded & arc_mask;

    % VTP 추출
    [vtp_rows, vtp_cols] = find(masked);
    if numel(vtp_rows) > 0
        x_VTP = mean(vtp_rows);
        y_VTP = mean(vtp_cols);
        ex = x_VTP - COG_X;
        ey = y_VTP - COG_Y;
        theta_next = atan2(ex, ey);
        Flag_VTP = true;
    else
        ex = 0;
        ey = 0;
        theta_next = theta_prev;  % 이전 방향 유지
        Flag_VTP = false;
    end
end
```

**성능 참고:** `arc_distances`와 `arc_angles`는 persistent로 1회만 계산.
매 프레임마다 재계산하지 않아 실시간 성능에 유리하다.

### 3.5 Step 5: 착륙 마커 감지

**동작:**
1. 이진화 프레임(Fbin)에 원형 커널(DISK_KERNEL=11)로 침식
2. 가느다란 트랙 라인은 소멸, 원형 마커만 잔존
3. 잔존 픽셀의 무게중심 → 마커 위치

**감지 판정 (확정):**
```
marker_pixels = numel(find(IMG_marker_eroded))

Flag_marker = (marker_pixels >= MARKER_MIN_PIXELS)
              ∧ (Flag_VTP == false)

MARKER_MIN_PIXELS = 10    [pixel] (baseline 값, 2026 환경에서 재튜닝 필요)
```

`~isempty`만으로는 노이즈 blob도 마커로 오인할 수 있으므로,
**최소 픽셀 수 임계값**을 추가한다. 값 10은 보수적인 초기값이며,
2026 환경에서 마커 크기(직경 20cm)에 맞춰 재조정한다.

**구현 코드:**
```matlab
function [ex, ey, Flag_marker] = findMarker(Fbin)
    se_disk = strel('disk', DISK_KERNEL);
    IMG_marker = imerode(Fbin, se_disk);

    [mark_rows, mark_cols] = find(IMG_marker);

    if numel(mark_rows) >= MARKER_MIN_PIXELS
        x_MARK = mean(mark_rows);
        y_MARK = mean(mark_cols);
        ex = x_MARK - COG_X;
        ey = y_MARK - COG_Y;
        Flag_marker = true;
    else
        ex = 0;
        ey = 0;
        Flag_marker = false;
    end
end
```

### 3.6 IPS 전체 통합

```matlab
function [ex, ey, Flag_VTP, Flag_marker] = IPS(IMG)
    persistent theta_prev
    if isempty(theta_prev)
        theta_prev = NaN;  % 초기화: 360° 전체 탐색 모드
    end

    % Step 1: 채널 변환
    F = double(IMG(:,:,1)) - double(IMG(:,:,2))/G_B_GAIN ...
                            - double(IMG(:,:,3))/G_B_GAIN;

    % Step 2: 이진화
    Fbin = F >= BINARIZER_THRESHOLD;

    % Step 3a: 트랙용 침식
    IMG_eroded = imerode(Fbin, strel('square', SQUARE_KERNEL));

    % Step 4: VTP 탐색
    [ex_vtp, ey_vtp, Flag_VTP, theta_next] = findVTP(IMG_eroded, theta_prev);

    % Step 5: 마커 감지
    [ex_mark, ey_mark, Flag_marker] = findMarker(Fbin);

    % Flag 상호배타성 적용: VTP가 있으면 마커 무시
    if Flag_VTP
        ex = ex_vtp;
        ey = ey_vtp;
        Flag_marker = false;       % VTP 우선
        theta_prev = theta_next;
    elseif Flag_marker
        ex = ex_mark;
        ey = ey_mark;
        Flag_VTP = false;          % 명시적 false
        % theta_prev는 갱신하지 않음 (마커 추종 시 방향 무관)
    else
        ex = 0;
        ey = 0;
        % theta_prev 유지 (유실 시 마지막 방향 보존)
    end
end
```

**Flag 상호배타성 구현:** IPS 내부에서 `Flag_VTP`와 `Flag_marker`가
동시에 true가 되지 않도록 강제한다. VTP가 감지되면 마커 플래그를 억제.
이 규칙은 State Machine의 전이 조건 `¬Flag_VTP ∧ Flag_marker`와 일관된다.

---

## 4. Path Planner (PP) 상세

### 4.1 핵심 원리

PP는 IPS의 오차(ex, ey)를 받아 월드 좌표계 위치 명령(xw, yw, zw)을 생성한다.
이미지 공간 오차를 직접 위치 갱신량으로 변환하는 **IBVS(Image-Based Visual Servoing)** 방식이다.

### 4.2 Following 모드 (S2)

```
x_{k+1} = x_k + α × ex
y_{k+1} = y_k + α × ey
z_{k+1} = Z_LOW                    (고도 일정: -1 m)
```

**속도와 α의 관계:**

α = GAIN_TRACK = 0.0038이고, PP가 T_PP = 0.005s마다 실행되므로,
1초에 1/0.005 = 200회 갱신. VTP는 반경 ≈ 27.5 pixel 위에 있으므로:

```
V_D ≈ (α / T_PP) × (Rmax + Rmin) / 2
    = (0.0038 / 0.005) × 27.5
    = 0.76 × 27.5
    ≈ 20.9 [m/s]  ← 단, 이 값은 즉시 반영되는 것이 아님 (아래 주의사항 참조)
```

**단위 주의:** α의 단위는 코드에서 명시되지 않는다.
위 계산은 `α × ex`가 [m] 단위라고 가정한 경우이며,
논문 Fig.11에서 실측 속도가 약 0.2~0.3 m/s인 점과 비교하면
위 공식의 값은 **순간 변위 누적률**에 해당하고,
실제 드론 속도는 저수준 제어기의 동특성에 의해 결정된다.
핵심은 **α가 클수록 빠르고 부정확, 작을수록 느리고 정확**하다는 정성적 관계이다.

### 4.3 End-Marker 모드 (S3)

마커 중심에 정렬하는 단계. centered_ok 판정까지 이 모드에 머문다.

```
error_norm = sqrt(ex^2 + ey^2)

if error_norm <= MAX_ERROR_LANDING:
    center_counter += 1
    x_{k+1} = x_k              (수평 위치 고정)
    y_{k+1} = y_k
    z_{k+1} = Z_LOW            (아직 착륙 아님, 고도 유지)
else:
    center_counter = 0          (오차 초과 시 카운터 리셋)
    x_{k+1} = x_k + β × ex
    y_{k+1} = y_k + β × ey
    z_{k+1} = Z_LOW
```

`center_counter >= CENTER_CONFIRM_FRAMES`(= 800, 약 4초)이면
S4(LANDING)로 전이하여 `z = 0`(착륙) 명령.

### 4.4 미분 제어 (Derivative Part)

급격한 방향 변화(코너)에서의 오버슈트를 억제한다.

**갱신 시점:** IPS 갱신 시점(5Hz)에서만 차분을 계산한다.
200Hz PP 스텝에서는 이전 hold 값의 차분이 대부분 0이므로 무의미.

```
% IPS가 새 값을 제공한 시점에서만 실행
Δex = ex - ex_prev_pp
Δey = ey - ey_prev_pp
Δe_norm = sqrt(Δex^2 + Δey^2)

if Δe_norm > CHANGE_DERIVATIVE_ERROR_THRESHOLD:
    deriv_hold_counter = TIME_HOLD / T_PP      (= 0.2/0.005 = 40 스텝)

ex_prev_pp = ex
ey_prev_pp = ey
```

```
% 매 PP 스텝에서
if deriv_hold_counter > 0:
    x_{k+1} += DERIVATIVE_GAIN × Δex
    y_{k+1} += DERIVATIVE_GAIN × Δey
    deriv_hold_counter -= 1
```

### 4.5 PP 전체 통합

```matlab
function [xw, yw, zw] = PathPlanner(ex, ey, Flag_VTP, Flag_marker, ...
                                     current_z, ips_updated)
    persistent x_prev y_prev mission_state hover_counter center_counter
    persistent deriv_hold_counter track_lost_counter
    persistent ex_prev_pp ey_prev_pp delta_ex delta_ey

    % 초기화
    if isempty(mission_state)
        mission_state = 1;  % TAKEOFF
        x_prev = 0; y_prev = 0;  % init.posNED에서 설정
        hover_counter = 0; center_counter = 0;
        deriv_hold_counter = 0; track_lost_counter = 0;
        ex_prev_pp = 0; ey_prev_pp = 0;
        delta_ex = 0; delta_ey = 0;
    end

    % 미분 계산 (IPS 갱신 시점에서만)
    if ips_updated
        delta_ex = ex - ex_prev_pp;
        delta_ey = ey - ey_prev_pp;
        delta_norm = sqrt(delta_ex^2 + delta_ey^2);
        if delta_norm > CHANGE_DERIVATIVE_ERROR_THRESHOLD
            deriv_hold_counter = uint32(TIME_HOLD / Ts);  % 40 steps
        end
        ex_prev_pp = ex;
        ey_prev_pp = ey;
    end

    switch mission_state
        case 1  % TAKEOFF
            xw = x_prev;
            yw = y_prev;
            zw = Z_LOW;  % 목표 고도로 상승
            if (current_z >= Z_HIGH) && (current_z <= Z_LOW) && Flag_VTP
                hover_counter = hover_counter + 1;
                if hover_counter >= 40  % HOVER_CONFIRM_FRAMES
                    mission_state = 2;
                end
            else
                hover_counter = 0;
            end

        case 2  % FOLLOWING
            if Flag_VTP
                track_lost_counter = 0;
                xw = x_prev + GAIN_TRACK * ex;
                yw = y_prev + GAIN_TRACK * ey;
                % 미분항 적용
                if deriv_hold_counter > 0
                    xw = xw + DERIVATIVE_GAIN * delta_ex;
                    yw = yw + DERIVATIVE_GAIN * delta_ey;
                    deriv_hold_counter = deriv_hold_counter - 1;
                end
                zw = Z_LOW;
            elseif Flag_marker
                mission_state = 3;  % S2 → S3 전이
                center_counter = 0;
                xw = x_prev + GAIN_LANDING * ex;
                yw = y_prev + GAIN_LANDING * ey;
                zw = Z_LOW;
            else
                % Track lost: 위치 유지
                track_lost_counter = track_lost_counter + 1;
                xw = x_prev;
                yw = y_prev;
                zw = Z_LOW;
            end

        case 3  % END_MARKER
            error_norm = sqrt(ex^2 + ey^2);
            if error_norm <= MAX_ERROR_LANDING
                center_counter = center_counter + 1;
                xw = x_prev;
                yw = y_prev;
                zw = Z_LOW;
                if center_counter >= uint32(DELAY_LANDING / Ts)  % 800
                    mission_state = 4;
                end
            else
                center_counter = 0;
                xw = x_prev + GAIN_LANDING * ex;
                yw = y_prev + GAIN_LANDING * ey;
                zw = Z_LOW;
            end

        case 4  % LANDING
            xw = x_prev;
            yw = y_prev;
            zw = 0;  % 착륙 명령
    end

    x_prev = xw;
    y_prev = yw;
end
```

---

## 5. Heading-Free 설계

UniSannio의 핵심 설계 결정: **yaw(heading)를 일절 제어하지 않는다.**

멀티로터는 heading 변경 없이 모든 방향으로 이동 가능하다.
heading 제어를 제거함으로써:
- yaw 회전 대기 시간 제거 → 미션 시간 단축
- yaw 제어 루프의 지연/오버슈트 제거 → 안정성 향상
- 구현 단순화

**코드 근거:**
```matlab
% commandVars.m
Command.yawStepAmplitude = 0;   % yaw 명령 = 0
```

**2026 주의:** 외란(바람)에 의한 yaw drift 시 카메라 방향이 틀어질 수 있다.
Round 2(실제 드론)에서 문제가 될 경우 Phase 2에서 yaw 보정 검토.

---

## 6. 예외 처리 정리

| 상황 | 조건 | 행동 | 비고 |
|------|------|------|------|
| 첫 프레임 VTP 없음 | theta_prev == NaN | 360° 전체 탐색 | Section 3.4.3 |
| Following 중 트랙 유실 | ¬Flag_VTP ∧ ¬Flag_marker | 위치 유지, lost_counter 증가 | Section 2.3 |
| Following 중 마커 감지 | ¬Flag_VTP ∧ Flag_marker | S3 전이 | Section 2.2 |
| End-Marker 중 마커 유실 | Flag_marker 소실 | 위치 유지, 마커 재탐색 대기 | baseline: hold |
| 착륙 정렬 중 오차 초과 | error_norm > MAX_ERROR_LANDING | center_counter 리셋, 재접근 | Section 4.3 |

---

## 7. 2026 환경 이식 검증 항목

### 7.1 그대로 유지 (아키텍처)

- IPS → PP → Controller cascade 구조
- 5Hz / 200Hz 이중 주기
- Heading-free 병진 추종
- 4-상태 State Machine
- Arc Mask 기반 VTP 탐색
- Following / Landing 게인 분리

### 7.2 환경 차이 및 검증 방법

| 항목 | 2020 원본 | 2026 대회 | 검증 방법 |
|------|----------|----------|----------|
| 3D 환경 | MATLAB VR | Unreal Engine | 육안 + 이진화 결과 비교 |
| 트랙 폭 | 1 cm | 10 cm | SQUARE_KERNEL sweep {8,12,16,20} |
| 마커 직경 | 2 cm | 20 cm | DISK_KERNEL sweep {11,15,19,23} |
| 날씨 변동 | 없음 | 태양/적설 랜덤 | 기본 날씨에서만 baseline 검증 |
| 트랙 교차 | 없음 | Advanced에서 허용 | baseline에서는 미대응, 실패 기록만 |
| 프레임 크기 | 120×160 | 확인 필요 | 모델 설치 후 즉시 확인 |
| MATLAB 버전 | R2019b | R2025b | 코드 생성 테스트 |

### 7.3 Baseline 재튜닝 순서 (Phase 1 내)

2026 모델 설치 후 아래 순서로 파라미터를 검증/재조정한다:

1. 프레임 크기 확인 → COG_X, COG_Y 재계산
2. 기본 날씨에서 카메라 프레임 캡처 → 트랙의 실제 RGB 분포 확인
3. BINARIZER_THRESHOLD 재탐색 (이진화 결과 육안 확인)
4. SQUARE_KERNEL 재조정 (트랙 폭 변화 반영)
5. MIN/MAX_RADIUS_CROWN 재조정 (프레임 크기 변화 반영)
6. DISK_KERNEL + MARKER_MIN_PIXELS 재조정 (마커 크기 변화 반영)
7. GAIN_TRACK, GAIN_LANDING 재조정 (속도-정확도 확인)
8. 전체 Simple 트랙 통과 테스트

---

## 8. 전체 파라미터 요약

### 8.1 Image Processing System

| 파라미터 | 변수명 | 값 | 단위 | 변경 가능 시점 |
|---------|--------|-----|------|-------------|
| G/B 감산 게인 | G_B_GAIN | 2 | - | 7.3 Step 2 이후 |
| 이진화 임계값 | BINARIZER_THRESHOLD | 100 | intensity | 7.3 Step 3 |
| 정사각 침식 커널 | SQUARE_KERNEL | 8 | pixel | 7.3 Step 4 |
| 원형 침식 커널 | DISK_KERNEL | 11 | pixel | 7.3 Step 6 |
| 마커 최소 픽셀 수 | MARKER_MIN_PIXELS | 10 | pixel | 7.3 Step 6 |
| Arc Mask 내경 | MIN_RADIUS_CROWN | 27 | pixel | 7.3 Step 5 |
| Arc Mask 외경 | MAX_RADIUS_CROWN | 28 | pixel | 7.3 Step 5 |
| 시야각 파라미터 | FOV | 2.9 | - | Phase 2 |
| 프레임 높이 | FRAME_SIZE_HEIGHT | 120 | pixel | 7.3 Step 1 |
| 프레임 너비 | FRAME_SIZE_WIDTH | 160 | pixel | 7.3 Step 1 |
| 프레임 중심 X | COG_X | 60 | pixel | 7.3 Step 1 |
| 프레임 중심 Y | COG_Y | 80 | pixel | 7.3 Step 1 |

### 8.2 Path Planner

| 파라미터 | 변수명 | 값 | 단위 | 변경 가능 시점 |
|---------|--------|-----|------|-------------|
| 추종 게인 | GAIN_TRACK | 0.0038 | (코드 단위) | 7.3 Step 7 |
| 착륙 게인 | GAIN_LANDING | 0.0022 | (코드 단위) | 7.3 Step 7 |
| 미분 게인 | DERIVATIVE_GAIN | 0.002 | (코드 단위) | Phase 2 |
| 미분 활성 임계값 | CHANGE_DERIVATIVE_ERROR_THRESHOLD | 2 | pixel | Phase 2 |
| 미분 유지 시간 | TIME_HOLD | 0.2 | s | Phase 2 |
| 착륙 지연 | DELAY_LANDING | 4 | s | 7.3 Step 7 |
| 착륙 오차 한계 | MAX_ERROR_LANDING | 5 | pixel | 7.3 Step 6 |

### 8.3 시스템 레벨

| 파라미터 | 변수명 | 값 | 단위 | 비고 |
|---------|--------|-----|------|------|
| PP 주기 | Ts | 0.005 | s | 변경 불가 (시스템 고정) |
| IPS 주기 | VTs | 0.2 | s | 변경 불가 |
| 시뮬레이션 시간 | TFinal | 100 | s | 대회 규격 확인 필요 |
| 호버링 고도 하한 | Z_LOW | -1 | m (NED) | |
| 호버링 고도 상한 | Z_HIGH | -1.2 | m (NED) | |
| 호버링 확인 프레임 | HOVER_CONFIRM_FRAMES | 40 | PP steps | = 0.2s |
| 착륙 확인 프레임 | CENTER_CONFIRM_FRAMES | 800 | PP steps | = 4s |
| 트랙 유실 타임아웃 | TRACK_LOST_TIMEOUT | 25 | IPS frames | = 5s |

---

## 9. Acceptance Criteria (완료 판정 기준)

### 9.1 기능 검증

| ID | 항목 | 합격 기준 | 테스트 방법 |
|----|------|----------|-----------|
| A1 | 코드 생성 | Build 에러 0건 | Code Generation 버튼 클릭 |
| A2 | Simple 트랙 완주 | 5개 트랙 중 4개 이상 성공 | Track Builder로 생성, 기본 날씨 |
| A3 | 마커 착륙 | A2 성공 트랙에서 마커 위 착륙 | 착륙 시 마커 중심 거리 확인 |
| A4 | 시뮬 시간 | Simple 트랙 완주 시 100초 이내 | Simulink 시뮬레이션 시간 확인 |
| A5 | 반복 안정성 | 동일 트랙 3회 연속 실행 시 3회 모두 성공 | 동일 조건 반복 |

### 9.2 Simple 트랙 테스트 규격

대회 Rules 기준:
- 세그먼트 수: 4~6개
- 세그먼트 간 각도: 15°~345°
- 근접 라인 없음

Track Builder 앱으로 아래 5개 트랙을 생성하여 테스트:
- T1: 4세그먼트, 완만한 각도 (120°~240°)
- T2: 6세그먼트, 혼합 각도
- T3: 4세그먼트, 급커브 포함 (30° 또는 330°)
- T4: 5세그먼트, 긴 직선 + 급커브
- T5: 6세그먼트, 지그재그

### 9.3 실패 모드 기록 (필수)

Phase 1 완료 시 아래 항목을 문서화한다:

| 기록 항목 | 형식 |
|----------|------|
| 트랙 유실 발생 빈도 | 트랙별 lost_counter 최대값 |
| 이진화 실패 사례 | 스크린샷 (이진화 전/후) |
| 날씨 변동 시 동작 | 태양 방위 0°/180°, 적설 0%/50%에서 관찰 |
| 교차 트랙 실패 | Advanced 트랙 1개에서 관찰, 실패 지점 기록 |

---

## 부록 A: 논문 Figure 대응표

| 논문 Figure | 내용 | Tech Spec 대응 섹션 |
|------------|------|-------------------|
| Fig. 3 | Pure Pursuit 개념도 | 3.4.1 |
| Fig. 4 | 시스템 아키텍처 | 1.1, 1.2 |
| Fig. 5 | State Machine | 2 |
| Fig. 6 | 채널변환 → 이진화 → 침식 | 3.1, 3.2, 3.3 |
| Fig. 7 | Arc Mask 구조 | 3.4.2 |
| Fig. 8 | Arc Mask 적용 결과 | 3.4.4 |
| Fig. 9 | 마커 침식 결과 | 3.5 |
| Fig. 10 | α값별 경로 비교 | 4.2 |
| Fig. 11 | 드론 속도 | 4.2 |
| Algorithm 1 | IPS 의사코드 | 3.6 |
| Algorithm 2 | PP 의사코드 | 4.5 |
| Table I | 파라미터 표 | 8 |

## 부록 B: Simulink 블록 매핑 가이드

| 알고리즘 단계 | Simulink 블록 후보 | 비고 |
|-------------|-------------------|------|
| 채널 변환 | MATLAB Function | 행렬 연산 |
| 이진화 | MATLAB Function | 임계값 비교 |
| 침식 | Image Processing Toolbox: Erosion | strel 사전 정의 |
| Arc Mask + VTP | MATLAB Function | persistent 활용 |
| State Machine | Stateflow Chart | 4-상태 전이 |
| PP 위치 갱신 | MATLAB Function | persistent 활용 |
| 미분 제어 | PP 내부에 통합 | 조건부 활성화 |

**코드 생성 제약:** MATLAB Function 블록 내에서 아래는 사용 금지:
- 동적 메모리 할당 (`cell`, 가변 크기 배열)
- `eval`, `feval`
- 일부 Image Processing Toolbox 함수 (코드 생성 미지원 시 대체 구현)
- try-catch 문

## 부록 C: 용어 정의

| 용어 | 정의 |
|------|------|
| IPS | Image Processing System. 카메라 프레임을 처리하여 오차/플래그를 출력하는 모듈 |
| PP | Path Planner. 오차를 받아 위치 명령을 생성하는 모듈 |
| VTP | Virtual Target Point. Arc Mask로 탐색한 트랙 위의 목표점 |
| CoM / CoG | Center of Mass / Center of Gravity. 프레임 중심 = 드론 투영 위치 |
| NED | North-East-Down 좌표계. z 양수가 아래 방향 |
| Arc Mask | 원호 형태의 이진 마스크. VTP 탐색 영역을 정의 |
| Flag_VTP | 트랙(VTP) 감지 여부 플래그 |
| Flag_marker | 착륙 마커 감지 여부 플래그 |
| IBVS | Image-Based Visual Servoing. 이미지 공간 오차를 직접 제어 입력으로 변환 |
| hold | 새 값이 없을 때 이전 값을 유지하는 방식 (zero-order hold) |
| centered | 마커 중심과의 오차가 허용 범위 이내인 상태 |

## 부록 D: Deviation Log (템플릿)

Phase 1 구현 중 원본 코드/논문과의 차이를 발견하면 여기에 기록한다.

| ID | 항목 | 원본 (코드/논문) | 본 구현 | 이유 |
|----|------|-----------------|--------|------|
| D1 | Centered 판정 | ex==0 && ey==0 (논문 Alg.2) | sqrt(ex²+ey²) ≤ 5 (코드 MAX_ERROR_LANDING) | 이산 이미지에서 정확히 0은 비현실적 |
| D2 | 마커 감지 조건 | ~isempty (논문 암시) | numel ≥ MARKER_MIN_PIXELS | 노이즈 blob 오인 방지 |
| D3 | theta_prev 초기화 | 미정의 (논문/코드) | NaN → 360° 전체 탐색 | 첫 프레임 안정성 보장 |
| D4 | S2→S3 전이조건 | Flag_VTP ∧ Flag_marker (논문 Fig.5) | ¬Flag_VTP ∧ Flag_marker | 상호배타성 원칙과 일관성 확보 |
| ... | ... | ... | ... | ... |
