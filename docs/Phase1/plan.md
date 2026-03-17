# Phase 1 Implementation Plan

> **목적:** `docs/Phase1/tech_spec.md`에 기반하여 UniSannio Vision-Based Path Following 알고리즘을 단계별로 구현한다.
>
> **사용법:** Plan Mode에서 이 파일을 열고, 한 번에 하나의 체크박스만 해결하도록 에이전트에 지시한다.
> 각 항목 완료 시 커밋하고, 체크박스를 `[x]`로 업데이트한다.
>
> **컨텍스트 소실 대비:** 각 항목은 독립적으로 수행 가능하도록 참조 파일 경로, 코드 스니펫, 수정 대상을 명시한다.
> 새 세션에서는 이 plan.md와 해당 항목에 명시된 파일만 읽으면 작업을 재개할 수 있다.

---

## 현재 상태 요약

### 완료된 항목
- [x] Tech Spec v2.0 작성 완료 (`docs/Phase1/tech_spec.md`)
- [x] 원본 UniSannio 2020 코드 확보 (`UniSannio_Team/` 디렉토리)
- [x] 파라미터 정의 완료 (`UniSannio_Team/variables.m`)
- [x] 시뮬레이션 환경 구축 (Simulink 모델, 드론 동역학, 센서)

### 코드베이스 핵심 파일 위치
| 파일 | 경로 | 역할 |
|------|------|------|
| 전역 파라미터 | `UniSannio_Team/variables.m` | IPS/PP 모든 파라미터 |
| 초기화 스크립트 | `UniSannio_Team/utilities/startVars.m` | Ts, VTs, init.posNED 등 |
| Flight Control | `UniSannio_Team/controller/flightControlSystem.slx` | IPS/PP 블록 삽입 대상 |
| 메인 시뮬레이션 | `UniSannio_Team/mainModels/parrotMinidroneCompetition.slx` | 전체 시뮬레이션 |
| 명령 변수 | `UniSannio_Team/tasks/commandVars.m` | yaw=0 (heading-free) |

---

## 구현 계획

### Step 1: IPS — 채널 변환 + 이진화 함수 작성
- [x] **`UniSannio_Team/IPS/channelConvertAndBinarize.m` 생성**

**참조:** `docs/Phase1/tech_spec.md` Section 3.1, 3.2

**수정/생성 대상:**
- 새 파일: `UniSannio_Team/IPS/channelConvertAndBinarize.m`

**구현 내용:**
RGB 이미지를 입력받아 빨간색 채널을 강조하고 이진화한 logical 행렬을 반환한다.

**코드 스니펫 (tech_spec 기반):**
```matlab
function Fbin = channelConvertAndBinarize(IMG, G_B_GAIN, BINARIZER_THRESHOLD)
%channelConvertAndBinarize  RGB → 빨간색 강조 → 이진화
%   IMG: uint8 H×W×3 RGB 프레임
%   Fbin: logical H×W 이진화 결과
%
% 참조: tech_spec.md Section 3.1, 3.2
% 파라미터 출처: variables.m (G_B_GAIN=2, BINARIZER_THRESHOLD=100)

    % Step 1: 채널 변환 — R에서 G/B를 감산하여 빨간색 강조
    F = double(IMG(:,:,1)) - double(IMG(:,:,2))/G_B_GAIN ...
                            - double(IMG(:,:,3))/G_B_GAIN;

    % Step 2: 이진화 — 임계값 이상이면 트랙 픽셀
    Fbin = F >= BINARIZER_THRESHOLD;
end
```

**파라미터 (variables.m:15-16에서 확인):**
```matlab
G_B_GAIN = 2;                % Green/Blue 감산 게인
BINARIZER_THRESHOLD = 100;   % 이진화 임계값
```

**검증 방법:**
- 테스트 이미지(빨간 트랙 + 회색 배경)에서 `sum(Fbin(:))` > 0 확인
- 회색 균일 이미지에서 `sum(Fbin(:))` == 0 확인

**완료 조건:** 함수가 생성되고, 입출력 시그니처가 tech_spec과 일치

---

### Step 2: IPS — 침식 함수 작성 (트랙용 + 마커용)
- [x] **`UniSannio_Team/IPS/erodeTrackAndMarker.m` 생성**

**참조:** `docs/Phase1/tech_spec.md` Section 3.3

**수정/생성 대상:**
- 새 파일: `UniSannio_Team/IPS/erodeTrackAndMarker.m`
- 의존: Step 1의 출력 `Fbin`

**구현 내용:**
이진화된 프레임에 두 가지 커널로 침식을 수행한다:
1. 정사각형 커널(8×8) → 트랙 추종용 (가느다란 노이즈 제거)
2. 원형 커널(반지름 11) → 마커 감지용 (트랙 라인 소멸, 원형 마커 잔존)

**코드 스니펫:**
```matlab
function [IMG_eroded, IMG_marker_eroded] = erodeTrackAndMarker(Fbin, ...
    SQUARE_KERNEL, DISK_KERNEL)
%erodeTrackAndMarker  이진 프레임에 트랙용/마커용 침식 수행
%   Fbin: logical H×W (Step 1 출력)
%   IMG_eroded: logical H×W — 트랙 추종용 (정사각형 침식)
%   IMG_marker_eroded: logical H×W — 마커 감지용 (원형 침식)
%
% 참조: tech_spec.md Section 3.3
% 파라미터 출처: variables.m (SQUARE_KERNEL=8, DISK_KERNEL=11)

    % 트랙 추종용 침식: 정사각형 8×8
    se_square = strel('square', SQUARE_KERNEL);
    IMG_eroded = imerode(Fbin, se_square);

    % 마커 감지용 침식: 원형 반지름 11
    se_disk = strel('disk', DISK_KERNEL);
    IMG_marker_eroded = imerode(Fbin, se_disk);
end
```

**파라미터 (variables.m:20, 35에서 확인):**
```matlab
SQUARE_KERNEL = 8;    % 정사각형 침식 커널 크기 [pixel]
DISK_KERNEL = 11;     % 원형 침식 커널 반지름 [pixel]
```

**코드 생성 주의:** `strel`과 `imerode`는 Image Processing Toolbox 함수.
Simulink MATLAB Function 블록에서 코드 생성 미지원 시 **수동 구현 필요** (Phase 1 내 확인 필요).
수동 구현 시 strel 대신 `ones(SQUARE_KERNEL)` 기반 min-filter로 대체 가능.

**완료 조건:** 함수 생성, 트랙 이미지에서 `IMG_eroded`에 트랙 잔존, `IMG_marker_eroded`에 트랙 소멸 확인

---

### Step 3: IPS — Arc Mask VTP 탐색 함수 작성 (findVTP)
- [x] **`UniSannio_Team/IPS/findVTP.m` 생성**

**참조:** `docs/Phase1/tech_spec.md` Section 3.4 (전체: 3.4.1~3.4.5)

**수정/생성 대상:**
- 새 파일: `UniSannio_Team/IPS/findVTP.m`
- 의존: Step 2의 출력 `IMG_eroded`

**구현 내용:**
Arc Mask(원호 형태 마스크)를 적용하여 Virtual Target Point를 탐색한다.
이것이 UniSannio 알고리즘의 **핵심 독창 기법** — Pure Pursuit의 look-ahead point.

**핵심 로직:**
1. 프레임 중심(COG_X=60, COG_Y=80)에서 각 픽셀까지의 거리/각도를 계산 (persistent, 1회만)
2. 반경 조건: 27 ≤ dist ≤ 28 (두께 1pixel의 원호)
3. 각도 조건: `|wrap_to_pi(angle - theta_prev)| ≤ π/FOV` (약 62°)
4. theta_prev가 NaN이면 360° 전체 탐색 (초기화 전)
5. 마스크 내 양성 픽셀의 **산술 평균**이 VTP 좌표

**코드 스니펫 (tech_spec Section 3.4.5에서 발췌):**
```matlab
function [ex, ey, Flag_VTP, theta_next] = findVTP(IMG_eroded, theta_prev, ...
    MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, COG_X, COG_Y, ...
    FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH)
%findVTP  Arc Mask 기반 VTP(Virtual Target Point) 탐색
%   IMG_eroded: logical H×W (트랙용 침식 결과)
%   theta_prev: double — 이전 VTP 방향각 [rad], NaN이면 360° 탐색
%   ex, ey: VTP와 프레임 중심의 오차 [pixel]
%   Flag_VTP: boolean — VTP 감지 여부
%   theta_next: double — 갱신된 VTP 방향각 [rad]
%
% 참조: tech_spec.md Section 3.4.1~3.4.5
% 파라미터 출처: variables.m:22-31

    persistent arc_distances arc_angles
    if isempty(arc_distances)
        [M, N] = meshgrid(1:FRAME_SIZE_WIDTH, 1:FRAME_SIZE_HEIGHT);
        arc_distances = sqrt((N - COG_X).^2 + (M - COG_Y).^2);
        arc_angles = atan2(N - COG_X, M - COG_Y);  % atan2(Δrow, Δcol)
    end

    % 반경 조건: 27 ≤ dist ≤ 28
    radius_mask = (arc_distances >= MIN_RADIUS_CROWN) & ...
                  (arc_distances <= MAX_RADIUS_CROWN);

    % 각도 조건
    if isnan(theta_prev)
        angle_mask = true(FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH);
    else
        half_fov = pi / FOV;  % π/2.9 ≈ 1.08 rad ≈ 62°
        angle_diff = mod(arc_angles - theta_prev + pi, 2*pi) - pi;
        angle_mask = abs(angle_diff) <= half_fov;
    end

    % Arc Mask 적용
    arc_mask = radius_mask & angle_mask;
    masked = IMG_eroded & arc_mask;

    % VTP 추출: 양성 픽셀의 산술 평균
    [vtp_rows, vtp_cols] = find(masked);
    if numel(vtp_rows) > 0
        x_VTP = mean(vtp_rows);
        y_VTP = mean(vtp_cols);
        ex = x_VTP - COG_X;
        ey = y_VTP - COG_Y;
        theta_next = atan2(ex, ey);  % 새 VTP 방향으로 갱신
        Flag_VTP = true;
    else
        ex = 0;
        ey = 0;
        theta_next = theta_prev;  % 이전 방향 유지
        Flag_VTP = false;
    end
end
```

**파라미터 (variables.m:22-31에서 확인):**
```matlab
FOV = 2.9;              % 시야각 = 2π/FOV ≈ 124° (half_fov ≈ 62°)
MIN_RADIUS_CROWN = 27;  % Arc Mask 내경 [pixel]
MAX_RADIUS_CROWN = 28;  % Arc Mask 외경 [pixel]
COG_X = 60;             % 프레임 중심 행 (= FRAME_SIZE_HEIGHT/2)
COG_Y = 80;             % 프레임 중심 열 (= FRAME_SIZE_WIDTH/2)
```

**atan2 좌표계 규약 (중요):**
- `atan2(Δrow, Δcol)` — 첫 인자가 행 차이, 두 번째가 열 차이
- angle=0: 오른쪽, angle=π/2: 아래쪽, 양수 방향: 시계 방향 (이미지 좌표계)

**완료 조건:** 함수 생성, theta_prev=NaN일 때 360° 탐색 동작 확인

---

### Step 4: IPS — 착륙 마커 감지 함수 작성 (findMarker)
- [x] **`UniSannio_Team/IPS/findMarker.m` 생성**

**참조:** `docs/Phase1/tech_spec.md` Section 3.5

**수정/생성 대상:**
- 새 파일: `UniSannio_Team/IPS/findMarker.m`
- 의존: Step 2의 출력 `IMG_marker_eroded`

**구현 내용:**
원형 침식 후 잔존 픽셀이 최소 기준(MARKER_MIN_PIXELS=10) 이상이면 착륙 마커로 판정.
잔존 픽셀의 무게중심을 마커 위치로 사용한다.

**코드 스니펫 (tech_spec Section 3.5에서 발췌):**
```matlab
function [ex, ey, Flag_marker] = findMarker(IMG_marker_eroded, ...
    COG_X, COG_Y, MARKER_MIN_PIXELS)
%findMarker  원형 침식 후 착륙 마커 감지
%   IMG_marker_eroded: logical H×W (원형 침식 결과)
%   ex, ey: 마커와 프레임 중심의 오차 [pixel]
%   Flag_marker: boolean — 마커 감지 여부
%
% 참조: tech_spec.md Section 3.5
% MARKER_MIN_PIXELS = 10 (노이즈 blob 오인 방지, tech_spec에서 추가)
% 주의: variables.m에는 MARKER_MIN_PIXELS가 없음 → 추가 필요

    [mark_rows, mark_cols] = find(IMG_marker_eroded);

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

**추가 작업:** `variables.m`에 `MARKER_MIN_PIXELS = 10;` 추가 필요 (tech_spec Section 3.5).
현재 `variables.m`에는 이 파라미터가 없다 (Deviation D2).

```matlab
% variables.m에 추가할 내용 (DISK_KERNEL = 11; 아래에):
MARKER_MIN_PIXELS = 10;   % 마커 판정 최소 픽셀 수 (tech_spec Section 3.5)
```

**완료 조건:** 함수 생성 + variables.m에 MARKER_MIN_PIXELS 추가

---

### Step 5: IPS — 통합 함수 작성 (IPS 메인)
- [x] **`UniSannio_Team/IPS/IPS.m` 생성**

**참조:** `docs/Phase1/tech_spec.md` Section 3.6

**수정/생성 대상:**
- 새 파일: `UniSannio_Team/IPS/IPS.m`
- 의존: Step 1~4의 모든 함수

**구현 내용:**
Step 1~5를 하나의 파이프라인으로 통합한다. persistent 변수 `theta_prev`를 관리.
**Flag 상호배타성:** VTP가 감지되면 마커 플래그를 억제 (동시 true 불가).

**코드 스니펫 (tech_spec Section 3.6에서 발췌):**
```matlab
function [ex, ey, Flag_VTP, Flag_marker] = IPS(IMG)
%IPS  Image Processing System — 카메라 프레임 → 오차/플래그 출력
%   IMG: uint8 H×W×3 RGB 프레임 (T_IPS = 0.2s, 5Hz)
%   ex, ey: double — 행/열 방향 오차 [pixel]
%   Flag_VTP: boolean — 트랙(VTP) 감지 여부
%   Flag_marker: boolean — 착륙 마커 감지 여부
%
% 참조: tech_spec.md Section 3.6
% 내부 상태: theta_prev (persistent, NaN으로 초기화)

    persistent theta_prev
    if isempty(theta_prev)
        theta_prev = NaN;  % 초기화: 360° 전체 탐색 모드
    end

    % Step 1+2: 채널 변환 + 이진화
    Fbin = channelConvertAndBinarize(IMG, G_B_GAIN, BINARIZER_THRESHOLD);

    % Step 3a: 트랙용 침식 (정사각형)
    [IMG_eroded, IMG_marker_eroded] = erodeTrackAndMarker(Fbin, ...
        SQUARE_KERNEL, DISK_KERNEL);

    % Step 4: VTP 탐색 (Arc Mask)
    [ex_vtp, ey_vtp, Flag_VTP, theta_next] = findVTP(IMG_eroded, ...
        theta_prev, MIN_RADIUS_CROWN, MAX_RADIUS_CROWN, FOV, ...
        COG_X, COG_Y, FRAME_SIZE_HEIGHT, FRAME_SIZE_WIDTH);

    % Step 5: 마커 감지
    [ex_mark, ey_mark, Flag_marker] = findMarker(IMG_marker_eroded, ...
        COG_X, COG_Y, MARKER_MIN_PIXELS);

    % Flag 상호배타성: VTP 우선
    if Flag_VTP
        ex = ex_vtp;
        ey = ey_vtp;
        Flag_marker = false;
        theta_prev = theta_next;
    elseif Flag_marker
        ex = ex_mark;
        ey = ey_mark;
        Flag_VTP = false;
        % theta_prev 갱신 안 함 (마커 추종 시 방향 무관)
    else
        ex = 0;
        ey = 0;
        % theta_prev 유지 (유실 시 마지막 방향 보존)
    end
end
```

**검증 방법:**
- 빨간 트랙 이미지 → Flag_VTP=true, ex/ey 값 ≠ 0
- 빨간 원형 마커만 있는 이미지 → Flag_marker=true
- 배경만 있는 이미지 → Flag_VTP=false, Flag_marker=false
- Flag_VTP와 Flag_marker가 동시에 true가 되지 않는지 확인

**완료 조건:** IPS 통합 함수 생성, 상호배타성 로직 포함

---

### Step 6: PP — State Machine + TAKEOFF 상태 구현
- [x] **`UniSannio_Team/PP/PathPlanner.m` 생성 (TAKEOFF 상태만 우선 구현)**

**참조:** `docs/Phase1/tech_spec.md` Section 2.1, 2.2 (S1→S2 전이), Section 4.5

**수정/생성 대상:**
- 새 파일: `UniSannio_Team/PP/PathPlanner.m`

**구현 내용:**
4-상태 State Machine의 기본 골격 + TAKEOFF(S1) 상태를 구현한다.

**State Machine 구조 (tech_spec Section 2):**
```
S1(TAKEOFF) → hovering_ok ∧ Flag_VTP → S2(FOLLOWING)
S2(FOLLOWING) → ¬Flag_VTP ∧ Flag_marker → S3(END_MARKER)
S3(END_MARKER) → centered_ok → S4(LANDING)
```

**TAKEOFF 전이 조건:**
```
hovering_ok = (Z_HIGH ≤ current_z ≤ Z_LOW) ∧ (hover_counter ≥ 40)
→ Z_HIGH=-1.2, Z_LOW=-1 (NED, 음수가 위)
→ HOVER_CONFIRM_FRAMES = T_IPS/T_PP = 0.2/0.005 = 40
```

**코드 스니펫 (PP 골격 + TAKEOFF):**
```matlab
function [xw, yw, zw] = PathPlanner(ex, ey, Flag_VTP, Flag_marker, ...
                                     current_z, ips_updated)
%PathPlanner  Path Planner — 오차 → 위치 명령
%   200Hz (T_PP=0.005s)로 실행. IPS는 5Hz (T_IPS=0.2s).
%
% 참조: tech_spec.md Section 4.5
% persistent 변수 초기값: tech_spec Section 1.3 PP 블록 참조

    persistent x_prev y_prev mission_state hover_counter center_counter
    persistent deriv_hold_counter track_lost_counter
    persistent ex_prev_pp ey_prev_pp delta_ex delta_ey

    if isempty(mission_state)
        mission_state = uint8(1);  % S1: TAKEOFF
        x_prev = 0;  y_prev = 0;  % init.posNED에서 설정 (startVars.m:39)
        hover_counter = uint32(0);
        center_counter = uint32(0);
        deriv_hold_counter = uint32(0);
        track_lost_counter = uint32(0);
        ex_prev_pp = 0;  ey_prev_pp = 0;
        delta_ex = 0;  delta_ey = 0;
    end

    switch mission_state
        case 1  % TAKEOFF
            xw = x_prev;
            yw = y_prev;
            zw = Z_LOW;  % = -1 (목표 고도)
            % Z_HIGH(-1.2) ≤ current_z ≤ Z_LOW(-1): 고도 범위 내 확인
            if (current_z >= Z_HIGH) && (current_z <= Z_LOW) && Flag_VTP
                hover_counter = hover_counter + 1;
                if hover_counter >= uint32(40)  % HOVER_CONFIRM_FRAMES
                    mission_state = uint8(2);  % → S2: FOLLOWING
                end
            else
                hover_counter = uint32(0);
            end

        case 2  % FOLLOWING (Step 7에서 구현)
            xw = x_prev; yw = y_prev; zw = Z_LOW;

        case 3  % END_MARKER (Step 8에서 구현)
            xw = x_prev; yw = y_prev; zw = Z_LOW;

        case 4  % LANDING (Step 8에서 구현)
            xw = x_prev; yw = y_prev; zw = 0;
    end

    x_prev = xw;
    y_prev = yw;
end
```

**파라미터 (variables.m:49-50, startVars.m:28,39):**
```matlab
Z_LOW = -1;      % 호버링 고도 하한 [m, NED]
Z_HIGH = -1.2;   % 호버링 고도 상한 [m, NED]
Ts = 0.005;      % PP 주기 (startVars.m:28)
% init.posNED = [57 95 -0.046]  (startVars.m:39)
```

**완료 조건:** PP 골격 생성, TAKEOFF → FOLLOWING 전이 로직 구현, 다른 상태는 placeholder

---

### Step 7: PP — FOLLOWING 상태 + 미분 제어 구현
- [x] **`UniSannio_Team/PP/PathPlanner.m`의 case 2 (FOLLOWING) 완성**

**참조:** `docs/Phase1/tech_spec.md` Section 4.2 (Following), 4.4 (Derivative), 2.3 (트랙 유실)

**수정 대상:**
- 기존 파일: `UniSannio_Team/PP/PathPlanner.m` — case 2 블록 교체

**구현 내용:**

1. **기본 추종:** `x_{k+1} = x_k + GAIN_TRACK × ex` (tech_spec 4.2)
2. **미분 제어:** IPS 갱신 시점(ips_updated=true)에서만 차분 계산,
   `Δe_norm > CHANGE_DERIVATIVE_ERROR_THRESHOLD`이면 40스텝간 미분항 적용 (tech_spec 4.4)
3. **트랙 유실:** Flag_VTP=false ∧ Flag_marker=false → 위치 유지, lost_counter 증가 (tech_spec 2.3)
4. **S2→S3 전이:** ¬Flag_VTP ∧ Flag_marker → mission_state=3 (tech_spec 2.2)

**코드 스니펫 (case 2 교체 내용):**
```matlab
        case 2  % FOLLOWING
            % 미분 계산 (IPS 갱신 시점에서만, 5Hz)
            if ips_updated
                delta_ex = ex - ex_prev_pp;
                delta_ey = ey - ey_prev_pp;
                delta_norm = sqrt(delta_ex^2 + delta_ey^2);
                if delta_norm > CHANGE_DERIVATIVE_ERROR_THRESHOLD  % > 2 pixel
                    deriv_hold_counter = uint32(TIME_HOLD / Ts);   % 0.2/0.005 = 40
                end
                ex_prev_pp = ex;
                ey_prev_pp = ey;
            end

            if Flag_VTP
                track_lost_counter = uint32(0);
                xw = x_prev + GAIN_TRACK * ex;       % 0.0038 × ex
                yw = y_prev + GAIN_TRACK * ey;       % 0.0038 × ey
                % 미분항 적용 (급커브 오버슈트 억제)
                if deriv_hold_counter > 0
                    xw = xw + DERIVATIVE_GAIN * delta_ex;  % 0.002 × Δex
                    yw = yw + DERIVATIVE_GAIN * delta_ey;  % 0.002 × Δey
                    deriv_hold_counter = deriv_hold_counter - 1;
                end
                zw = Z_LOW;
            elseif Flag_marker
                % S2 → S3: 트랙 끝, 마커 감지
                mission_state = uint8(3);
                center_counter = uint32(0);
                xw = x_prev + GAIN_LANDING * ex;     % 0.0022 × ex
                yw = y_prev + GAIN_LANDING * ey;
                zw = Z_LOW;
            else
                % 트랙 유실: 위치 유지
                track_lost_counter = track_lost_counter + 1;
                xw = x_prev;
                yw = y_prev;
                zw = Z_LOW;
            end
```

**파라미터 (variables.m:43-56):**
```matlab
GAIN_TRACK = 0.0038;                      % 추종 게인
GAIN_LANDING = 0.0022;                    % 착륙 게인
CHANGE_DERIVATIVE_ERROR_THRESHOLD = 2;    % 미분 활성 임계값 [pixel]
DERIVATIVE_GAIN = 0.002;                  % 미분 게인
TIME_HOLD = 0.2;                          % 미분 유지 시간 [s]
```

**완료 조건:** FOLLOWING 상태 완성, 미분 제어 포함, 트랙 유실 처리 포함

---

### Step 8: PP — END_MARKER + LANDING 상태 구현
- [x] **`UniSannio_Team/PP/PathPlanner.m`의 case 3, 4 완성**

**참조:** `docs/Phase1/tech_spec.md` Section 4.3 (End-Marker), Section 2.2 (S3→S4)

**수정 대상:**
- 기존 파일: `UniSannio_Team/PP/PathPlanner.m` — case 3, 4 블록 교체

**구현 내용:**

1. **END_MARKER (S3):** 마커 중심 정렬
   - `error_norm ≤ MAX_ERROR_LANDING(=5)` → center_counter 증가, 위치 고정
   - `error_norm > 5` → counter 리셋, GAIN_LANDING으로 재접근
   - `center_counter ≥ CENTER_CONFIRM_FRAMES(=800)` → S4 전이 (4초간 오차 5px 이하 유지)

2. **LANDING (S4):** `zw = 0` (착륙 명령), 수평 위치 고정

**코드 스니펫 (case 3, 4 교체 내용):**
```matlab
        case 3  % END_MARKER: 마커 중심 정렬
            error_norm = sqrt(ex^2 + ey^2);
            if error_norm <= MAX_ERROR_LANDING  % ≤ 5 pixel
                center_counter = center_counter + 1;
                xw = x_prev;  % 수평 위치 고정
                yw = y_prev;
                zw = Z_LOW;   % 아직 착륙 아님, 고도 유지
                % CENTER_CONFIRM_FRAMES = DELAY_LANDING / Ts = 4/0.005 = 800
                if center_counter >= uint32(DELAY_LANDING / Ts)
                    mission_state = uint8(4);  % → S4: LANDING
                end
            else
                center_counter = uint32(0);  % 오차 초과 시 카운터 리셋
                xw = x_prev + GAIN_LANDING * ex;
                yw = y_prev + GAIN_LANDING * ey;
                zw = Z_LOW;
            end

        case 4  % LANDING: 하강 착륙
            xw = x_prev;   % 수평 위치 고정
            yw = y_prev;
            zw = 0;         % NED z=0 → 지면 착륙 명령
```

**파라미터 (variables.m:60-62, startVars.m:28):**
```matlab
DELAY_LANDING = 4;          % 착륙 확인 지연 [s]
MAX_ERROR_LANDING = 5;      % 착륙 오차 한계 [pixel]
Ts = 0.005;                 % PP 주기 → CENTER_CONFIRM_FRAMES = 4/0.005 = 800
```

**완료 조건:** END_MARKER 정렬 로직 + LANDING 착륙 명령 구현

---

### Step 9: Simulink 통합 — flightControlSystem.slx에 IPS/PP 블록 삽입
- [x] **`UniSannio_Team/controller/flightControlSystem.slx` 수정** (통합 가이드 스크립트 `utilities/integrateFCS.m` 작성)

**참조:** `docs/Phase1/tech_spec.md` Section 1.1 (전체 구조), 1.3 (블록 인터페이스), 1.4 (동작 주기)

**수정 대상:**
- `UniSannio_Team/controller/flightControlSystem.slx` — IPS/PP MATLAB Function 블록 추가

**구현 내용:**

이 단계는 Simulink GUI에서 수행해야 하는 작업이 대부분이다.
코드로 자동화할 수 없는 부분이 있으므로, **수동 작업 가이드**를 포함한다.

**블록 구성:**
```
[Camera Input] ──(uint8 H×W×3)──▶ [IPS MATLAB Function Block]
                                    │  @ T_IPS = 0.2s (Rate Transition)
                                    │
                                    ├─ ex (double)
                                    ├─ ey (double)
                                    ├─ Flag_VTP (boolean)
                                    └─ Flag_marker (boolean)
                                          │ (Zero-Order Hold)
                                          ▼
                                   [PP MATLAB Function Block]
                                    │  @ T_PP = 0.005s
                                    │
                                    ├─ xw (double) ──▶ [Position Controller]
                                    ├─ yw (double) ──▶
                                    └─ zw (double) ──▶
```

**수동 작업 체크리스트:**
1. `flightControlSystem.slx` 열기
2. MATLAB Function 블록 2개 추가: `IPS`, `PathPlanner`
3. IPS 블록: Sample Time = `VTs` (0.2), 입력 = IMG, 출력 = ex, ey, Flag_VTP, Flag_marker
4. PP 블록: Sample Time = `Ts` (0.005), 입력 = ex, ey, Flag_VTP, Flag_marker, current_z, ips_updated
5. IPS→PP 사이에 Rate Transition 블록 (Zero-Order Hold) 삽입
6. PP 출력을 기존 위치 제어기 입력에 연결
7. `ips_updated` 신호: IPS 출력이 갱신되었는지 PP에서 감지 (Rate Transition의 출력 변화 감지)

**Simulink 스크립트 대안 (프로그래밍 방식):**
```matlab
% 블록 추가 자동화 (MATLAB 명령창에서 실행)
open_system('flightControlSystem');
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    'flightControlSystem/IPS');
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    'flightControlSystem/PathPlanner');
% 이후 블록 파라미터 및 연결은 수동으로 설정
```

**코드 생성 제약 확인 (tech_spec 부록 B):**
- `strel`, `imerode` → 코드 생성 미지원 시 수동 erosion 구현 필요
- `cell`, 가변 배열, `eval`, `try-catch` 사용 금지

**완료 조건:** Simulink 모델에 IPS/PP 블록이 삽입되고 신호 연결 완료, 빌드 에러 0건

---

### Step 10: 단위 테스트 작성
- [x] **`UniSannio_Team/tests/` 아래 테스트 스크립트 작성**

**참조:** `docs/Phase1/tech_spec.md` Section 9 (Acceptance Criteria)

**수정/생성 대상:**
- 새 파일: `UniSannio_Team/tests/test_IPS.m`
- 새 파일: `UniSannio_Team/tests/test_PP.m`

**구현 내용:**

각 모듈의 입출력을 합성 데이터로 검증한다.

**test_IPS.m 테스트 케이스:**
```matlab
%% Test 1: 빨간 트랙 이미지 → Flag_VTP = true
IMG_red_track = uint8(zeros(120, 160, 3));
% 중심에서 오른쪽으로 27~28 pixel 거리에 빨간 호 생성
for n = 1:120
    for m = 1:160
        dist = sqrt((n-60)^2 + (m-80)^2);
        if dist >= 27 && dist <= 28 && m > 80  % 오른쪽 반원
            IMG_red_track(n, m, 1) = 255;  % Red만 높게
        end
    end
end
[ex, ey, Flag_VTP, Flag_marker] = IPS(IMG_red_track);
assert(Flag_VTP == true, 'VTP should be detected on red track');
assert(Flag_marker == false, 'Marker should not be detected');
assert(ey > 0, 'VTP should be to the right (positive ey)');

%% Test 2: 빨간 원형 마커 → Flag_marker = true
IMG_marker = uint8(zeros(120, 160, 3));
% 중심 근처에 큰 빨간 원 (반지름 15 pixel)
for n = 1:120
    for m = 1:160
        if sqrt((n-60)^2 + (m-80)^2) <= 15
            IMG_marker(n, m, 1) = 255;
        end
    end
end
% theta_prev 리셋을 위해 persistent 초기화 필요 → clear IPS 또는 새 함수 호출
[ex, ey, Flag_VTP, Flag_marker] = IPS(IMG_marker);
% 마커는 침식 후에도 남아야 함 (반지름 15 > DISK_KERNEL 11)
assert(Flag_marker == true, 'Marker should be detected');

%% Test 3: 빈 이미지 → 모두 false
IMG_empty = uint8(128 * ones(120, 160, 3));  % 회색 균일
[ex, ey, Flag_VTP, Flag_marker] = IPS(IMG_empty);
assert(Flag_VTP == false && Flag_marker == false, 'No detection on grey image');
```

**test_PP.m 테스트 케이스:**
```matlab
%% Test 1: TAKEOFF → FOLLOWING 전이
% hover_counter가 40에 도달하면 state 2로 전이
for i = 1:50
    [xw, yw, zw] = PathPlanner(5, 5, true, false, -1.1, false);
end
% 50스텝 후 FOLLOWING 상태에서 위치 갱신 시작 확인
% (x_prev가 더 이상 고정되지 않음)

%% Test 2: FOLLOWING 모드에서 위치 갱신
% ex=10, ey=5 → xw += 0.0038*10, yw += 0.0038*5
[xw1, ~, ~] = PathPlanner(10, 5, true, false, -1.1, false);
[xw2, ~, ~] = PathPlanner(10, 5, true, false, -1.1, false);
assert(xw2 > xw1, 'Position should increase with positive ex');

%% Test 3: LANDING 상태에서 zw = 0
% (state를 4로 만든 후) zw == 0 확인
```

**완료 조건:** IPS/PP 단위 테스트 작성, 주요 경로(트랙 감지, 마커 감지, 상태 전이) 커버

---

### Step 11: 2026 환경 검증 및 파라미터 재튜닝
- [ ] **2026 모델 설치 후 파라미터 검증/재조정**

**참조:** `docs/Phase1/tech_spec.md` Section 7.2, 7.3

**수정 대상:**
- `UniSannio_Team/variables.m` — 파라미터 값 조정
- `UniSannio_Team/utilities/startVars.m` — 필요 시 초기 조건 조정

**재튜닝 순서 (tech_spec Section 7.3):**

| 순서 | 항목 | 확인 방법 | 영향 파라미터 |
|------|------|----------|-------------|
| 1 | 프레임 크기 확인 | 카메라 출력 size() | COG_X, COG_Y, FRAME_SIZE_* |
| 2 | 트랙 RGB 분포 확인 | 기본 날씨 프레임 캡처 | G_B_GAIN |
| 3 | BINARIZER_THRESHOLD | 이진화 결과 육안 확인 | BINARIZER_THRESHOLD |
| 4 | 트랙 폭 → 침식 커널 | sweep {8,12,16,20} | SQUARE_KERNEL |
| 5 | Arc Mask 반경 | 프레임 크기 비례 조정 | MIN/MAX_RADIUS_CROWN |
| 6 | 마커 크기 → 침식 커널 | sweep {11,15,19,23} | DISK_KERNEL, MARKER_MIN_PIXELS |
| 7 | 게인 조정 | 속도-정확도 트레이드오프 | GAIN_TRACK, GAIN_LANDING |
| 8 | Simple 트랙 통과 | 전체 시뮬레이션 | 종합 |

**주의:** 이 단계는 2026 대회 모델(Unreal Engine 기반)이 설치된 후에만 수행 가능.
모델 미설치 시 이 항목은 건너뛰고 Step 12로 진행.

**완료 조건:** 각 파라미터 재조정 완료, 변경 사항 variables.m에 반영 및 커밋

---

### Step 12: Acceptance Test 수행
- [ ] **5개 Simple 트랙에서 통과 테스트**

**참조:** `docs/Phase1/tech_spec.md` Section 9 (Acceptance Criteria)

**테스트 규격 (tech_spec Section 9.2):**
| 트랙 | 설명 |
|------|------|
| T1 | 4세그먼트, 완만한 각도 (120°~240°) |
| T2 | 6세그먼트, 혼합 각도 |
| T3 | 4세그먼트, 급커브 포함 (30° 또는 330°) |
| T4 | 5세그먼트, 긴 직선 + 급커브 |
| T5 | 6세그먼트, 지그재그 |

**합격 기준:**
| ID | 항목 | 기준 |
|----|------|------|
| A1 | 코드 생성 | Build 에러 0건 |
| A2 | Simple 트랙 완주 | 5개 중 4개 이상 성공 |
| A3 | 마커 착륙 | A2 성공 트랙에서 마커 위 착륙 |
| A4 | 시뮬 시간 | 100초 이내 |
| A5 | 반복 안정성 | 동일 트랙 3회 연속 성공 |

**실패 모드 기록 (필수, tech_spec Section 9.3):**
| 기록 항목 | 형식 |
|----------|------|
| 트랙 유실 빈도 | 트랙별 lost_counter 최대값 |
| 이진화 실패 사례 | 스크린샷 (이진화 전/후) |
| 날씨 변동 시 동작 | 태양 0°/180°, 적설 0%/50% |
| 교차 트랙 실패 | Advanced 트랙 1개, 실패 지점 기록 |

**완료 조건:** A1~A5 합격, 실패 모드 기록 완료 → **Phase 1 완료**

---

## 컨텍스트 소실 안전성 점검

각 Step이 새 세션에서 독립적으로 수행 가능한지 점검한 결과:

| Step | 필요한 선행 지식 | 새 세션에서 복구 방법 |
|------|----------------|-------------------|
| 1 | tech_spec Section 3.1-3.2 | plan.md의 코드 스니펫 + variables.m 읽기 |
| 2 | Step 1의 출력 형식 (Fbin) | plan.md의 Step 1 코드 스니펫에서 확인 |
| 3 | Step 2의 출력 (IMG_eroded) + Arc Mask 개념 | plan.md의 상세 코드 스니펫 + tech_spec 3.4 |
| 4 | Step 2의 출력 (IMG_marker_eroded) | plan.md의 Step 2 코드 스니펫에서 확인 |
| 5 | Step 1~4의 함수 시그니처 | 각 .m 파일의 함수 선언 1줄 읽기 |
| 6 | IPS 출력 형식 (ex, ey, Flags) | plan.md의 Step 5 또는 tech_spec 1.3 |
| 7 | Step 6의 PP 골격 + persistent 변수 목록 | PathPlanner.m 읽기 (Step 6에서 생성) |
| 8 | Step 6~7의 PP 골격 | PathPlanner.m 읽기 |
| 9 | Step 1~8의 모든 함수 + Simulink 구조 | plan.md의 블록 다이어그램 + tech_spec 1.1 |
| 10 | Step 1~8의 함수 시그니처 | 각 .m 파일 읽기 |
| 11 | 전체 시스템 파라미터 | variables.m + plan.md Step 11 표 |
| 12 | 전체 시스템 동작 | Simulink 모델 실행 |

**결론:** 모든 Step은 `plan.md` + 해당 Step에 명시된 파일만 읽으면 수행 가능.
Step 7~8은 Step 6에서 생성한 `PathPlanner.m`을 읽어야 하므로 Step 6 완료가 전제.
Step 9는 Step 1~8이 모두 완료된 후 수행해야 한다 (Simulink 통합).
