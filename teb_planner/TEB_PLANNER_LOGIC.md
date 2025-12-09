# TEB Planner 로직 정리

## 📋 목차
1. [전체 아키텍처](#전체-아키텍처)
2. [파일 구조 및 역할](#파일-구조-및-역할)
3. [주요 로직 흐름](#주요-로직-흐름)
4. [FLC-TEB 논문 적용](#flc-teb-논문-적용)
5. [토픽 및 메시지](#토픽-및-메시지)

---

## 전체 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    spline_planner_node                       │
│  (ObstacleSpliner - 메인 노드, 20Hz 루프)                    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ├─ 장애물 필터링
                            ├─ 회피 방향 결정
                            ├─ 초기 경로 생성
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                         TEB 클래스                           │
│  - Elastic Bands 기반 경로 최적화                            │
│  - 시간 최적화 및 차량 제약 처리                             │
└─────────────────────────────────────────────────────────────┘
                            │
                            ├─ Narrowness 계산
                            ├─ Turning Complexity 계산
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    FuzzyController                          │
│  - Fuzzy Logic로 가중치 동적 조정                           │
│  - 논문의 Table 3 규칙 적용                                 │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
                    동적 가중치 적용
                            │
                            ▼
                    최적화된 경로 반환
```

---

## 파일 구조 및 역할

### 1. **spline_planner_node.cpp/hpp**
**역할**: 메인 노드, 전체 로직 오케스트레이션

**주요 기능**:
- ROS 2 노드 초기화 및 토픽 구독/발행
- 20Hz 메인 루프 (`spliner_loop`)
- 장애물 필터링 및 회피 방향 결정
- TEB 객체 생성 및 FLC 통합
- Frenet ↔ Cartesian 좌표 변환
- Waypoint 및 마커 생성

**핵심 함수**:
- `spliner_loop()`: 메인 루프 (20Hz)
- `do_spline()`: 회피 경로 생성 로직
- `obs_filtering()`: 장애물 필터링 (lookahead 거리 내)
- `more_space()`: 회피 방향 결정 (left/right)
- `get_initial_path_from_global()`: 전역 경로에서 초기 경로 추출

---

### 2. **teb.cpp/hpp**
**역할**: TEB (Timed Elastic Band) 알고리즘 구현

**주요 기능**:
- Elastic Bands 기반 경로 최적화
- 시간 최적화 (Time Optimization)
- 차량 제약 처리 (속도, 가속도, 회전 반경)
- **FLC-TEB 추가 Objectives**:
  - Smoothness Objective (Equation 10)
  - Jerk Objective (Equation 11)
- Narrowness 계산 (Equation 13)
- Turning Complexity 계산 (Equation 14)

**핵심 함수**:
- `optimize()`: 경로 최적화 반복 수행
- `update_poses()`: 각 포즈에 힘 적용하여 업데이트
- `compute_narrowness()`: 경로의 좁음 정도 계산
- `compute_turning_complexity()`: 회전 복잡도 계산
- `smoothness_objective()`: Smoothness 비용 계산
- `jerk_objective()`: Jerk 비용 계산
- `set_weights()`: FLC에서 계산된 가중치 적용

**TEB Objectives**:
1. **장애물 거리** (`kr_repulsive_`): 장애물과의 거리 최대화
2. **속도 제약** (`kv_`): 최대 속도 제한
3. **가속도 제약** (`ka_`): 최대 가속도 제한
4. **비홀로노믹 제약** (`knh_`): 차량 운동학 제약
5. **회전 반경 제약** (`kr_radius_`): 최소 회전 반경
6. **시간 최적화** (`kt_`): 시간 최소화
7. **Smoothness** (`ksm_`): 경로 부드러움 (FLC-TEB 추가)
8. **Jerk** (`kjerk_`): 가속도 변화율 (FLC-TEB 추가)

---

### 3. **fuzzy_controller.cpp/hpp**
**역할**: Fuzzy Logic Controller 구현 (논문의 FLC)

**주요 기능**:
- Narrowness와 Turning Complexity를 입력으로 받음
- Fuzzy Membership Functions 적용
- Fuzzy Rules (Table 3) 적용
- Defuzzification (중심값 방법)
- 가중치 매핑 (Equation 15)

**핵심 함수**:
- `compute_weights()`: Narrowness, Turning Complexity → 가중치
- `membership_narrowness()`: Narrowness membership function
- `membership_turning_complexity()`: Turning Complexity membership function
- `apply_fuzzy_rules()`: Table 3 규칙 적용
- `defuzzify()`: Fuzzy 출력을 crisp 값으로 변환

**Fuzzy Sets**:
- **입력**:
  - Narrowness: [S, M, L] (Small, Middle, Large)
  - Turning Complexity: [S, M, L]
- **출력**:
  - Obstacle weight: [VL, L, M, H, VH]
  - Smoothness/Velocity/Acceleration/Jerk: [VL, L, M, H, VH]

---

## 주요 로직 흐름

### 1. 초기화 단계
```
ObstacleSpliner 생성자
  ├─ ROS 2 파라미터 선언
  ├─ 토픽 구독 설정:
  │   ├─ /perception/obstacles
  │   ├─ /car_state/odom_frenet
  │   ├─ /global_waypoints
  │   └─ /global_waypoints_scaled
  ├─ 토픽 발행 설정:
  │   ├─ /planner/avoidance/otwpnts
  │   ├─ /planner/avoidance/markers
  │   └─ ...
  ├─ 메시지 대기 (wait_for_messages)
  ├─ Frenet 변환기 초기화
  └─ 20Hz 타이머 시작
```

### 2. 메인 루프 (spliner_loop, 20Hz)
```
spliner_loop()
  ├─ 글로벌 경로 확인
  ├─ 장애물이 있는가?
  │   ├─ YES → do_spline() 호출
  │   └─ NO  → 마커 삭제 (DELETEALL)
  ├─ Waypoint 및 마커 발행
  └─ 반복
```

### 3. 회피 경로 생성 (do_spline)
```
do_spline(obstacles, gb_wpnts)
  │
  ├─ 1. 장애물 필터링
  │   └─ obs_filtering():
  │       ├─ 트랙 근처 장애물만 선택 (obs_traj_tresh_)
  │       └─ Lookahead 거리 내 장애물만 선택 (lookahead_)
  │
  ├─ 2. 가장 가까운 장애물 찾기
  │
  ├─ 3. Apex 계산
  │   └─ 장애물 중심 위치 (s_apex)
  │
  ├─ 4. 회피 방향 결정
  │   ├─ Outside 방향 계산 (kappa 기반)
  │   └─ more_space(): left 또는 right 선택
  │
  ├─ 5. 회피 가능 여부 체크
  │   └─ check_ot_side_possible()
  │       ├─ 가능 → 계속
  │       └─ 불가능 → danger_flag = true
  │
  ├─ 6. TEB 경로 생성 (danger_flag == false)
  │   │
  │   ├─ 6.1. 초기 경로 생성
  │   │   └─ get_initial_path_from_global():
  │   │       └─ 전역 경로에서 장애물 주변 경로 추출
  │   │
  │   ├─ 6.2. 장애물 포인트 변환
  │   │   └─ convert_obstacle_to_points():
  │   │       └─ 장애물을 2D 포인트로 변환
  │   │
  │   ├─ 6.3. TEB 객체 생성
  │   │   └─ TEB(initial_path, obstacles, v_max, a_max, rho_min, wheelbase)
  │   │
  │   ├─ 6.4. FLC-TEB 적용
  │   │   ├─ Narrowness 계산 (Equation 13)
  │   │   │   └─ N_t = d_obs / ρ_min
  │   │   ├─ Turning Complexity 계산 (Equation 14)
  │   │   │   └─ T_c = (n_turn + 2*n_sharp) / (n-1)
  │   │   ├─ FuzzyController로 가중치 계산
  │   │   │   └─ compute_weights(narrowness, turning_complexity)
  │   │   └─ TEB에 가중치 적용
  │   │       └─ set_weights(σ_obs, σ_sm, σ_v, σ_acc, σ_j)
  │   │
  │   ├─ 6.5. TEB 최적화
  │   │   └─ teb.optimize(10): 10번 반복 최적화
  │   │
  │   ├─ 6.6. 경로 변환
  │   │   ├─ get_path_with_velocities(): 최적화된 경로 가져오기
  │   │   ├─ Cartesian → Frenet 변환
  │   │   ├─ Track bounds 체크
  │   │   │   └─ 경계 너무 가까우면 danger_flag = true
  │   │   └─ Waypoint 생성
  │   │
  │   └─ 6.7. 마커 생성
  │       ├─ CYLINDER 마커 (각 waypoint)
  │       └─ LINE_STRIP 마커 (연두색 선)
  │
  ├─ 7. Fallback 처리
  │   ├─ danger_flag == true → 글로벌 경로 사용 (속도 70%)
  │   └─ wpnts.empty() → 글로벌 경로 사용 (속도 80%)
  │
  └─ 8. Waypoint 및 마커 반환
```

### 4. TEB 최적화 과정 (optimize)
```
optimize(max_iter)
  │
  └─ for (iter = 0; iter < max_iter; iter++)
      │
      ├─ 1. 각 포즈에 힘 계산
      │   └─ update_poses():
      │       ├─ 장애물 반발력 (kr_repulsive_)
      │       ├─ 속도 제약 (kv_)
      │       ├─ 가속도 제약 (ka_)
      │       ├─ 비홀로노믹 제약 (knh_)
      │       ├─ 회전 반경 제약 (kr_radius_)
      │       ├─ 시간 최적화 (kt_)
      │       ├─ Smoothness (ksm_) ← FLC-TEB 추가
      │       └─ Jerk (kjerk_) ← FLC-TEB 추가
      │
      ├─ 2. 포즈 업데이트
      │   └─ 힘에 따라 위치 및 방향 조정
      │
      ├─ 3. 포즈 간격 조정
      │   └─ maintain_overlap(): 중복 포즈 제거
      │
      └─ 4. 반복
```

---

## FLC-TEB 논문 적용

### 논문: "Improvement of the TEB Algorithm for Local Path Planning of Car-like Mobile Robots Based on Fuzzy Logic Control"

### 1. 추가된 Objectives

#### Smoothness Objective (Equation 10)
```cpp
// teb.cpp의 smoothness_objective()
f_sm = Σ_{i=1}^{n-1} (Δβ_{i+1} - Δβ_i)²
```
- **목적**: 연속된 포즈 간 각도 변화를 최소화하여 경로 부드러움 향상
- **구현**: `update_poses()`에서 각 포즈의 각도 변화를 계산하여 힘으로 적용

#### Jerk Objective (Equation 11)
```cpp
// teb.cpp의 jerk_objective()
f_jerk = Σ_{i=1}^{n-3} |(a_{i+1} - a_i) / (ΔT_i/4 + ΔT_{i+1}/2 + ΔT_{i+2}/4)|
```
- **목적**: 가속도 변화율을 최소화하여 제어 안정성 향상
- **구현**: 가속도 변화를 계산하여 힘으로 적용

### 2. Fuzzy Inputs 계산

#### Narrowness (Equation 13)
```cpp
// teb.cpp의 compute_narrowness()
d_obs = (1/n) * Σ_{i=1}^n d_obs_i
N_t = d_obs / ρ_min
```
- **범위**: [0, 4]
- **의미**: 경로의 좁음 정도 (장애물까지 평균 거리 / 최소 회전 반경)
- **Fuzzy Sets**: [S (Small), M (Middle), L (Large)]

#### Turning Complexity (Equation 14)
```cpp
// teb.cpp의 compute_turning_complexity()
κ_turn = 0.2 * κ_max
κ_sharp = 0.8 * κ_max
T_c = (n_turn + 2*n_sharp) / (n-1)
```
- **범위**: [0, 2]
- **의미**: 경로의 회전 복잡도 (turning 세그먼트 비율)
- **Fuzzy Sets**: [S (Small), M (Middle), L (Large)]

### 3. Fuzzy Logic Controller

#### Membership Functions
- **Trapezoidal**: 경계 영역 (S, L)
- **Triangular**: 중간 영역 (M)

#### Fuzzy Rules (Table 3)
9개의 규칙으로 구성:
- 입력: Narrowness (S/M/L) × Turning Complexity (S/M/L)
- 출력: 각 가중치의 Fuzzy Set (VL/L/M/H/VH)

예시:
- Rule 1: IF N_t=S AND T_c=S THEN σ_obs=L, σ_sm=L, σ_v=VL, ...
- Rule 7: IF N_t=L AND T_c=S THEN σ_obs=H, σ_sm=L, ...

#### Defuzzification
- **방법**: 중심값 방법 (Centroid)
- 각 규칙의 출력을 가중 평균하여 crisp 값 계산

#### Weight Mapping (Equation 15)
```cpp
// fuzzy_controller.cpp의 map_weight()
σ' ∈ [-1, 0) → σ = 1 - σ'
σ' ∈ [0, 3]  → σ = 10^σ'
```
- **최종 범위**: [0, 1000] (장애물), [0, 100] (기타)

### 4. 동적 가중치 적용

```cpp
// spline_planner_node.cpp의 do_spline()
// 1. Narrowness와 Turning Complexity 계산
double narrowness = teb.compute_narrowness();
double turning_complexity = teb.compute_turning_complexity();

// 2. Fuzzy Controller로 가중치 계산
FuzzyController flc;
auto weights = flc.compute_weights(narrowness, turning_complexity);

// 3. TEB에 가중치 적용
teb.set_weights(weights.sigma_obs, weights.sigma_sm, 
                weights.sigma_v, weights.sigma_acc, weights.sigma_j);

// 4. 최적화 수행
teb.optimize(10);
```

### 5. 논문 적용 요약

| 논문 요소 | 구현 위치 | 상태 |
|----------|---------|------|
| Smoothness Objective (Eq. 10) | `teb.cpp::smoothness_objective()` | ✅ 구현됨 |
| Jerk Objective (Eq. 11) | `teb.cpp::jerk_objective()` | ✅ 구현됨 |
| Narrowness (Eq. 13) | `teb.cpp::compute_narrowness()` | ✅ 구현됨 |
| Turning Complexity (Eq. 14) | `teb.cpp::compute_turning_complexity()` | ✅ 구현됨 |
| Fuzzy Membership Functions | `fuzzy_controller.cpp` | ✅ 구현됨 |
| Fuzzy Rules (Table 3) | `fuzzy_controller.cpp::apply_fuzzy_rules()` | ✅ 구현됨 |
| Defuzzification | `fuzzy_controller.cpp::defuzzify()` | ✅ 구현됨 |
| Weight Mapping (Eq. 15) | `fuzzy_controller.cpp::map_weight()` | ✅ 구현됨 |
| 동적 가중치 적용 | `spline_planner_node.cpp::do_spline()` | ✅ 구현됨 |

---

## 토픽 및 메시지

### 구독 토픽
- `/perception/obstacles` (ObstacleArray): 장애물 정보
- `/car_state/odom_frenet` (Odometry): 차량 상태 (Frenet 좌표)
- `/global_waypoints` (WpntArray): 글로벌 경로
- `/global_waypoints_scaled` (WpntArray): 스케일된 글로벌 경로

### 발행 토픽
- `/planner/avoidance/otwpnts` (OTWpntArray, 20Hz): 회피 경로 waypoints
- `/planner/avoidance/markers` (MarkerArray, 20Hz): 시각화 마커
- `/planner/avoidance/considered_OBS` (Marker): 고려된 장애물 마커
- `/planner/avoidance/propagated_obs` (Marker): 예측된 장애물 마커
- `/planner/avoidance/latency` (Float32): 지연 시간 (measuring 모드)

---

## 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|---------|--------|------|
| `lookahead_` | 10.0 m | 장애물 필터링 거리 |
| `evasion_dist_` | 0.77 m | 회피 거리 (장애물로부터) |
| `obs_traj_tresh_` | 0.3 m | 트랙 근처 장애물 임계값 |
| `spline_bound_mindist_` | 0.1 m | 트랙 경계 최소 거리 |
| `fixed_pred_time_` | 0.15 s | 장애물 예측 시간 |

---

## 성능 최적화

1. **최적화 반복 횟수**: 10번 (기본 50번에서 감소)
2. **비정상 d 값 스킵**: 10m 이상의 d 값은 Frenet 변환 오류로 간주하여 스킵
3. **Fallback 경로**: TEB 실패 시 글로벌 경로 사용 (속도 감소)

---

## 현재 이슈 및 해결 방안

### 이슈 1: 비정상적인 d 값
- **증상**: Frenet 변환에서 d 값이 매우 큼 (예: -7219.681)
- **원인**: TEB 경로가 트랙 밖으로 나가거나 Frenet 변환 오류
- **해결**: d > 10m인 포인트는 스킵

### 이슈 2: Track bounds 체크로 인한 abort
- **증상**: "Evasion trajectory too close to TRACKBOUNDS" 경고 후 경로 생성 중단
- **원인**: TEB 경로가 트랙 경계를 벗어남
- **해결**: 비정상 d 값 스킵으로 개선, 추가 조정 필요

---

## 향후 개선 사항

1. **TEB 초기 경로 개선**: 트랙 경계 내에 있도록 보장
2. **Frenet 변환 검증**: 변환 결과의 유효성 검사 강화
3. **가중치 튜닝**: FLC 출력 가중치 범위 조정
4. **성능 모니터링**: 최적화 시간 및 경로 품질 메트릭 추가

