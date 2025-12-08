# FLC-TEB 논문 수식 및 규칙 정리

## 1. Smoothness Objective (Equation 10)

```
f_sm = Σ_{i=1}^{n-1} (Δβ_{i+1} - Δβ_i)²
```

- `Δβ_i`: 인접 포즈 간 각도 변화
- 목적: 연속된 포즈 간 각도 변화를 최소화하여 경로 부드러움 향상

## 2. Jerk Objective (Equation 11)

```
f_jerk = Σ_{i=1}^{n-3} |(a_{i+1} - a_i) / (ΔT_i/4 + ΔT_{i+1}/2 + ΔT_{i+2}/4)|
```

- `a_i`: i번째 포즈의 가속도
- `ΔT_i`: i번째 시간 간격
- 목적: 가속도 변화율을 최소화하여 제어 안정성 향상

## 3. Narrowness 계산 (Equation 13)

```
d_obs = (1/n) * Σ_{i=1}^n d_obs_i
N_t = d_obs / ρ_min
```

- `d_obs_i`: i번째 포즈에서 가장 가까운 장애물까지의 거리
- `ρ_min`: 최소 회전 반경
- 범위: [0, 4]
- Fuzzy set: [S (Small), M (Middle), L (Large)]

## 4. Turning Complexity 계산 (Equation 14)

```
κ_i = Δθ_i / Δs_i
κ_turn = 0.2 * κ_max
κ_sharp = 0.8 * κ_max
κ_max = 1 / ρ_min

T_c = (n_turn + 2*n_sharp) / (n-1)
```

- `κ_i`: i번째 세그먼트의 곡률
- `n_turn`: turning 세그먼트 개수 (κ_i > κ_turn)
- `n_sharp`: sharp turning 세그먼트 개수 (κ_turn < κ_i < κ_sharp)
- 범위: [0, 2]
- Fuzzy set: [S (Small), M (Middle), L (Large)]

## 5. Fuzzy Logic Controller

### 5.1 Membership Functions

#### Narrowness (N_t)
- Domain: [0, 4]
- Fuzzy sets: [S, M, L]
- Trapezoidal at boundaries, Triangular in middle

#### Turning Complexity (T_c)
- Domain: [0, 2]
- Fuzzy sets: [S, M, L]
- Trapezoidal at boundaries, Triangular in middle

#### Output Weights
- Obstacle weight (σ_obs): [-1, 3] → [0, 1000]
- Smoothness/Velocity/Acceleration/Jerk weights: [-1, 2] → [0, 100]
- Time/Path length weights: [0, 5] (not mapped)
- Fuzzy sets: [VL (Very Low), L (Low), M (Middle), H (High), VH (Very High)]

### 5.2 Fuzzy Rules (Table 3)

| No. | N_t | T_c | σ_obs | σ_sm | σ_v/σ_acc/σ_j (linear) | σ_v/σ_acc/σ_j (angular) | σ_sp | σ_to |
|-----|-----|-----|-------|------|------------------------|-------------------------|------|------|
| 1   | S   | S   | L     | L    | VL                    | VH                      | L    | L    |
| 2   | S   | M   | L     | L    | VL                    | L                       | L    | L    |
| 3   | S   | L   | L     | M    | VL                    | L                       | L    | L    |
| 4   | M   | S   | L     | VL   | H                     | L                       | M    | M    |
| 5   | M   | M   | L     | M    | L                     | M                       | L    | L    |
| 6   | M   | L   | L     | M    | L                     | M                       | L    | L    |
| 7   | L   | S   | H     | L    | L                     | L                       | L    | L    |
| 8   | L   | M   | M     | M    | VL                    | L                       | L    | L    |
| 9   | L   | L   | H     | M    | VL                    | L                       | L    | L    |

### 5.3 Weight Mapping (Equation 15)

```
σ' ∈ [-1, 0) → σ = 1 - σ'
σ' ∈ [0, 3]  → σ = 10^σ'
```

- 최종 범위: [0, 1000]

## 6. Updated Objective Function (Equation 12)

```
V(B) = Σ_{k=1}^{n-1} [f(B) + f_sm + f_jerk]
```

- 기존 TEB objectives에 smoothness와 jerk 추가

