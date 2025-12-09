#include "curvature.hpp"
#include <algorithm>
#include <cmath>

namespace Curvature {
  static inline double clip(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  // 곡률 계산 (앞쪽 5m 평균)
  double calc_curvature(
    int idx_nearest_wp,
    const std::vector<WpRow>& waypoints
  ) {
    if (static_cast<int>(waypoints.size()) - idx_nearest_wp <= 2) {
      return 0.0;
    }

    constexpr double curvature_horizon_m = 5.0;
    constexpr double waypoint_res_m = 0.1;
    const int max_points_ahead = static_cast<int>(curvature_horizon_m / waypoint_res_m);
    
    double sum = 0.0;
    int cnt = 0;
    const int last_idx = std::min(
        idx_nearest_wp + max_points_ahead,
        static_cast<int>(waypoints.size()) - 1
    );
    
    for (int i = idx_nearest_wp; i <= last_idx; ++i) {
      sum += std::abs(waypoints[i][5]); // WpRow[5] = kappa
      ++cnt;
    }
    return (cnt > 0) ? (sum / static_cast<double>(cnt)) : 0.0;
  }

  // 미래 곡률 변화량 추정
  double estimate_future_curvature_change(
    int idx_nearest_wp,
    const std::vector<WpRow>& waypoints
  ) {
    if (waypoints.size() < 3 || idx_nearest_wp < 0) {
      return 0.0;
    }

    constexpr double curvature_delta_horizon_m = 5.0;  // [m] 미래 구간 길이 (3.0 ~ 5.0 사이 조절 가능)
    constexpr double waypoint_res_m = 0.1;
    const int horizon_idx = static_cast<int>(curvature_delta_horizon_m / waypoint_res_m);

    const int start_idx = idx_nearest_wp;
    const int mid_idx = std::min(start_idx + horizon_idx / 2, static_cast<int>(waypoints.size()) - 1);
    const int end_idx = std::min(start_idx + horizon_idx, static_cast<int>(waypoints.size()) - 1);

    const double curv_now = waypoints[start_idx][5]; // WpRow[5] = kappa
    const double curv_mid = waypoints[mid_idx][5];
    const double curv_end = waypoints[end_idx][5];

    // 현재-중간-미래 곡률의 변화 추세를 단순 미분으로 근사
    return (curv_end - curv_mid) + (curv_mid - curv_now);
  }

  // 곡률 기반 Stanley 가중치 계산
  double calc_stanley_weight(
    double curvature_now,
    double& prev_weight
  ) {
    constexpr double curvature_thresh = 0.05;
    const double k_norm = clip(std::abs(curvature_now) / curvature_thresh, 0.0, 1.0);
    constexpr double w_min = 0.12;
    constexpr double w_max = 1.0;
    double stanley_weight_raw = w_min + (w_max - w_min) * k_norm;
    constexpr double weight_alpha = 0.7;
    double stanley_weight = weight_alpha * prev_weight + (1.0 - weight_alpha) * stanley_weight_raw;
    prev_weight = stanley_weight;
    return stanley_weight;
  }
}

