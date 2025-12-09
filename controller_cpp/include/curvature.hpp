#pragma once
#include <array>
#include <vector>
#include <cmath>

namespace Curvature {
  using WpRow = std::array<double, 8>;

  // 곡률 계산 (앞쪽 5m 평균)
  double calc_curvature(
    int idx_nearest_wp,
    const std::vector<WpRow>& waypoints
  );

  // 미래 곡률 변화량 추정
  double estimate_future_curvature_change(
    int idx_nearest_wp,
    const std::vector<WpRow>& waypoints
  );

  // 곡률 기반 Stanley 가중치 계산
  double calc_stanley_weight(
    double curvature_now,
    double& prev_weight
  );
}

