#pragma once
#include <array>
#include <vector>
#include <optional>
#include <cmath>
#include <string>
#include <functional>

namespace Stanley {
  using WpRow = std::array<double, 8>;
  using Fren4 = std::array<double, 4>;

  // 스탠리 조향각 계산
  double calc_angle(
    double yaw,
    double lat_e_norm,
    int idx_nearest_wp,
    const std::vector<WpRow>& waypoints,
    const std::optional<Fren4>& frenet,
    double speed_now,
    std::function<void(const std::string&)> logger_warn
  );
}

