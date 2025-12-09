#include "stanley.hpp"
#include <algorithm>
#include <cmath>
#include <string>

namespace Stanley {
  double calc_angle(
    double yaw,
    double lat_e_norm,
    int idx_nearest_wp,
    const std::vector<WpRow>& waypoints,
    const std::optional<Fren4>& frenet,
    double speed_now,
    std::function<void(const std::string&)> logger_warn
  ) {
    if (idx_nearest_wp < 0 || idx_nearest_wp >= static_cast<int>(waypoints.size())) {
      logger_warn(std::string("[Stanley] idx_nearest_wp not set"));
      return 0.0;
    }

    if (!frenet) {
      logger_warn(std::string("[Stanley] frenet not available"));
      return 0.0;
    }

    const double heading = yaw;
    const double map_heading = waypoints[idx_nearest_wp][6]; // WpRow[6] = psi (heading)

    const double dpsi = std::atan2(std::sin(heading - map_heading), std::cos(heading - map_heading));
    const double lat_error = (*frenet)[1];  // signed lateral error

    constexpr double stanley_softening = 1.2;   // 0.5 -> 1.2로 증가 (스탠리 반응을 매우 부드럽게)
    const double vel = std::max(speed_now, 0.1);
    const double correction = std::atan2(lat_error, stanley_softening + vel);

    // 헤딩 오차(dpsi)와 측방 오차(lat_error)를 동시에 고려
    double stanley_angle = dpsi + correction;
    
    // 스탠리 각도에 rate limiting 추가 (극단적으로 강한 제한)
    static double stanley_prev = 0.0;
    constexpr double stanley_rate_limit = 0.06;  // 0.15 -> 0.06으로 대폭 감소 (매우 느린 변화만 허용)
    const double stanley_diff = stanley_angle - stanley_prev;
    if (std::abs(stanley_diff) > stanley_rate_limit) {
      stanley_angle = stanley_prev + (stanley_diff > 0 ? stanley_rate_limit : -stanley_rate_limit);
    }
    stanley_prev = stanley_angle;
    
    return stanley_angle;
  }
}

