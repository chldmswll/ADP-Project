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
        constexpr double k_gain = 0.6;  // cross-track 항 강도 감소 (1.0 -> 0.6)
        const double vel = std::max(speed_now, 0.1);
        const double correction = std::atan2(k_gain * lat_error, stanley_softening + vel);

        // 헤딩 오차(dpsi)와 측방 오차(lat_error)를 동시에 고려
        // 부호 반전: dpsi > 0이면 경로보다 오른쪽으로 향함 → 왼쪽 조향 필요 → 음수 조향각
        // correction > 0이면 경로 오른쪽에 있음 → 왼쪽 조향 필요 → 음수 조향각
        constexpr double heading_error_scale = 0.7;  // heading_error scale 감소 (1.0 -> 0.7)
        double stanley_angle = -(heading_error_scale * dpsi + correction);

        // 스탠리 각도에 rate limiting 추가 (극단적으로 강한 제한)
        static double stanley_prev = 0.0;
        constexpr double stanley_rate_limit = 0.035;  // 0.06 -> 0.035로 감소 (더 스무스한 조향)
        const double stanley_diff = stanley_angle - stanley_prev;
        if (std::abs(stanley_diff) > stanley_rate_limit) {
            stanley_angle = stanley_prev + (stanley_diff > 0 ? stanley_rate_limit : -stanley_rate_limit);
        }
        stanley_prev = stanley_angle;

        return stanley_angle;
    }
}