//속도 제한 및 제약 반영 헤더 (Clothoid 곡선 최소 곡률 경로 보존)

#ifndef VELOCITY_PLANNER_HPP
#define VELOCITY_PLANNER_HPP

#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include <vector>

using f110_msgs::msg::Wpnt;
using f110_msgs::msg::WpntArray;

class VelocityPlanner {
public:
    VelocityPlanner();
    WpntArray generate_velocity_profile(const WpntArray& curvature_path);
    float calculate_max_speed(float curvature);

private:
    // 곡률 기반 속도 제한 계산
    float calculate_max_speed_from_curvature(float curvature);
    
    // 곡률 변화율 기반 속도 제한 (클로소이드 특성 보존)
    float calculate_max_speed_from_curvature_rate(
        const Wpnt& wpnt, size_t idx, const std::vector<Wpnt>& waypoints);
    
    // 횡가속도 기반 속도 제한
    float calculate_max_speed_from_lateral_accel(float curvature);
    
    // 가속/감속 제약 적용
    void apply_forward_constraints(
        std::vector<float>& velocities, const std::vector<Wpnt>& waypoints);
    void apply_backward_constraints(
        std::vector<float>& velocities, const std::vector<Wpnt>& waypoints);
    
    // 차량 물리 제약 파라미터
    float max_acceleration_;      // 최대 가속도 (m/s²)
    float max_deceleration_;      // 최대 감속도 (m/s²)
    float max_velocity_;          // 최대 속도 (m/s)
    float friction_coefficient_;  // 마찰 계수
    float gravity_;              // 중력 가속도 (m/s²)
    float lateral_accel_limit_;  // 최대 횡가속도 (m/s²)
};

#endif  // VELOCITY_PLANNER_HPP
