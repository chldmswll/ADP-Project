//최단시간 경로 계산 헤더 (Clothoid 곡선 및 제약 속도 고려)

#ifndef TIME_OPTIMAL_PLANNER_HPP
#define TIME_OPTIMAL_PLANNER_HPP

#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include <vector>

using f110_msgs::msg::Wpnt;
using f110_msgs::msg::WpntArray;

class TimeOptimalPlanner {
public:
    TimeOptimalPlanner();
    WpntArray optimize_time_path(const WpntArray& velocity_profile);
    float optimize_velocity(float current_velocity);

private:
    // 구간 거리 계산
    void calculate_segment_distances(
        const std::vector<Wpnt>& waypoints, std::vector<float>& distances);
    
    // 전방 시간 최적화 (Bang-bang 제어: 최대 감속 사용)
    std::vector<float> apply_forward_time_optimization(
        const std::vector<Wpnt>& waypoints,
        const std::vector<float>& segment_distances);
    
    // 후방 시간 최적화 (Bang-bang 제어: 최대 가속 사용)
    std::vector<float> apply_backward_time_optimization(
        const std::vector<float>& forward_velocities,
        const std::vector<Wpnt>& waypoints,
        const std::vector<float>& segment_distances);
    
    // 클로소이드 곡선의 곡률 변화율을 고려한 속도 스무딩
    std::vector<float> smooth_velocity_for_clothoid(
        const std::vector<float>& velocities,
        const std::vector<Wpnt>& waypoints,
        const std::vector<float>& segment_distances);
    
    // 구간 통과 시간 계산
    float calculate_segment_time(
        float v0, float v1, float ds, float max_accel, float max_decel);
    
    // 물리 제약 파라미터
    float max_acceleration_;  // 최대 가속도 (m/s²)
    float max_deceleration_;  // 최대 감속도 (m/s²)
    float max_velocity_;      // 최대 속도 (m/s)
    float time_step_;         // 시간 스텝 (s)
};

#endif  // TIME_OPTIMAL_PLANNER_HPP
