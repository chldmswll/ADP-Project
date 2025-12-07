//속도 제한 및 가속/감속 제약 반영 (Clothoid 곡선 최소 곡률 경로 보존)

#include "velocity_planner.hpp"
#include <cmath>
#include <algorithm>

VelocityPlanner::VelocityPlanner() {
    // 초기화: 차량 물리 제약 파라미터
    max_acceleration_ = 3.0f;      // 최대 가속도 (m/s²)
    max_deceleration_ = 5.0f;      // 최대 감속도 (m/s²)
    max_velocity_ = 20.0f;         // 최대 속도 (m/s)
    friction_coefficient_ = 1.0f; // 마찰 계수
    gravity_ = 9.81f;             // 중력 가속도 (m/s²)
    lateral_accel_limit_ = 3.0f;  // 최대 횡가속도 (m/s²)
}

WpntArray VelocityPlanner::generate_velocity_profile(const WpntArray& curvature_path) {
    WpntArray velocity_profile;
    velocity_profile.header = curvature_path.header;
    
    if (curvature_path.wpnts.empty()) {
        return velocity_profile;
    }
    
    // 1단계: 각 점에서의 곡률 기반 최대 속도 계산
    // 클로소이드 곡선의 곡률 변화율을 고려하여 속도 제한
    std::vector<float> max_velocities;
    max_velocities.reserve(curvature_path.wpnts.size());
    
    for (size_t i = 0; i < curvature_path.wpnts.size(); ++i) {
        const auto& wpnt = curvature_path.wpnts[i];
        
        // 곡률 기반 속도 제한
        float curvature_limit = calculate_max_speed_from_curvature(wpnt.kappa_radpm);
        
        // 곡률 변화율 기반 속도 제한 (클로소이드 특성 보존)
        float dkappa_limit = calculate_max_speed_from_curvature_rate(
            wpnt, i, curvature_path.wpnts);
        
        // 횡가속도 제한
        float lateral_limit = calculate_max_speed_from_lateral_accel(wpnt.kappa_radpm);
        
        // 최종 최대 속도는 가장 작은 값
        float max_vel = std::min({curvature_limit, dkappa_limit, lateral_limit, max_velocity_});
        max_velocities.push_back(max_vel);
    }
    
    // 2단계: 가속/감속 제약을 고려한 속도 프로파일 생성
    // 전방 스캔: 곡률이 증가하는 구간에서 미리 속도 감소
    std::vector<float> forward_velocities = max_velocities;
    apply_forward_constraints(forward_velocities, curvature_path.wpnts);
    
    // 후방 스캔: 가속 제약을 고려하여 속도 조정
    std::vector<float> backward_velocities = forward_velocities;
    apply_backward_constraints(backward_velocities, curvature_path.wpnts);
    
    // 3단계: 최종 속도 프로파일 생성
    for (size_t i = 0; i < curvature_path.wpnts.size(); ++i) {
        Wpnt velocity_wpnt = curvature_path.wpnts[i];
        velocity_wpnt.vx_mps = static_cast<double>(backward_velocities[i]);
        velocity_profile.wpnts.push_back(velocity_wpnt);
    }
    
    return velocity_profile;
}

float VelocityPlanner::calculate_max_speed_from_curvature(float curvature) {
    // 곡률 기반 최대 속도: v_max = sqrt(μ·g / κ)
    // 클로소이드 곡선의 최소 곡률 경로를 유지하기 위한 속도 제한
    if (std::abs(curvature) < 1e-6) {
        return max_velocity_; // 직선 구간은 최대 속도 허용
    }
    
    float max_speed = std::sqrt(friction_coefficient_ * gravity_ / std::abs(curvature));
    return std::min(max_speed, max_velocity_);
}

float VelocityPlanner::calculate_max_speed_from_curvature_rate(
    const Wpnt& wpnt, size_t idx, const std::vector<Wpnt>& waypoints) {
    
    // 클로소이드 곡선의 곡률 변화율(κ')을 고려한 속도 제한
    // 곡률 변화율이 클수록 더 부드럽게 속도를 조절해야 함
    
    if (waypoints.size() < 2 || idx == 0 || idx == waypoints.size() - 1) {
        return max_velocity_;
    }
    
    // 이전/현재/다음 포인트 간 거리 계산
    float dx_prev = wpnt.x_m - waypoints[idx - 1].x_m;
    float dy_prev = wpnt.y_m - waypoints[idx - 1].y_m;
    float ds_prev = std::sqrt(dx_prev * dx_prev + dy_prev * dy_prev);
    
    float dx_next = waypoints[idx + 1].x_m - wpnt.x_m;
    float dy_next = waypoints[idx + 1].y_m - wpnt.y_m;
    float ds_next = std::sqrt(dx_next * dx_next + dy_next * dy_next);
    
    if (ds_prev < 1e-6 || ds_next < 1e-6) {
        return max_velocity_;
    }
    
    // 곡률 변화율 계산: dκ/ds
    float dkappa = (waypoints[idx + 1].kappa_radpm - waypoints[idx - 1].kappa_radpm) / (ds_prev + ds_next);
    
    // 곡률 변화율이 클수록 속도 제한을 더 엄격하게
    // 클로소이드의 부드러운 곡률 변화를 보존하기 위함
    float dkappa_abs = std::abs(dkappa);
    
    if (dkappa_abs < 1e-6) {
        return max_velocity_; // 곡률 변화가 거의 없으면 최대 속도
    }
    
    // 곡률 변화율 기반 속도 감소 계수
    // 곡률 변화율이 클수록 속도를 더 줄여서 경로의 부드러움 유지
    float reduction_factor = 1.0f / (1.0f + 0.5f * dkappa_abs);
    float base_speed = calculate_max_speed_from_curvature(wpnt.kappa_radpm);
    
    return base_speed * reduction_factor;
}

float VelocityPlanner::calculate_max_speed_from_lateral_accel(float curvature) {
    // 횡가속도 제한: a_lat = v²·κ ≤ a_lat_max
    // v_max = sqrt(a_lat_max / κ)
    if (std::abs(curvature) < 1e-6) {
        return max_velocity_;
    }
    
    float max_speed = std::sqrt(lateral_accel_limit_ / std::abs(curvature));
    return std::min(max_speed, max_velocity_);
}

void VelocityPlanner::apply_forward_constraints(
    std::vector<float>& velocities, const std::vector<Wpnt>& waypoints) {
    
    // 전방 스캔: 곡률이 증가하는 구간에서 미리 속도를 줄여서
    // 클로소이드 곡선의 최소 곡률 경로를 해치지 않도록 함
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        float dx = waypoints[i + 1].x_m - waypoints[i].x_m;
        float dy = waypoints[i + 1].y_m - waypoints[i].y_m;
        float ds = std::sqrt(dx * dx + dy * dy);
        
        if (ds < 1e-6) {
            continue;
        }
        
        float current_vel = velocities[i];
        float next_vel = velocities[i + 1];
        
        // 감속 제약: v_next² ≤ v_current² - 2·a_decel·ds
        // 최대 감속도로 감속할 수 있는 최소 속도 계산
        float min_next_vel = std::sqrt(
            std::max(0.0f, current_vel * current_vel - 2.0f * max_deceleration_ * ds));
        
        // 다음 속도가 너무 크면 현재 속도에서 조정
        if (next_vel > min_next_vel + 0.1f) {
            // 다음 구간의 곡률이 더 크면 미리 속도 감소
            if (std::abs(waypoints[i + 1].kappa_radpm) > std::abs(waypoints[i].kappa_radpm)) {
                velocities[i] = std::min(velocities[i], 
                    std::sqrt(next_vel * next_vel + 2.0f * max_deceleration_ * ds));
            }
        }
    }
}

void VelocityPlanner::apply_backward_constraints(
    std::vector<float>& velocities, const std::vector<Wpnt>& waypoints) {
    
    // 후방 스캔: 가속 제약을 고려하여 속도 조정
    // 클로소이드 곡선의 부드러운 속도 변화를 보장
    
    for (int i = static_cast<int>(waypoints.size()) - 2; i >= 0; --i) {
        float dx = waypoints[i + 1].x_m - waypoints[i].x_m;
        float dy = waypoints[i + 1].y_m - waypoints[i].y_m;
        float ds = std::sqrt(dx * dx + dy * dy);
        
        if (ds < 1e-6) {
            continue;
        }
        
        float current_vel = velocities[i];
        float next_vel = velocities[i + 1];
        
        // 가속 제약: v_next² ≤ v_current² + 2·a_accel·ds
        // 최대 가속도로 가속할 수 있는 최대 속도 계산
        float max_next_vel = std::sqrt(
            current_vel * current_vel + 2.0f * max_acceleration_ * ds);
        
        // 다음 속도가 가속 제약을 초과하면 조정
        if (next_vel > max_next_vel) {
            velocities[i + 1] = max_next_vel;
        }
        
        // 현재 속도도 조정 (역방향으로)
        float max_current_vel = std::sqrt(
            std::max(0.0f, next_vel * next_vel - 2.0f * max_deceleration_ * ds));
        
        if (current_vel > max_current_vel) {
            velocities[i] = max_current_vel;
        }
    }
}

float VelocityPlanner::calculate_max_speed(float curvature) {
    // 기존 함수 호환성을 위한 래퍼
    return calculate_max_speed_from_curvature(curvature);
}

/*
구현 내용
1. 곡률 기반 속도 제한
기본 공식: v_max = sqrt(μ·g / κ)
곡률이 클수록 속도 제한 강화
2. 곡률 변화율 기반 속도 제한 (클로소이드 특성 보존)
클로소이드의 곡률 변화율(κ')을 고려
곡률 변화율이 클수록 속도 감소 계수 적용
클로소이드의 부드러운 곡률 변화를 보존
3. 횡가속도 제한
횡가속도 제한: a_lat = v²·κ ≤ a_lat_max
곡률과 속도의 관계를 물리적으로 제약
4. 가속/감속 제약 적용
전방 스캔: 곡률 증가 구간에서 미리 속도 감소
감속 제약: v_next² ≤ v_current² - 2·a_decel·ds
후방 스캔: 가속 제약을 고려한 속도 조정
가속 제약: v_next² ≤ v_current² + 2·a_accel·ds
5. 물리 제약 파라미터
최대 가속도: 3.0 m/s²
최대 감속도: 5.0 m/s²
최대 속도: 20.0 m/s
최대 횡가속도: 3.0 m/s²
주요 함수:
calculate_max_speed_from_curvature(): 곡률 기반 속도 제한
calculate_max_speed_from_curvature_rate(): 곡률 변화율 기반 속도 제한 (클로소이드 특성 보존)
calculate_max_speed_from_lateral_accel(): 횡가속도 기반 속도 제한
apply_forward_constraints(): 전방 감속 제약 적용
apply_backward_constraints(): 후방 가속 제약 적용
이 구현은 클로소이드 곡선의 최소 곡률 경로를 유지하면서 물리적 제약을 만족하는 속도 프로파일을 생성합니다. 곡률 변화율을 고려해 경로의 부드러움을 보존합니다.
*/
