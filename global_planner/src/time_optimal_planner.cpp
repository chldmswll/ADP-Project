//최단시간 경로 계산 (Clothoid 곡선 및 제약 속도 고려)

#include "time_optimal_planner.hpp"
#include <cmath>
#include <algorithm>

TimeOptimalPlanner::TimeOptimalPlanner() {
    // 초기화: 물리 제약 파라미터
    max_acceleration_ = 3.0f;      // 최대 가속도 (m/s²)
    max_deceleration_ = 5.0f;      // 최대 감속도 (m/s²)
    max_velocity_ = 20.0f;         // 최대 속도 (m/s)
    time_step_ = 0.01f;           // 시간 스텝 (s) - 최적화 계산용
}

WpntArray TimeOptimalPlanner::optimize_time_path(const WpntArray& velocity_profile) {
    WpntArray time_optimal_path;
    time_optimal_path.header = velocity_profile.header;
    
    if (velocity_profile.wpnts.empty()) {
        return time_optimal_path;
    }
    
    // 1단계: 각 구간의 거리 계산
    std::vector<float> segment_distances;
    calculate_segment_distances(velocity_profile.wpnts, segment_distances);
    
    // 1.5단계: 최소곡률(클로소이드) 기반 속도 제한 저장
    // velocity_profile의 속도는 이미 곡률 기반으로 계산된 최대 속도이므로
    // 이를 절대 상회하지 않도록 제약으로 사용
    std::vector<float> curvature_based_limits;
    curvature_based_limits.reserve(velocity_profile.wpnts.size());
    for (const auto& wpnt : velocity_profile.wpnts) {
        curvature_based_limits.push_back(static_cast<float>(wpnt.vx_mps));
    }
    
    // 2단계: Bang-bang 제어 원리를 사용한 최단 시간 속도 프로파일 생성
    // 최대 가속/감속을 사용하여 시간을 최소화
    // 단, 최소곡률(클로소이드) 제약을 위반하지 않도록 함
    
    // 전방 스캔: 곡률 제약을 고려하여 미리 감속해야 하는 구간 찾기
    std::vector<float> forward_velocities = apply_forward_time_optimization(
        velocity_profile.wpnts, segment_distances);
    
    // 곡률 제약 적용: 최소곡률 기반 속도 제한을 초과하지 않도록
    for (size_t i = 0; i < forward_velocities.size(); ++i) {
        forward_velocities[i] = std::min(forward_velocities[i], curvature_based_limits[i]);
    }
    
    // 후방 스캔: 가속 제약을 고려하여 가능한 최대 속도까지 가속
    std::vector<float> backward_velocities = apply_backward_time_optimization(
        forward_velocities, velocity_profile.wpnts, segment_distances);
    
    // 곡률 제약 적용: 최소곡률 기반 속도 제한을 초과하지 않도록
    for (size_t i = 0; i < backward_velocities.size(); ++i) {
        backward_velocities[i] = std::min(backward_velocities[i], curvature_based_limits[i]);
    }
    
    // 3단계: 클로소이드 곡선의 곡률 변화율을 고려한 부드러운 속도 조정
    std::vector<float> smoothed_velocities = smooth_velocity_for_clothoid(
        backward_velocities, velocity_profile.wpnts, segment_distances);
    
    // 곡률 제약 최종 적용: 최소곡률 기반 속도 제한을 초과하지 않도록
    for (size_t i = 0; i < smoothed_velocities.size(); ++i) {
        smoothed_velocities[i] = std::min(smoothed_velocities[i], curvature_based_limits[i]);
    }
    
    // 4단계: 최소곡률 경로 보존 확인 및 최종 경로 생성
    for (size_t i = 0; i < velocity_profile.wpnts.size(); ++i) {
        Wpnt wpnt = velocity_profile.wpnts[i];
        // 최소곡률(클로소이드) 경로의 곡률과 방향각은 그대로 유지
        // 속도만 최적화 (단, 곡률 제약 내에서)
        wpnt.vx_mps = static_cast<double>(smoothed_velocities[i]);
        // 곡률과 방향각은 velocity_profile에서 가져온 최소곡률 경로 값 유지
        time_optimal_path.wpnts.push_back(wpnt);
    }
    
    return time_optimal_path;
}

void TimeOptimalPlanner::calculate_segment_distances(
    const std::vector<Wpnt>& waypoints, std::vector<float>& distances) {
    
    distances.clear();
    distances.reserve(waypoints.size() - 1);
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        float dx = waypoints[i + 1].x_m - waypoints[i].x_m;
        float dy = waypoints[i + 1].y_m - waypoints[i].y_m;
        float ds = std::sqrt(dx * dx + dy * dy);
        distances.push_back(ds);
    }
}

std::vector<float> TimeOptimalPlanner::apply_forward_time_optimization(
    const std::vector<Wpnt>& waypoints,
    const std::vector<float>& segment_distances) {
    
    // 전방 스캔: 곡률이 증가하는 구간에서 미리 감속
    // 최단 시간을 위해 최대 감속도를 사용 (Bang-bang 제어)
    std::vector<float> velocities(waypoints.size());
    
    if (velocities.empty()) {
        return velocities;
    }
    
    // 초기 속도 설정
    velocities[0] = static_cast<float>(waypoints[0].vx_mps);
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        float current_vel = velocities[i];
        float ds = segment_distances[i];
        
        if (ds < 1e-6) {
            velocities[i + 1] = current_vel;
            continue;
        }
        
        // 다음 구간의 곡률 제약 (최소곡률 클로소이드 기반)
        // waypoints[i+1].vx_mps는 이미 곡률 기반으로 계산된 최대 속도
        float next_max_vel = static_cast<float>(waypoints[i + 1].vx_mps);
        
        // 최대 감속도로 감속할 수 있는 최소 속도
        // v_next² = v_current² - 2·a_decel·ds
        float min_next_vel = std::sqrt(
            std::max(0.0f, current_vel * current_vel - 2.0f * max_deceleration_ * ds));
        
        // 곡률 제약과 감속 제약 중 더 엄격한 것 선택
        // 최소곡률(클로소이드) 경로를 해치지 않도록 곡률 제약 우선 적용
        velocities[i + 1] = std::min(next_max_vel, 
            std::max(min_next_vel, current_vel - max_deceleration_ * (ds / std::max(current_vel, 0.1f))));
        
        // 다음 구간의 곡률이 더 크면 미리 감속
        if (i + 1 < waypoints.size() - 1) {
            float future_curvature = waypoints[i + 1].kappa_radpm;
            float current_curvature = waypoints[i].kappa_radpm;
            
            if (std::abs(future_curvature) > std::abs(current_curvature)) {
                // 곡률이 증가하는 구간: 미리 감속하여 최단 시간 달성
                float future_ds = segment_distances[i + 1];
                float required_vel = static_cast<float>(waypoints[i + 2].vx_mps);
                
                // 필요한 속도까지 감속할 수 있는지 확인
                float required_decel = (velocities[i + 1] * velocities[i + 1] - 
                                       required_vel * required_vel) / (2.0f * (ds + future_ds));
                
                if (required_decel > max_deceleration_) {
                    // 더 일찍 감속 시작
                    velocities[i + 1] = std::sqrt(
                        required_vel * required_vel + 2.0f * max_deceleration_ * (ds + future_ds));
                }
            }
        }
    }
    
    return velocities;
}

std::vector<float> TimeOptimalPlanner::apply_backward_time_optimization(
    const std::vector<float>& forward_velocities,
    const std::vector<Wpnt>& waypoints,
    const std::vector<float>& segment_distances) {
    
    // 후방 스캔: 가속 제약을 고려하여 가능한 최대 속도까지 가속
    // 최단 시간을 위해 최대 가속도를 사용 (Bang-bang 제어)
    std::vector<float> velocities = forward_velocities;
    
    if (velocities.size() < 2) {
        return velocities;
    }
    
    // 역방향으로 스캔하여 가속 가능한 구간 찾기
    for (int i = static_cast<int>(velocities.size()) - 2; i >= 0; --i) {
        float next_vel = velocities[i + 1];
        float ds = segment_distances[i];
        
        if (ds < 1e-6) {
            continue;
        }
        
        // 최대 가속도로 가속할 수 있는 최대 속도
        // v_next² = v_current² + 2·a_accel·ds
        float max_current_vel = std::sqrt(
            next_vel * next_vel + 2.0f * max_acceleration_ * ds);
        
        // 곡률 제약과 가속 제약 중 더 엄격한 것 선택
        // 최소곡률(클로소이드) 경로를 해치지 않도록 곡률 제약 우선 적용
        float curvature_limit = static_cast<float>(waypoints[i].vx_mps);
        // curvature_limit은 이미 곡률 기반으로 계산된 최대 속도이므로 이를 우선 적용
        velocities[i] = std::min({max_current_vel, curvature_limit, max_velocity_});
        
        // 현재 속도가 너무 낮으면 가속
        if (velocities[i] < next_vel - 0.1f) {
            // 가속 가능한 최대 속도까지 가속
            velocities[i] = std::min(max_current_vel, max_velocity_);
        }
    }
    
    return velocities;
}

std::vector<float> TimeOptimalPlanner::smooth_velocity_for_clothoid(
    const std::vector<float>& velocities,
    const std::vector<Wpnt>& waypoints,
    const std::vector<float>& segment_distances) {
    
    // 클로소이드 곡선의 곡률 변화율을 고려한 부드러운 속도 조정
    // 곡률 변화율이 클수록 속도 변화도 부드럽게
    std::vector<float> smoothed = velocities;
    
    if (smoothed.size() < 3) {
        return smoothed;
    }
    
    // 곡률 변화율 기반 스무딩
    for (size_t i = 1; i < smoothed.size() - 1; ++i) {
        // 곡률 변화율 계산
        float dkappa = 0.0f;
        if (i > 0 && i < waypoints.size() - 1) {
            float ds_prev = segment_distances[i - 1];
            float ds_next = segment_distances[i];
            
            if (ds_prev > 1e-6 && ds_next > 1e-6) {
                dkappa = (waypoints[i + 1].kappa_radpm - waypoints[i - 1].kappa_radpm) / 
                         (ds_prev + ds_next);
            }
        }
        
        // 곡률 변화율이 클수록 속도 변화를 부드럽게
        // 클로소이드 곡선의 최소곡률 특성을 보존하기 위해 곡률 변화율 고려
        float dkappa_abs = std::abs(dkappa);
        if (dkappa_abs > 0.1f) {
            // 속도 변화율 제한
            float max_vel_change = 0.5f / (1.0f + dkappa_abs * 10.0f);
            
            float vel_prev = smoothed[i - 1];
            float vel_next = smoothed[i + 1];
            float vel_avg = (vel_prev + vel_next) / 2.0f;
            
            // 부드러운 보간
            float alpha = 0.3f; // 스무딩 강도
            smoothed[i] = (1.0f - alpha) * smoothed[i] + alpha * vel_avg;
            
            // 속도 변화율 제한
            if (std::abs(smoothed[i] - vel_prev) > max_vel_change) {
                smoothed[i] = vel_prev + (smoothed[i] > vel_prev ? 1.0f : -1.0f) * max_vel_change;
            }
        }
        
        // 최소곡률(클로소이드) 제약 확인: 곡률 기반 최대 속도 초과 방지
        // waypoints[i].vx_mps는 이미 곡률 기반으로 계산된 최대 속도
        float curvature_max_vel = static_cast<float>(waypoints[i].vx_mps);
        smoothed[i] = std::min(smoothed[i], curvature_max_vel);
    }
    
    return smoothed;
}

float TimeOptimalPlanner::calculate_segment_time(
    float v0, float v1, float ds, float max_accel, float max_decel) {
    
    // 구간을 통과하는 데 걸리는 시간 계산
    // 가속 또는 감속 중 하나를 사용 (Bang-bang 제어)
    
    if (ds < 1e-6) {
        return 0.0f;
    }
    
    if (std::abs(v1 - v0) < 1e-3) {
        // 등속 운동
        return ds / v0;
    }
    
    if (v1 > v0) {
        // 가속 구간
        // v1² = v0² + 2·a·ds
        float accel = (v1 * v1 - v0 * v0) / (2.0f * ds);
        accel = std::min(accel, max_accel);
        
        // t = (v1 - v0) / a
        float t = (v1 - v0) / accel;
        return t;
    } else {
        // 감속 구간
        // v1² = v0² - 2·a·ds
        float decel = (v0 * v0 - v1 * v1) / (2.0f * ds);
        decel = std::min(decel, max_decel);
        
        // t = (v0 - v1) / a
        float t = (v0 - v1) / decel;
        return t;
    }
}

float TimeOptimalPlanner::optimize_velocity(float current_velocity) {
    // 기존 함수 호환성을 위한 래퍼
    // 실제 최적화는 optimize_time_path에서 수행
    return std::min(current_velocity, max_velocity_);
}



/*
구현 내용
1. Bang-bang 제어 원리 적용
최대 가속/감속을 사용해 시간을 최소화
Pontryagin의 최대 원리 기반
2. 전방 시간 최적화 (apply_forward_time_optimization)
곡률 증가 구간에서 미리 감속
감속 제약: v_next² = v_current² - 2·a_decel·ds
곡률이 증가하는 구간을 미리 감지해 최대 감속도로 감속
3. 후방 시간 최적화 (apply_backward_time_optimization)
가속 제약을 고려해 가능한 최대 속도까지 가속
가속 제약: v_next² = v_current² + 2·a_accel·ds
역방향 스캔으로 가속 가능 구간 탐색
4. 클로소이드 곡선 고려 스무딩 (smooth_velocity_for_clothoid)
곡률 변화율(κ')을 고려한 속도 조정
곡률 변화율이 클수록 속도 변화를 부드럽게
클로소이드의 부드러운 곡률 변화 보존
5. 구간 시간 계산 (calculate_segment_time)
각 구간 통과 시간 계산
가속/감속 구간별 시간 계산
전체 경로 시간 최소화
주요 함수:
calculate_segment_distances(): 각 구간의 거리 계산
apply_forward_time_optimization(): 전방 스캔으로 감속 구간 최적화
apply_backward_time_optimization(): 후방 스캔으로 가속 구간 최적화
smooth_velocity_for_clothoid(): 클로소이드 곡률 변화율 고려 스무딩
calculate_segment_time(): 구간 통과 시간 계산
물리 제약 파라미터:
최대 가속도: 3.0 m/s²
최대 감속도: 5.0 m/s²
최대 속도: 20.0 m/s
이 구현은 클로소이드 곡선의 최소 곡률 경로와 제약 속도를 고려하여 최단 시간 경로를 계산하고 속도를 최적화합니다. Bang-bang 제어 원리를 적용해 시간을 최소화합니다.*/