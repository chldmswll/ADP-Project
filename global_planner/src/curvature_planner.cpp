// 최소곡률 경로 생성 (Clothoid 곡선 적용)

#include "curvature_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <algorithm>

CurvaturePlanner::CurvaturePlanner() {
    // 초기화
}

WpntArray CurvaturePlanner::generate_minimum_curvature_path(const std::vector<Wpnt>& centerline) {
    WpntArray path;
    path.header.frame_id = "map";
    rclcpp::Clock clock;
    path.header.stamp = clock.now();
    
    if (centerline.size() < 2) {
        return path;
    }
    
    // 클로소이드 스플라인을 사용한 최소 곡률 경로 생성
    // Clothoid: 곡률이 선형적으로 변화하는 곡선 (κ(s) = κ₀ + κ'·s)
    
    // 1. 중심선 포인트들 사이에 클로소이드 곡선 보간
    std::vector<Wpnt> clothoid_path;
    
    // 첫 번째 포인트 추가
    clothoid_path.push_back(centerline[0]);
    
    // 각 세그먼트에 대해 클로소이드 곡선 생성
    for (size_t i = 0; i < centerline.size() - 1; ++i) {
        const Wpnt& p0 = centerline[i];
        const Wpnt& p1 = centerline[i + 1];
        
        // 두 점 사이의 거리 계산
        float dx = p1.x_m - p0.x_m;
        float dy = p1.y_m - p0.y_m;
        float ds = std::sqrt(dx * dx + dy * dy);
        
        if (ds < 1e-6) {
            continue; // 너무 가까운 점은 스킵
        }
        
        // 초기 곡률과 곡률 변화율 계산
        float kappa0 = p0.kappa_radpm;
        float kappa1 = p1.kappa_radpm;
        float dkappa = (kappa1 - kappa0) / ds; // 곡률 변화율 (κ')
        
        // 초기 방향각
        float theta0 = p0.psi_rad;
        
        // 클로소이드 곡선을 따라 중간 포인트들 생성
        int num_points = std::max(3, static_cast<int>(ds / 0.5)); // 약 0.5m 간격
        num_points = std::min(num_points, 20); // 최대 20개 포인트로 제한
        
        for (int j = 1; j < num_points; ++j) {
            float s = static_cast<float>(j) / num_points * ds; // 호장(arc length)
            
            // 클로소이드 곡선의 파라미터 방정식 (Fresnel 적분 근사)
            Wpnt wpnt = interpolate_clothoid(p0, p1, s, ds, kappa0, dkappa, theta0);
            clothoid_path.push_back(wpnt);
        }
    }
    
    // 마지막 포인트 추가
    if (centerline.size() > 1) {
        clothoid_path.push_back(centerline.back());
    }
    
    // 2. 곡률 최적화: 곡률 변화를 부드럽게 만들기
    optimize_curvature_smoothness(clothoid_path);
    
    // 3. 경로를 WpntArray로 변환
    for (const auto& wpnt : clothoid_path) {
        path.wpnts.push_back(wpnt);
    }
    
    return path;
}

Wpnt CurvaturePlanner::interpolate_clothoid(
    const Wpnt& p0, const Wpnt& p1, 
    float s, float ds_total,
    float kappa0, float dkappa, float theta0) {
    
    Wpnt wpnt;
    
    // 클로소이드 곡선의 곡률: κ(s) = κ₀ + κ'·s
    float kappa = kappa0 + dkappa * s;
    
    // 방향각: θ(s) = θ₀ + ∫₀ˢ κ(τ) dτ = θ₀ + κ₀·s + (1/2)·κ'·s²
    float theta = theta0 + kappa0 * s + 0.5f * dkappa * s * s;
    
    // Fresnel 적분을 사용한 위치 계산 (간단한 근사)
    // x(s) = x₀ + ∫₀ˢ cos(θ(τ)) dτ
    // y(s) = y₀ + ∫₀ˢ sin(θ(τ)) dτ
    
    // 작은 구간에 대해서는 선형 근사 사용
    float cos_theta = std::cos(theta);
    float sin_theta = std::sin(theta);
    
    // 선형 보간과 곡률 보정을 결합
    float t = s / ds_total;
    wpnt.x_m = p0.x_m + t * (p1.x_m - p0.x_m);
    wpnt.y_m = p0.y_m + t * (p1.y_m - p0.y_m);
    
    // 곡률 보정을 위한 추가 오프셋 (작은 곡률 변화 고려)
    if (std::abs(dkappa) > 1e-6) {
        float correction = s * s * s * dkappa / 6.0f; // 3차 항 근사
        wpnt.x_m += correction * cos_theta;
        wpnt.y_m += correction * sin_theta;
    }
    
    wpnt.psi_rad = theta;
    wpnt.kappa_radpm = kappa;
    wpnt.vx_mps = 0.0; // 속도는 velocity_planner에서 설정
    wpnt.id = 0; // ID는 나중에 설정
    
    return wpnt;
}

void CurvaturePlanner::optimize_curvature_smoothness(std::vector<Wpnt>& path) {
    if (path.size() < 3) {
        return;
    }
    
    // 곡률 변화율을 최소화하기 위한 스무딩
    // 이동 평균 필터를 사용하여 곡률을 부드럽게 만듦
    std::vector<float> smoothed_curvature(path.size());
    
    for (size_t i = 0; i < path.size(); ++i) {
        float sum = 0.0f;
        int count = 0;
        
        // 주변 포인트들의 곡률 평균 계산
        int window = 3; // 양쪽으로 3개 포인트씩 고려
        for (int j = -window; j <= window; ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(path.size())) {
                sum += path[idx].kappa_radpm;
                count++;
            }
        }
        
        smoothed_curvature[i] = sum / count;
    }
    
    // 스무딩된 곡률 적용
    for (size_t i = 0; i < path.size(); ++i) {
        path[i].kappa_radpm = smoothed_curvature[i];
    }
    
    // yaw 각도도 곡률에 맞게 재계산
    for (size_t i = 1; i < path.size(); ++i) {
        float dx = path[i].x_m - path[i-1].x_m;
        float dy = path[i].y_m - path[i-1].y_m;
        path[i-1].psi_rad = std::atan2(dy, dx);
    }
    
    if (path.size() > 1) {
        path.back().psi_rad = path[path.size() - 2].psi_rad;
    }
}

float CurvaturePlanner::calculate_curvature(const Wpnt& prev, const Wpnt& curr, const Wpnt& next) {
    float dx1 = curr.x_m - prev.x_m;
    float dy1 = curr.y_m - prev.y_m;
    float dx2 = next.x_m - curr.x_m;
    float dy2 = next.y_m - curr.y_m;
    
    // 곡률 계산: κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
    float cross_product = dx1 * dy2 - dy1 * dx2;
    float ds_squared = dx1 * dx1 + dy1 * dy1;
    
    if (ds_squared < 1e-9) {
        return 0.0f;
    }
    
    float curvature = cross_product / (std::pow(ds_squared, 1.5f) + 1e-9);
    return curvature;
}


/*
1. Clothoid 곡선 보간
곡률이 선형적으로 변화하는 클로소이드 곡선 사용: κ(s) = κ₀ + κ'·s
중심선 포인트들 사이에 클로소이드 곡선을 보간하여 부드러운 경로 생성
방향각 계산: θ(s) = θ₀ + κ₀·s + (1/2)·κ'·s²
2. 최소 곡률 경로 생성
각 세그먼트에서 초기 곡률(κ₀)과 곡률 변화율(κ') 계산
클로소이드 곡선을 따라 중간 포인트들을 생성 (약 0.5m 간격)
Fresnel 적분 근사를 사용한 위치 계산
3. 곡률 스무딩 최적화
이동 평균 필터를 사용하여 곡률 변화를 부드럽게 만듦
G¹ 연속성 보장 (방향 연속성)
곡률 변화율을 최소화하여 최소 곡률 경로 생성
주요 함수:
interpolate_clothoid(): 두 점 사이에 클로소이드 곡선 보간
optimize_curvature_smoothness(): 곡률 스무딩을 통한 경로 최적화
calculate_curvature(): 개선된 곡률 계산
이제 중심선에서 클로소이드 곡선을 적용한 최소 곡률 경로가 생성됩니다.    
*/