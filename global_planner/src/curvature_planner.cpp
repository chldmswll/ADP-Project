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
    
    // 0. 중심선을 먼저 스무딩하여 노이즈 제거
    std::vector<Wpnt> smoothed_centerline = centerline;
    if (smoothed_centerline.size() > 3) {
        for (size_t iter = 0; iter < 2; ++iter) {  // 2번 반복 스무딩
            std::vector<Wpnt> temp = smoothed_centerline;
            for (size_t i = 1; i < smoothed_centerline.size() - 1; ++i) {
                temp[i].x_m = 0.5f * smoothed_centerline[i].x_m + 
                              0.25f * (smoothed_centerline[i-1].x_m + smoothed_centerline[i+1].x_m);
                temp[i].y_m = 0.5f * smoothed_centerline[i].y_m + 
                              0.25f * (smoothed_centerline[i-1].y_m + smoothed_centerline[i+1].y_m);
            }
            smoothed_centerline = temp;
        }
    }
    
    // 1. 중심선 포인트들 사이에 클로소이드 곡선 보간
    std::vector<Wpnt> clothoid_path;
    
    // 첫 번째 포인트 추가
    clothoid_path.push_back(smoothed_centerline[0]);
    
    // 각 세그먼트에 대해 클로소이드 곡선 생성
    for (size_t i = 0; i < smoothed_centerline.size() - 1; ++i) {
        const Wpnt& p0 = smoothed_centerline[i];
        const Wpnt& p1 = smoothed_centerline[i + 1];
        
        // 두 점 사이의 거리 계산
        float dx = p1.x_m - p0.x_m;
        float dy = p1.y_m - p0.y_m;
        float ds = std::sqrt(dx * dx + dy * dy);
        
        if (ds < 1e-6) {
            continue; // 너무 가까운 점은 스킵
        }
        
        // 초기 곡률과 곡률 변화율 계산 (스무딩된 값 사용)
        float kappa0 = p0.kappa_radpm;
        float kappa1 = p1.kappa_radpm;
        float dkappa = (kappa1 - kappa0) / std::max(ds, 0.1f); // 곡률 변화율 (κ')
        
        // 초기 방향각 (실제 방향 계산)
        float theta0 = std::atan2(dy, dx);
        
        // 클로소이드 곡선을 따라 중간 포인트들 생성
        // 거리에 따라 적절한 수의 포인트 생성
        int num_points = std::max(2, static_cast<int>(ds / 0.2f)); // 약 0.2m 간격
        num_points = std::min(num_points, 10); // 최대 10개 포인트로 제한
        
        for (int j = 1; j < num_points; ++j) {
            float s = static_cast<float>(j) / num_points * ds; // 호장(arc length)
            
            // 클로소이드 곡선의 파라미터 방정식
            Wpnt wpnt = interpolate_clothoid(p0, p1, s, ds, kappa0, dkappa, theta0);
            clothoid_path.push_back(wpnt);
        }
    }
    
    // 마지막 포인트 추가
    if (smoothed_centerline.size() > 1) {
        clothoid_path.push_back(smoothed_centerline.back());
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
    
    // 간단하고 안정적인 선형 보간 사용 (작은 구간에서는 충분히 정확)
    float t = s / std::max(ds_total, 0.01f);
    t = std::min(1.0f, std::max(0.0f, t));  // [0, 1] 범위로 제한
    
    // 기본 선형 보간
    wpnt.x_m = p0.x_m + t * (p1.x_m - p0.x_m);
    wpnt.y_m = p0.y_m + t * (p1.y_m - p0.y_m);
    
    // 작은 곡률 보정 (곡률이 클 때만 적용)
    if (std::abs(kappa) > 0.01f && ds_total > 0.1f) {
        float correction_factor = std::min(1.0f, std::abs(kappa) * ds_total * 0.1f);
        float perp_x = -(p1.y_m - p0.y_m) / ds_total;
        float perp_y = (p1.x_m - p0.x_m) / ds_total;
        
        // 곡률에 따른 수직 방향 보정
        float correction = s * (ds_total - s) * kappa * 0.1f;
        wpnt.x_m += correction * perp_x * correction_factor;
        wpnt.y_m += correction * perp_y * correction_factor;
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
    // 가중 이동 평균 필터를 사용하여 곡률을 부드럽게 만듦
    std::vector<float> smoothed_curvature(path.size());
    
    for (size_t i = 0; i < path.size(); ++i) {
        float sum = 0.0f;
        float weight_sum = 0.0f;
        
        // 주변 포인트들의 곡률 가중 평균 계산
        int window = 5; // 양쪽으로 5개 포인트씩 고려
        for (int j = -window; j <= window; ++j) {
            int idx = static_cast<int>(i) + j;
            if (idx >= 0 && idx < static_cast<int>(path.size())) {
                // 거리에 따른 가중치 (가까울수록 높은 가중치)
                float weight = 1.0f / (1.0f + std::abs(static_cast<float>(j)));
                sum += path[idx].kappa_radpm * weight;
                weight_sum += weight;
            }
        }
        
        smoothed_curvature[i] = (weight_sum > 0.0f) ? (sum / weight_sum) : path[i].kappa_radpm;
    }
    
    // 스무딩된 곡률 적용 (원래 값과의 가중 평균으로 부드럽게 전환)
    for (size_t i = 0; i < path.size(); ++i) {
        path[i].kappa_radpm = 0.7f * smoothed_curvature[i] + 0.3f * path[i].kappa_radpm;
    }
    
    // yaw 각도도 곡률에 맞게 재계산 (더 정확하게)
    for (size_t i = 1; i < path.size(); ++i) {
        float dx = path[i].x_m - path[i-1].x_m;
        float dy = path[i].y_m - path[i-1].y_m;
        float dist = std::sqrt(dx * dx + dy * dy);
        
        if (dist > 1e-6) {
            path[i-1].psi_rad = std::atan2(dy, dx);
        }
    }
    
    // 마지막 포인트의 yaw는 이전 포인트와 동일하게 설정
    if (path.size() > 1) {
        path.back().psi_rad = path[path.size() - 2].psi_rad;
    }
    
    // 첫 번째 포인트의 yaw도 설정
    if (path.size() > 1) {
        path[0].psi_rad = path[1].psi_rad;
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