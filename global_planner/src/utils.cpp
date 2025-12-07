//공통 유틸리티 함수 (곡률 계산, 거리 계산, 각도 계산 등)

#include "utils.hpp"
#include <cmath>

// 두 점 사이의 거리 계산
float calculate_distance(const Wpnt& p1, const Wpnt& p2) {
    return calculate_distance(p1.x_m, p1.y_m, p2.x_m, p2.y_m);
}

float calculate_distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}
float calculate_yaw(const Wpnt& from, const Wpnt& to) {
    float dx = to.x_m - from.x_m;
    float dy = to.y_m - from.y_m;
    return calculate_yaw(dx, dy);
}

float calculate_yaw(float dx, float dy) {
    return std::atan2(dy, dx);
}

// 곡률 계산: κ = (x'y'' - y'x'') / (x'² + y'²)^(3/2)
float calculate_curvature(const Wpnt& prev, const Wpnt& curr, const Wpnt& next) {
    float dx1 = curr.x_m - prev.x_m;
    float dy1 = curr.y_m - prev.y_m;
    float dx2 = next.x_m - curr.x_m;
    float dy2 = next.y_m - curr.y_m;
    
    // 곡률 계산
    float cross_product = dx1 * dy2 - dy1 * dx2;
    float ds_squared = dx1 * dx1 + dy1 * dy1;
    
    if (ds_squared < 1e-9) {
        return 0.0f;
    }
    
    float curvature = cross_product / (std::pow(ds_squared, 1.5f) + 1e-9);
    return curvature;
}

// 곡률 변화율 계산 (클로소이드 특성 고려): dκ/ds
float calculate_curvature_rate(
    const Wpnt& prev, const Wpnt& curr, const Wpnt& next,
    float ds_prev, float ds_next) {
    
    if (ds_prev < 1e-6 || ds_next < 1e-6) {
        return 0.0f;
    }
    
    // 곡률 변화율: dκ/ds = (κ_next - κ_prev) / (ds_prev + ds_next)
    float dkappa = (next.kappa_radpm - prev.kappa_radpm) / (ds_prev + ds_next);
    return dkappa;
}

// 각도 정규화 (-π ~ π)
float normalize_angle(float angle) {
    while (angle > M_PI) {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -M_PI) {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

/*추가된 공통 유틸리티 함수
1. 거리 계산 함수
calculate_distance(const Wpnt& p1, const Wpnt& p2): 두 Wpnt 포인트 사이의 거리
calculate_distance(float x1, float y1, float x2, float y2): 좌표값으로 거리 계산
여러 파일에서 중복 사용되던 거리 계산 로직 통합
2. 방향각(yaw) 계산 함수
calculate_yaw(const Wpnt& from, const Wpnt& to): 두 포인트 사이의 방향각
calculate_yaw(float dx, float dy): 차이값으로 방향각 계산
centerline_extractor.cpp와 curvature_planner.cpp에서 사용되던 로직 통합
3. 곡률 계산 함수 (개선)
calculate_curvature(): 개선된 곡률 계산
0으로 나누기 방지 및 안정성 개선
curvature_planner.cpp의 개선된 버전 반영
4. 곡률 변화율 계산 함수
calculate_curvature_rate(): 클로소이드 곡선의 곡률 변화율(κ') 계산
velocity_planner.cpp에서 사용되던 로직을 공통 함수로 분리
클로소이드 특성 고려
5. 각도 정규화 함수
normalize_angle(): 각도를 -π ~ π 범위로 정규화
향후 각도 계산 시 유용
이제 다른 파일들에서 이 공통 함수들을 #include "utils.hpp"로 사용할 수 있습니다. 코드 중복을 줄이고 유지보수가 쉬워집니다.*/
