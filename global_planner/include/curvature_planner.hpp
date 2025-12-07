//최소곡률 경로 헤더 (Clothoid 곡선 적용)

#ifndef CURVATURE_PLANNER_HPP
#define CURVATURE_PLANNER_HPP

#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include <vector>

using f110_msgs::msg::Wpnt;
using f110_msgs::msg::WpntArray;

class CurvaturePlanner {
public:
    CurvaturePlanner();
    WpntArray generate_minimum_curvature_path(const std::vector<Wpnt>& centerline);
    float calculate_curvature(const Wpnt& prev, const Wpnt& curr, const Wpnt& next);

private:
    // Clothoid 곡선 보간 함수
    // Clothoid: 곡률이 선형적으로 변화하는 곡선 (κ(s) = κ₀ + κ'·s)
    Wpnt interpolate_clothoid(
        const Wpnt& p0, const Wpnt& p1,
        float s, float ds_total,
        float kappa0, float dkappa, float theta0);
    
    // 곡률 스무딩을 통한 경로 최적화
    void optimize_curvature_smoothness(std::vector<Wpnt>& path);
};

#endif  // CURVATURE_PLANNER_HPP
