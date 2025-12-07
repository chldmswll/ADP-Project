//공통 함수 헤더

#ifndef UTILS_HPP
#define UTILS_HPP

#include "f110_msgs/msg/wpnt.hpp"
#include <vector>

using f110_msgs::msg::Wpnt;

// 거리 계산
float calculate_distance(const Wpnt& p1, const Wpnt& p2);
float calculate_distance(float x1, float y1, float x2, float y2);

// 방향각(yaw) 계산
float calculate_yaw(const Wpnt& from, const Wpnt& to);
float calculate_yaw(float dx, float dy);

// 곡률 계산
float calculate_curvature(const Wpnt& prev, const Wpnt& curr, const Wpnt& next);

// 곡률 변화율 계산 (클로소이드 특성 고려)
float calculate_curvature_rate(
    const Wpnt& prev, const Wpnt& curr, const Wpnt& next,
    float ds_prev, float ds_next);

// 각도 정규화 (-π ~ π)
float normalize_angle(float angle);

#endif  // UTILS_HPP
