//중심선 추출 헤더

#ifndef CENTERLINE_EXTRACTOR_HPP
#define CENTERLINE_EXTRACTOR_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include <vector>
#include <utility>

using f110_msgs::msg::Wpnt;

class CenterlineExtractor {
public:
    CenterlineExtractor();
    void extract_centerline(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    const std::vector<Wpnt>& get_centerline() const;
    const std::vector<std::pair<double, double>>& get_left_boundary() const;
    const std::vector<std::pair<double, double>>& get_right_boundary() const;

private:
    std::vector<Wpnt> centerline_;  // 추출된 중심선
    std::vector<std::pair<double, double>> left_boundary_;   // 좌측 경계선 (x, y)
    std::vector<std::pair<double, double>> right_boundary_; // 우측 경계선 (x, y)
};

#endif  // CENTERLINE_EXTRACTOR_HPP
