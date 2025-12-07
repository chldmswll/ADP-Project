//중심선 추출 헤더

#ifndef CENTERLINE_EXTRACTOR_HPP
#define CENTERLINE_EXTRACTOR_HPP

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include <vector>

using f110_msgs::msg::Wpnt;

class CenterlineExtractor {
public:
    CenterlineExtractor();
    void extract_centerline(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    const std::vector<Wpnt>& get_centerline() const;

private:
    std::vector<Wpnt> centerline_;  // 추출된 중심선
};

#endif  // CENTERLINE_EXTRACTOR_HPP
