#ifndef GLOBAL_PLANNER__READWRITE_GLOBAL_WAYPOINTS_HPP_
#define GLOBAL_PLANNER__READWRITE_GLOBAL_WAYPOINTS_HPP_

#include <string>
#include <memory>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

namespace global_planner
{

void write_global_waypoints(
  const std::string & map_dir,
  const std::string & map_info_str,
  const std_msgs::msg::Float32 & est_lap_time,
  const visualization_msgs::msg::MarkerArray & centerline_markers,
  const f110_msgs::msg::WpntArray & centerline_waypoints,
  const visualization_msgs::msg::MarkerArray & global_traj_markers_iqp,
  const f110_msgs::msg::WpntArray & global_traj_wpnts_iqp,
  const visualization_msgs::msg::MarkerArray & global_traj_markers_sp,
  const f110_msgs::msg::WpntArray & global_traj_wpnts_sp,
  const visualization_msgs::msg::MarkerArray & trackbounds_markers
);

std::tuple<
  std_msgs::msg::String,
  std_msgs::msg::Float32,
  visualization_msgs::msg::MarkerArray,
  f110_msgs::msg::WpntArray,
  visualization_msgs::msg::MarkerArray,
  f110_msgs::msg::WpntArray,
  visualization_msgs::msg::MarkerArray,
  f110_msgs::msg::WpntArray,
  visualization_msgs::msg::MarkerArray
> read_global_waypoints(const std::string & map_dir);

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__READWRITE_GLOBAL_WAYPOINTS_HPP_
