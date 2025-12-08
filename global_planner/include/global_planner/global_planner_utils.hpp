#ifndef GLOBAL_PLANNER__GLOBAL_PLANNER_UTILS_HPP_
#define GLOBAL_PLANNER__GLOBAL_PLANNER_UTILS_HPP_

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <geometry_msgs/msg/point.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace global_planner
{

std::string get_data_path(const std::string & subpath = "");

std::vector<cv::Point2i> extract_centerline(
  const cv::Mat & skeleton,
  double cent_length,
  double map_resolution,
  bool map_editor_mode);

std::vector<cv::Point2f> smooth_centerline(const std::vector<cv::Point2i> & centerline);

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> extract_track_bounds(
  const std::vector<cv::Point2i> & centerline,
  const cv::Mat & filtered_bw,
  bool map_editor_mode,
  double map_resolution,
  const geometry_msgs::msg::Point & map_origin,
  const Eigen::Vector3d & initial_position,
  bool show_plots);

std::pair<std::vector<double>, std::vector<double>> dist_to_bounds(
  const std::vector<Eigen::Vector2d> & trajectory,
  const std::vector<Eigen::Vector2d> & bound_r,
  const std::vector<Eigen::Vector2d> & bound_l,
  const std::vector<Eigen::Vector2d> & centerline,
  double safety_width,
  bool show_plots,
  bool reverse = false);

std::vector<Eigen::Vector4d> add_dist_to_cent(
  const std::vector<cv::Point2f> & centerline_smooth,
  const std::vector<Eigen::Vector2d> & centerline_meter,
  double map_resolution,
  double safety_width,
  bool show_plots,
  const cv::Mat & dist_transform = cv::Mat(),
  const std::vector<Eigen::Vector2d> & bound_r = {},
  const std::vector<Eigen::Vector2d> & bound_l = {},
  bool reverse = false);

std::pair<f110_msgs::msg::WpntArray, visualization_msgs::msg::MarkerArray> write_centerline(
  const std::vector<Eigen::Vector4d> & centerline,
  bool sp_bool = false);

visualization_msgs::msg::MarkerArray publish_track_bounds(
  const std::vector<Eigen::Vector2d> & bound_r,
  const std::vector<Eigen::Vector2d> & bound_l,
  bool reverse = false);

std::pair<f110_msgs::msg::WpntArray, visualization_msgs::msg::MarkerArray> create_wpnts_markers(
  const std::vector<Eigen::VectorXd> & trajectory,
  const std::vector<double> & d_right,
  const std::vector<double> & d_left,
  bool second_traj = false);

double conv_psi(double psi);

bool compare_direction(double alpha, double beta);

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__GLOBAL_PLANNER_UTILS_HPP_
