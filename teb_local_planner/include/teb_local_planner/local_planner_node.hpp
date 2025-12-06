#pragma once

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <tuple>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include "teb_local_planner/teb.hpp"

#include "rclcpp/rclcpp.hpp"

namespace teb_local_planner
{

struct PlannerConfig
{
  // Basic parameters
  double refresh_hz{20.0};
  double horizon_length{10.0};
  double safety_margin{0.2};
  
  // Flags
  bool from_bag{false};
  bool measure{false};
  
  // Spline parameters
  double pre_apex_0{-4.0};
  double pre_apex_1{-3.0};
  double pre_apex_2{-1.5};
  double post_apex_0{2.0};
  double post_apex_1{3.0};
  double post_apex_2{4.0};
  
  // Obstacle avoidance parameters
  double evasion_dist{0.77};
  double obs_traj_tresh{0.3};
  double spline_bound_mindist{0.1};
  double fixed_pred_time{0.15};
  double kd_obs_pred{1.0};
  
  // Lookahead distance
  double lookahead{10.0};  // in meters
};

class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  double cur_s_{0.0};
  double cur_d_{0.0};
  double cur_vs_{0.0};

  double gb_vmax_{0.0};
  int gb_max_idx_{0};
  double gb_max_s_{0.0};

  double lookahead_{10.0};
  bool messages_ready_{false};
  
  void initialize_publishers();
  void initialize_subscriptions();
  void load_parameters();
  bool check_messages_ready();

  void on_timer();

  void obstaclesCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void globalWaypointsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void scaledWaypointsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg);

  // 장애물 필터링 및 처리
  std::vector<f110_msgs::msg::Obstacle> filter_obstacles();
  f110_msgs::msg::Obstacle find_closest_obstacle(const std::vector<f110_msgs::msg::Obstacle>& obstacles);
  f110_msgs::msg::Obstacle predict_obstacle_movement(const f110_msgs::msg::Obstacle& obs);
  
  // 경로 생성
  std::pair<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray> generate_evasion_trajectory(const f110_msgs::msg::Obstacle& obstacle);
  
  // TEB 헬퍼 함수들
  std::vector<Eigen::Vector2d> get_initial_path_from_global(const f110_msgs::msg::Obstacle& obstacle);
  std::vector<Eigen::Vector2d> convert_obstacle_to_points(const f110_msgs::msg::Obstacle& obstacle);

  // 시각화
  visualization_msgs::msg::MarkerArray create_delete_all_marker();
  
  f110_msgs::msg::ObstacleArray::SharedPtr obstacles_;
  f110_msgs::msg::WpntArray::SharedPtr global_wpnts_;
  f110_msgs::msg::WpntArray::SharedPtr global_wpnts_scaled_;
  nav_msgs::msg::Odometry::SharedPtr current_state_;

  PlannerConfig config_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_wpnts_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_wpnts_scaled_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr otwpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr considered_obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr propagated_obs_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;
  
  // Race line visualization publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr race_line_markers_pub_;
  
  // Helper function to convert waypoints to markers
  visualization_msgs::msg::MarkerArray create_race_line_markers(
    const f110_msgs::msg::WpntArray::SharedPtr& waypoints);
};

}  // namespace teb_local_planner

