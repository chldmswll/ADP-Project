#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

#include "f110_msgs/msg/obstacle.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/ot_wpnt_array.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "frenet_conversion_cpp/frenet_converter_cpp.hpp"
#include "spline_planner_cpp/teb.hpp"
#include <Eigen/Dense>

class ObstacleSpliner : public rclcpp::Node {
public:
  ObstacleSpliner();

private:
  // Callbacks
  void obs_cb(const f110_msgs::msg::ObstacleArray::SharedPtr msg);
  void state_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void gb_cb(const f110_msgs::msg::WpntArray::SharedPtr msg);
  void gb_scaled_cb(const f110_msgs::msg::WpntArray::SharedPtr msg);
  
  // Dynamic parameters callback
  rcl_interfaces::msg::SetParametersResult dyn_param_cb(
    const std::vector<rclcpp::Parameter> &params);

  // Main loop
  void spliner_loop();

  // Helper functions
  void wait_for_messages();
  void initialize_converter();
  
  // Utility functions
  f110_msgs::msg::Obstacle predict_obs_movement(
    const f110_msgs::msg::Obstacle& obs, 
    const std::string& mode = "constant");
  
  bool check_ot_side_possible(const std::string& more_space);
  
  std::pair<std::string, double> more_space(
    const f110_msgs::msg::Obstacle& obstacle,
    const std::vector<f110_msgs::msg::Wpnt>& gb_wpnts,
    const std::vector<int>& gb_idxs);
  
  // Main spline function
  std::pair<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray>
  do_spline(const f110_msgs::msg::ObstacleArray& obstacles,
            const std::vector<f110_msgs::msg::Wpnt>& gb_wpnts);
  
  std::vector<f110_msgs::msg::Obstacle> obs_filtering(
    const f110_msgs::msg::ObstacleArray& obstacles);
  
  // TEB helper functions
  std::vector<Eigen::Vector2d> get_initial_path_from_global(
    const f110_msgs::msg::Obstacle& obstacle);
  std::vector<Eigen::Vector2d> convert_obstacle_to_points(
    const f110_msgs::msg::Obstacle& obstacle);
  
  // Visualization functions
  visualization_msgs::msg::Marker xyv_to_markers(
    double x, double y, double v, size_t id);
  
  visualization_msgs::msg::Marker xy_to_point(
    double x, double y, bool opponent = true);
  
  f110_msgs::msg::Wpnt xyv_to_wpnts(
    double s, double d, double x, double y, double v, size_t id);

  // Member variables
  f110_msgs::msg::ObstacleArray obs_;
  f110_msgs::msg::WpntArray::SharedPtr gb_wpnts_;
  f110_msgs::msg::WpntArray::SharedPtr gb_scaled_wpnts_;
  double gb_vmax_;
  int gb_max_idx_;
  double gb_max_s_;
  
  double cur_s_;
  double cur_d_;
  double cur_vs_;
  
  double lookahead_;
  rclcpp::Time last_switch_time_;
  std::string last_ot_side_;
  
  // Parameters
  bool from_bag_;
  bool measuring_;
  
  // Dynamic parameters
  double pre_apex_0_;
  double pre_apex_1_;
  double pre_apex_2_;
  double post_apex_0_;
  double post_apex_1_;
  double post_apex_2_;
  double evasion_dist_;
  double obs_traj_tresh_;
  double spline_bound_mindist_;
  double fixed_pred_time_;
  double kd_obs_pred_;
  
  // Converter
  std::unique_ptr<FrenetConverter> converter_;
  
  // Publishers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mrks_pub_;
  rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr evasion_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closest_obs_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_propagated_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;
  
  // Subscribers
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr gb_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr gb_scaled_sub_;
  
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

