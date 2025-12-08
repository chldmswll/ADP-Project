#ifndef GLOBAL_PLANNER__GLOBAL_PLANNER_LOGIC_HPP_
#define GLOBAL_PLANNER__GLOBAL_PLANNER_LOGIC_HPP_

#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace global_planner
{

class GlobalPlannerLogic
{
public:
  GlobalPlannerLogic(
    double safety_width,
    double safety_width_sp,
    double occupancy_grid_threshold,
    bool map_editor_mode,
    bool create_map,
    const std::string & map_name,
    const std::string & map_dir,
    const std::string & finish_script_path,
    const std::string & input_path,
    bool show_plots = false,
    int filter_kernel_size = 3,
    int required_laps = 1,
    bool reverse_mapping = false,
    std::function<void(const std::string &)> loginfo = nullptr,
    std::function<void(const std::string &)> logwarn = nullptr,
    std::function<void(const std::string &)> logerror = nullptr);

  std::pair<bool, std::string> global_plan_logic();

  void update_map(const nav_msgs::msg::OccupancyGrid & msg);
  void update_pose(const geometry_msgs::msg::PoseStamped & msg);

  bool compute_global_trajectory(double cent_length);

  cv::Mat filter_map_occupancy_grid();
  void save_map_png(const cv::Mat & filtered_map);

  std::string get_map_info_str() const { return map_info_str_; }
  std_msgs::msg::Float32 get_est_lap_time() const { return est_lap_time_; }

private:
  void update_lap_count();
  bool at_init_pos_check() const;

  // Parameters
  double safety_width_;
  double safety_width_sp_;
  double occupancy_grid_threshold_;
  int filter_kernel_size_;
  bool show_plots_;
  bool map_editor_mode_;
  bool compute_global_traj_;
  bool create_map_;
  std::string map_name_;
  std::string map_dir_;
  std::string script_path_;
  std::string input_path_;
  int required_laps_;
  bool reverse_mapping_;
  bool watershed_;

  // Logging functions
  std::function<void(const std::string &)> loginfo_;
  std::function<void(const std::string &)> logwarn_;
  std::function<void(const std::string &)> logerror_;

  // Map parameters
  bool map_valid_;
  geometry_msgs::msg::Point map_origin_;
  double map_resolution_;
  std::string map_info_str_;

  // Pose parameters
  bool pose_valid_;
  Eigen::Vector3d current_position_;
  Eigen::Vector3d initial_position_;

  // Lap counting
  bool was_at_init_pos_;
  double x_max_diff_;
  double y_max_diff_;
  double theta_max_diff_;
  int lap_count_;

  // Map data
  uint32_t map_width_;
  uint32_t map_height_;
  std::vector<int8_t> map_occupancy_grid_;

  // Driven path
  std::vector<Eigen::Vector3d> cent_driven_;

  // Estimated lap time
  std_msgs::msg::Float32 est_lap_time_;

  // Map editor mode
  bool ready_to_plan_;
  bool logged_once_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__GLOBAL_PLANNER_LOGIC_HPP_
