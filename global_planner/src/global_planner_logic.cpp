#include "global_planner/global_planner_logic.hpp"
#include "global_planner/global_planner_utils.hpp"
#include "global_planner/readwrite_global_waypoints.hpp"
#include "global_planner/trajectory_optimizer_wrapper.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <filesystem>
#include <limits>
#include <yaml-cpp/yaml.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdlib>

namespace global_planner
{

GlobalPlannerLogic::GlobalPlannerLogic(
  double safety_width,
  double safety_width_sp,
  double occupancy_grid_threshold,
  bool map_editor_mode,
  bool create_map,
  const std::string & map_name,
  const std::string & map_dir,
  const std::string & finish_script_path,
  const std::string & input_path,
  bool show_plots,
  int filter_kernel_size,
  int required_laps,
  bool reverse_mapping,
  std::function<void(const std::string &)> loginfo,
  std::function<void(const std::string &)> logwarn,
  std::function<void(const std::string &)> logerror)
: safety_width_(safety_width),
  safety_width_sp_(safety_width_sp),
  occupancy_grid_threshold_(occupancy_grid_threshold),
  filter_kernel_size_(filter_kernel_size),
  show_plots_(show_plots),
  map_editor_mode_(map_editor_mode),
  compute_global_traj_(!map_editor_mode),
  create_map_(create_map),
  map_name_(map_name),
  map_dir_(map_dir),
  script_path_(finish_script_path),
  input_path_(input_path),
  required_laps_(required_laps),
  reverse_mapping_(reverse_mapping),
  watershed_(true),
  loginfo_(loginfo),
  logwarn_(logwarn),
  logerror_(logerror),
  map_valid_(false),
  pose_valid_(false),
  current_position_(Eigen::Vector3d::Zero()),
  initial_position_(Eigen::Vector3d::Zero()),
  was_at_init_pos_(true),
  x_max_diff_(0.5),
  y_max_diff_(0.5),
  theta_max_diff_(M_PI / 2.0),
  lap_count_(0),
  ready_to_plan_(false),
  logged_once_(false)
{
  // Auto-detect input_path if not provided
  if (input_path_.empty()) {
    // Try to find the global_racetrajectory_optimization module's inputs directory
    std::vector<std::filesystem::path> possible_paths;
    
    // Try ament_index for installed location first
    try {
      std::string share_dir = ament_index_cpp::get_package_share_directory("global_planner");
      std::filesystem::path installed_path = std::filesystem::path(share_dir) / "python_modules" / "global_racetrajectory_optimization" / "global_racetrajectory_optimization" / "inputs";
      possible_paths.push_back(installed_path);
    } catch (...) {
      // Ignore if package not found
    }
    
    // Add source location paths
    const char * home = std::getenv("HOME");
    if (home) {
      possible_paths.push_back(
        std::filesystem::path(home) / "shared_dir" / "planner" / "global_planner" / "python_modules" / 
        "global_racetrajectory_optimization" / "global_racetrajectory_optimization" / "inputs");
    }
    possible_paths.push_back(
      std::filesystem::path("/home/misys/forza_ws/race_stack/stack_master/src/race_stack/stack_master/planner/global_planner/python_modules/global_racetrajectory_optimization/global_racetrajectory_optimization/inputs"));
    possible_paths.push_back(
      std::filesystem::path("/home/misys/forza_ws/race_stack/src/race_stack/stack_master/planner/global_planner/python_modules/global_racetrajectory_optimization/global_racetrajectory_optimization/inputs"));
    
    for (const auto & path : possible_paths) {
      if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
        input_path_ = path.string();
        if (loginfo_) {
          loginfo_("Auto-detected input_path: " + input_path_);
        }
        break;
      }
    }
    
    if (input_path_.empty() && logwarn_) {
      logwarn_("Could not auto-detect input_path. Trajectory optimizer may fail.");
    }
  }

  if (!create_map_) {
    std::filesystem::path yaml_path = std::filesystem::path(map_dir_) / (map_name_ + ".yaml");
    if (!std::filesystem::exists(yaml_path)) {
      // Try to find YAML in alternative locations (map_server might have it)
      // This is just for reading map metadata, not critical if not found
      if (logwarn_) {
        logwarn_("Map YAML not found at: " + yaml_path.string() + ". Map metadata will be read from map_server topic.");
      }
    } else {
      try {
        YAML::Node config = YAML::LoadFile(yaml_path.string());
        map_resolution_ = config["resolution"].as<double>();
        map_origin_.x = config["origin"][0].as<double>();
        map_origin_.y = config["origin"][1].as<double>();
        map_valid_ = true;
        initial_position_ = Eigen::Vector3d(map_origin_.x, map_origin_.y, 0.0);
      } catch (const std::exception & e) {
        if (logwarn_) {
          logwarn_("Failed to load map YAML: " + std::string(e.what()) + ". Map metadata will be read from map_server topic.");
        }
      }
    }
  } else {
    ready_to_plan_ = false;
  }
}

std::pair<bool, std::string> GlobalPlannerLogic::global_plan_logic()
{
  if (!create_map_) {
    if (loginfo_) {
      loginfo_("Not creating map, only computing trajectory.");
    }
    compute_global_trajectory(0.0);
    if (loginfo_) {
      loginfo_("Successfully computed trajectory.");
    }
    return {true, map_name_};
  }

  if (!pose_valid_ || !map_valid_) {
    return {false, ""};
  }

  if (!logged_once_) {
    if (loginfo_) {
      loginfo_("Map and pose received.");
    }
    logged_once_ = true;
  }

  if (map_editor_mode_) {
    if (!logged_once_) {
      if (loginfo_) {
        loginfo_("Map editor ready");
      }
    }
    // In map editor mode, wait for user input
    // For now, we'll skip the interactive part and proceed
    ready_to_plan_ = true;
  } else {
    if (!logged_once_) {
      if (loginfo_) {
        loginfo_("Waiting for the car to complete a lap");
      }
    }
    update_lap_count();

    if (lap_count_ < required_laps_) {
      return {false, ""};
    }
  }

  double cent_length = 0.0;
  if (required_laps_ > 0 && !cent_driven_.empty()) {
    for (size_t i = 1; i < cent_driven_.size(); ++i) {
      Eigen::Vector2d diff = cent_driven_[i].head<2>() - cent_driven_[i - 1].head<2>();
      cent_length += diff.norm();
    }
    std::string cent_len_str = "Approximate centerline length: " +
      std::to_string(static_cast<int>(cent_length * 100) / 100.0) + "m; ";
    map_info_str_ += cent_len_str;
  }

  if (ready_to_plan_) {
    initial_position_ = current_position_;
    cv::Mat filtered_map = filter_map_occupancy_grid();
    save_map_png(filtered_map);
    if (loginfo_) {
      loginfo_("Saved map.");
    }

    if (compute_global_traj_) {
      if (compute_global_trajectory(cent_length)) {
        if (loginfo_) {
          loginfo_("Successfully computed global waypoints.");
        }
        return {true, map_name_};
      } else {
        if (logwarn_) {
          logwarn_("Was unable to compute global waypoints!");
        }
        ready_to_plan_ = false;
        return {false, ""};
      }
    }
    return {true, map_name_};
  }

  return {false, ""};
}

bool GlobalPlannerLogic::compute_global_trajectory(double cent_length)
{
  // Open PNG map from file
  std::filesystem::path img_path = std::filesystem::path(map_dir_) / (map_name_ + ".png");
  cv::Mat img = cv::imread(img_path.string(), cv::IMREAD_GRAYSCALE);
  if (img.empty()) {
    if (logerror_) {
      logerror_("Failed to load map image: " + img_path.string());
    }
    return false;
  }
  cv::flip(img, img, 0);
  cv::Mat filtered_map = img;

  // Skeletonize - use distance transform approach
  cv::Mat dist;
  cv::distanceTransform(filtered_map, dist, cv::DIST_L2, 5);
  cv::Mat skeleton;
  cv::threshold(dist, skeleton, 0.5, 255, cv::THRESH_BINARY);
  skeleton.convertTo(skeleton, CV_8U);
  
  // Additional skeletonization using morphological operations
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::Mat temp;
  cv::morphologyEx(skeleton, temp, cv::MORPH_OPEN, kernel);
  skeleton = temp;

  // Extract centerline
  std::vector<cv::Point2i> centerline;
  try {
    centerline = extract_centerline(skeleton, cent_length, map_resolution_, map_editor_mode_);
  } catch (const std::exception & e) {
    if (logwarn_) {
      logwarn_(std::string("Error extracting centerline: ") + e.what());
    }
    return false;
  }

  if (centerline.empty()) {
    if (logerror_) {
      logerror_("Centerline extraction failed: empty centerline. This may indicate:");
      logerror_("  - Map image has no valid track area (all black or all white)");
      logerror_("  - Threshold parameters are incorrect (occupancy_grid_threshold, filter_kernel_size)");
      logerror_("  - Track is too narrow or map resolution is too low");
      logerror_("  - Try adjusting map preprocessing parameters or check the map image quality");
    }
    return false;
  }
  
  if (loginfo_) {
    loginfo_("Extracted centerline with " + std::to_string(centerline.size()) + " points");
  }

  std::vector<cv::Point2f> centerline_smooth = smooth_centerline(centerline);

  // Convert centerline from cells to meters
  std::vector<Eigen::Vector2d> centerline_meter;
  for (const auto & pt : centerline_smooth) {
    double x = pt.x * map_resolution_ + map_origin_.x;
    double y = pt.y * map_resolution_ + map_origin_.y;
    centerline_meter.push_back(Eigen::Vector2d(x, y));
  }

  if (centerline_meter.empty()) {
    if (logerror_) {
      logerror_("Centerline conversion failed: empty centerline_meter");
    }
    return false;
  }

  // Interpolate centerline (simplified - would need proper interpolation)
  // For now, use the existing points

  // Find closest point to initial position
  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;
  for (size_t i = 0; i < centerline_meter.size(); ++i) {
    double dist = (centerline_meter[i] - initial_position_.head<2>()).norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  // Check direction
  size_t prev_idx = (min_idx == 0) ? centerline_meter.size() - 1 : min_idx - 1;
  double cent_direction = std::atan2(
    centerline_meter[min_idx].y() - centerline_meter[prev_idx].y(),
    centerline_meter[min_idx].x() - centerline_meter[prev_idx].x());

  // Flip centerline if directions don't match
  if (!compare_direction(cent_direction, initial_position_[2])) {
    std::reverse(centerline_smooth.begin(), centerline_smooth.end());
    std::reverse(centerline_meter.begin(), centerline_meter.end());
  }

  if (reverse_mapping_) {
    std::reverse(centerline_smooth.begin(), centerline_smooth.end());
    std::reverse(centerline_meter.begin(), centerline_meter.end());
    if (loginfo_) {
      loginfo_("Centerline flipped");
    }
  }

  // Extract track bounds
  std::vector<Eigen::Vector2d> bound_r, bound_l;
  cv::Mat dist_transform;
  try {
    auto bounds = extract_track_bounds(
      centerline, filtered_map, map_editor_mode_, map_resolution_,
      map_origin_, initial_position_, show_plots_);
    bound_r = bounds.first;
    bound_l = bounds.second;
    if (loginfo_) {
      loginfo_("Using watershed for track bound extraction...");
    }
  } catch (const std::exception & e) {
    if (logwarn_) {
      logwarn_("More than two track bounds detected with watershed algorithm");
      loginfo_("Trying with simple distance transform...");
    }
    watershed_ = false;
    cv::distanceTransform(filtered_map, dist_transform, cv::DIST_L2, 5);
  }

  // Add distance to centerline
  std::vector<Eigen::Vector4d> cent_with_dist;
  try {
    cent_with_dist = add_dist_to_cent(
      centerline_smooth, centerline_meter, map_resolution_, safety_width_,
      show_plots_, dist_transform, bound_r, bound_l, reverse_mapping_);
  } catch (const std::exception & e) {
    if (logerror_) {
      logerror_(std::string("Failed to add distance to centerline: ") + e.what());
    }
    return false;
  }

  if (cent_with_dist.empty()) {
    if (logerror_) {
      logerror_("Centerline with distance is empty");
    }
    return false;
  }

  // Validate track widths and ensure minimum
  double min_track_width = std::max(safety_width_ * 2.0, 0.5);  // Minimum 0.5m or 2*safety_width
  bool has_valid_width = false;
  double min_width_found = std::numeric_limits<double>::max();
  
  for (const auto & pt : cent_with_dist) {
    double total_width = pt[2] + pt[3];
    if (total_width > 0.1) {  // At least 10cm total width
      has_valid_width = true;
      if (total_width < min_width_found) {
        min_width_found = total_width;
      }
    }
  }
  
  if (!has_valid_width || min_width_found < min_track_width) {
    if (logwarn_) {
      logwarn_("Warning: Track widths are too small. Minimum width found: " + 
               std::to_string(min_width_found) + "m. Using minimum: " + 
               std::to_string(min_track_width) + "m");
    }
    // Ensure all points have minimum width
    for (auto & pt : cent_with_dist) {
      if (pt[2] < min_track_width / 2.0) {
        pt[2] = min_track_width / 2.0;
      }
      if (pt[3] < min_track_width / 2.0) {
        pt[3] = min_track_width / 2.0;
      }
    }
  }

  // Write centerline
  auto [centerline_waypoints, centerline_markers] = write_centerline(cent_with_dist);
  
  // Verify centerline file was written
  std::string iqp_centerline_path = (std::filesystem::path(std::getenv("HOME")) / ".ros" / "map_centerline.csv").string();
  std::string sp_centerline_path = (std::filesystem::path(std::getenv("HOME")) / ".ros" / "map_centerline_2.csv").string();
  
  if (!std::filesystem::exists(iqp_centerline_path)) {
    if (logerror_) {
      logerror_("Failed to write centerline file: " + iqp_centerline_path);
    }
    return false;
  }
  
  // Check if centerline file has content
  std::ifstream check_file(iqp_centerline_path);
  std::string first_line;
  std::getline(check_file, first_line);
  check_file.close();
  
  if (first_line.empty()) {
    if (logerror_) {
      logerror_("Centerline file is empty: " + iqp_centerline_path);
      logerror_("This indicates track extraction failed. Check map image and preprocessing parameters.");
    }
    return false;
  }
  
  if (loginfo_) {
    loginfo_("Centerline file written successfully: " + iqp_centerline_path + " (" + std::to_string(cent_with_dist.size()) + " points)");
  }

  // Call trajectory optimizer
  
  std::vector<Eigen::VectorXd> global_trajectory_iqp;
  std::vector<Eigen::VectorXd> global_trajectory_sp;
  std::vector<Eigen::Vector2d> bound_r_iqp, bound_l_iqp;
  std::vector<Eigen::Vector2d> bound_r_sp, bound_l_sp;
  double est_t_iqp = 0.0;
  double est_t_sp = 0.0;

  try {
    if (loginfo_) {
      loginfo_("Start Global Trajectory optimization with iterative minimum curvature...");
    }
    
    auto result_iqp = call_trajectory_optimizer(
      input_path_, iqp_centerline_path, "mincurv_iqp", safety_width_,
      show_plots_ && !map_editor_mode_);
    
    global_trajectory_iqp = result_iqp.trajectory;
    bound_r_iqp = result_iqp.bound_r;
    bound_l_iqp = result_iqp.bound_l;
    est_t_iqp = result_iqp.est_lap_time;
    
    // Use watershed bounds if available
    if (watershed_ && !bound_r.empty() && !bound_l.empty()) {
      bound_r_iqp = bound_r;
      bound_l_iqp = bound_l;
    }
  } catch (const std::exception & e) {
    if (logwarn_) {
      logwarn_(std::string("Error during iterative minimum curvature optimization: ") + e.what());
      loginfo_("Try again later!");
    }
    return false;
  }

  map_info_str_ += "IQP estimated lap time: " + std::to_string(est_t_iqp) + "s; ";
  
  // Find max speed
  double max_speed_iqp = 0.0;
  for (const auto & traj : global_trajectory_iqp) {
    if (traj.size() > 5 && traj[5] > max_speed_iqp) {
      max_speed_iqp = traj[5];
    }
  }
  map_info_str_ += "IQP maximum speed: " + std::to_string(max_speed_iqp) + "m/s; ";

  // Calculate distances to bounds for IQP trajectory
  std::vector<Eigen::Vector2d> traj_points_iqp;
  for (const auto & traj : global_trajectory_iqp) {
    if (traj.size() >= 3) {
      traj_points_iqp.push_back(Eigen::Vector2d(traj[1], traj[2]));
    }
  }
  auto [d_right_iqp, d_left_iqp] = dist_to_bounds(
    traj_points_iqp, bound_r_iqp, bound_l_iqp, centerline_meter, safety_width_,
    show_plots_, reverse_mapping_);

  auto [global_traj_wpnts_iqp, global_traj_markers_iqp] = create_wpnts_markers(
    global_trajectory_iqp, d_right_iqp, d_left_iqp);

  if (loginfo_) {
    loginfo_("Done with iterative minimum curvature optimization");
    loginfo_("Lap Completed now publishing global waypoints");
  }

  // Shortest path optimization
  if (loginfo_) {
    loginfo_("Start reverse Global Trajectory optimization with shortest path...");
    loginfo_("Start Global Trajectory optimization with iterative minimum curvature for overtaking...");
  }
  
  try {
    // First call mincurv_iqp with safety_width_sp for overtaking
    auto result_iqp_ot = call_trajectory_optimizer(
      input_path_, iqp_centerline_path, "mincurv_iqp", safety_width_sp_,
      show_plots_ && !map_editor_mode_);
    
    // Use new iqp path as centerline
    std::vector<cv::Point2f> new_centerline_smooth;
    std::vector<Eigen::Vector2d> new_centerline_meter;
    for (const auto & traj : result_iqp_ot.trajectory) {
      if (traj.size() >= 3) {
        new_centerline_smooth.push_back(cv::Point2f(
          (traj[1] - map_origin_.x) / map_resolution_,
          (traj[2] - map_origin_.y) / map_resolution_));
        new_centerline_meter.push_back(Eigen::Vector2d(traj[1], traj[2]));
      }
    }
    
    std::vector<Eigen::Vector4d> new_cent_with_dist = add_dist_to_cent(
      new_centerline_smooth, new_centerline_meter, map_resolution_, safety_width_sp_,
      show_plots_, cv::Mat(), bound_r, bound_l, reverse_mapping_);
    
    auto [_, new_centerline_markers] = write_centerline(new_cent_with_dist, true);
    
    // Now call shortest_path
    auto result_sp = call_trajectory_optimizer(
      input_path_, sp_centerline_path, "shortest_path", safety_width_sp_,
      show_plots_ && !map_editor_mode_);
    
    global_trajectory_sp = result_sp.trajectory;
    bound_r_sp = result_sp.bound_r;
    bound_l_sp = result_sp.bound_l;
    est_t_sp = result_sp.est_lap_time;
    
    // Use watershed bounds if available
    if (watershed_ && !bound_r.empty() && !bound_l.empty()) {
      bound_r_sp = bound_r;
      bound_l_sp = bound_l;
    }
  } catch (const std::exception & e) {
    if (logwarn_) {
      logwarn_(std::string("Error during shortest path optimization: ") + e.what());
    }
    return false;
  }
  
  // Calculate distances to bounds for SP trajectory
  std::vector<Eigen::Vector2d> traj_points_sp;
  for (const auto & traj : global_trajectory_sp) {
    if (traj.size() >= 3) {
      traj_points_sp.push_back(Eigen::Vector2d(traj[1], traj[2]));
    }
  }
  auto [d_right_sp, d_left_sp] = dist_to_bounds(
    traj_points_sp, bound_r_sp, bound_l_sp, centerline_meter, safety_width_sp_,
    show_plots_, reverse_mapping_);

  auto [global_traj_wpnts_sp, global_traj_markers_sp] = create_wpnts_markers(
    global_trajectory_sp, d_right_sp, d_left_sp, true);

  est_lap_time_.data = est_t_sp;
  map_info_str_ += "SP estimated lap time: " + std::to_string(est_t_sp) + "s; ";
  
  // Find max speed for SP
  double max_speed_sp = 0.0;
  for (const auto & traj : global_trajectory_sp) {
    if (traj.size() > 5 && traj[5] > max_speed_sp) {
      max_speed_sp = traj[5];
    }
  }
  map_info_str_ += "SP maximum speed: " + std::to_string(max_speed_sp) + "m/s; ";

  // Publish track bounds
  visualization_msgs::msg::MarkerArray bounds_markers = publish_track_bounds(
    bound_r_iqp, bound_l_iqp, false);
  
  if (loginfo_) {
    loginfo_("Done with shortest path optimization");
    loginfo_("Lap Completed now publishing shortest path global waypoints");
  }

  // Save to JSON
  write_global_waypoints(
    map_dir_, map_info_str_, est_lap_time_,
    centerline_markers, centerline_waypoints,
    global_traj_markers_iqp, global_traj_wpnts_iqp,
    global_traj_markers_sp, global_traj_wpnts_sp,
    bounds_markers);

  return true;
}

cv::Mat GlobalPlannerLogic::filter_map_occupancy_grid()
{
  cv::Mat original_map(map_height_, map_width_, CV_8SC1, map_occupancy_grid_.data());

  // Mark unknown (-1) as occupied (100)
  cv::Mat bw;
  original_map.copyTo(bw);
  bw.setTo(100, bw == -1);

  // Binarize map
  cv::Mat binary;
  cv::threshold(bw, binary, static_cast<int>(occupancy_grid_threshold_ * 100), 255, cv::THRESH_BINARY);
  binary.convertTo(binary, CV_8U);

  // Filtering with morphological opening
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_RECT, cv::Size(filter_kernel_size_, filter_kernel_size_));
  cv::Mat filtered_map;
  cv::morphologyEx(binary, filtered_map, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);

  return filtered_map;
}

void GlobalPlannerLogic::save_map_png(const cv::Mat & filtered_map)
{
  try {
    std::filesystem::create_directories(map_dir_);
  } catch (const std::exception & e) {
    if (logerror_) {
      logerror_("Could not create the folder " + map_dir_ + " because it already exists.");
    }
    throw;
  }

  if (loginfo_) {
    loginfo_("Successfully created the folder " + map_dir_);
  }

  // Save map as PNG
  std::filesystem::path img_path = std::filesystem::path(map_dir_) / (map_name_ + ".png");
  cv::Mat flipped_map;
  cv::flip(filtered_map, flipped_map, 0);
  cv::imwrite(img_path.string(), flipped_map);

  // Save pf_map
  std::filesystem::path pf_img_path = std::filesystem::path(map_dir_) / "pf_map.png";
  cv::imwrite(pf_img_path.string(), flipped_map);

  // Save YAML
  std::filesystem::path yaml_path = std::filesystem::path(map_dir_) / (map_name_ + ".yaml");
  YAML::Node dict_map;
  dict_map["image"] = map_name_ + ".png";
  dict_map["resolution"] = map_resolution_;
  dict_map["origin"] = YAML::Node(YAML::NodeType::Sequence);
  dict_map["origin"].push_back(map_origin_.x);
  dict_map["origin"].push_back(map_origin_.y);
  dict_map["origin"].push_back(0.0);
  dict_map["negate"] = 0;
  dict_map["occupied_thresh"] = 0.65;
  dict_map["free_thresh"] = 0.196;

  std::ofstream yaml_file(yaml_path);
  yaml_file << dict_map;
  yaml_file.close();

  // Call finish script (would need to execute subprocess)
  // For now, skip this

  if (loginfo_) {
    loginfo_("PNG and YAML file created and saved in the " + map_dir_ + " folder");
  }
}

void GlobalPlannerLogic::update_lap_count()
{
  bool is_at_init_pos = at_init_pos_check();
  if (is_at_init_pos && !was_at_init_pos_) {
    was_at_init_pos_ = true;
    lap_count_++;
    if (loginfo_) {
      loginfo_("Laps completed " + std::to_string(lap_count_));
    }
  } else if (!is_at_init_pos) {
    was_at_init_pos_ = false;
  }
}

bool GlobalPlannerLogic::at_init_pos_check() const
{
  double x_diff = std::abs(current_position_[0] - initial_position_[0]);
  double y_diff = std::abs(current_position_[1] - initial_position_[1]);

  double theta_diff0 = std::abs(current_position_[2] - initial_position_[2]);
  double theta_diff1 = 2 * M_PI - theta_diff0;
  double theta_diff = std::min(theta_diff0, theta_diff1);

  return (x_diff < x_max_diff_) && (y_diff < y_max_diff_) && (theta_diff < theta_max_diff_);
}

void GlobalPlannerLogic::update_map(const nav_msgs::msg::OccupancyGrid & msg)
{
  if (create_map_) {
    map_width_ = msg.info.width;
    map_height_ = msg.info.height;
    map_resolution_ = msg.info.resolution;
    map_origin_ = msg.info.origin.position;
    map_occupancy_grid_ = std::vector<int8_t>(msg.data.begin(), msg.data.end());
  }
  map_valid_ = true;
}

void GlobalPlannerLogic::update_pose(const geometry_msgs::msg::PoseStamped & msg)
{
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;

  tf2::Quaternion q(
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double theta = yaw;

  if (!pose_valid_) {
    pose_valid_ = true;
    initial_position_ = Eigen::Vector3d(x, y, theta);
  }

  current_position_ = Eigen::Vector3d(x, y, theta);

  if (lap_count_ == 0) {
    cent_driven_.push_back(current_position_);
  }
}

}  // namespace global_planner
