#include "global_planner/global_planner_utils.hpp"
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <cstdlib>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core.hpp>

namespace global_planner
{

std::string get_data_path(const std::string & subpath)
{
  try {
    std::string share_dir = ament_index_cpp::get_package_share_directory("stack_master");
    std::filesystem::path path(share_dir);
    path = path.parent_path().parent_path().parent_path() / "src/race_stack/stack_master" / subpath;
    return path.string();
  } catch (...) {
    return subpath;
  }
}

std::vector<cv::Point2i> extract_centerline(
  const cv::Mat & skeleton,
  double cent_length,
  double map_resolution,
  bool map_editor_mode)
{
  std::vector<std::vector<cv::Point>> contours;
  cv::Mat hierarchy;
  cv::findContours(skeleton, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

  // Save all closed contours
  std::vector<std::vector<cv::Point>> closed_contours;
  for (size_t i = 0; i < contours.size(); ++i) {
    int parent = hierarchy.at<cv::Vec4i>(i)[3];
    int child = hierarchy.at<cv::Vec4i>(i)[2];
    if (parent >= 0 || child >= 0) {  // Not opened
      closed_contours.push_back(contours[i]);
    }
  }

  if (closed_contours.empty()) {
    throw std::runtime_error("No closed contours");
  }

  // Calculate line length for every contour
  std::vector<double> line_lengths(closed_contours.size(), std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < closed_contours.size(); ++i) {
    double line_length = 0.0;
    for (size_t k = 0; k < closed_contours[i].size(); ++k) {
      size_t prev_k = (k == 0) ? closed_contours[i].size() - 1 : k - 1;
      double dx = closed_contours[i][k].x - closed_contours[i][prev_k].x;
      double dy = closed_contours[i][k].y - closed_contours[i][prev_k].y;
      line_length += std::sqrt(dx * dx + dy * dy);
    }
    line_length *= map_resolution;

    if (std::abs(cent_length / line_length - 1.0) < 0.15) {
      line_lengths[i] = line_length;
    } else if (map_editor_mode || cent_length == 0.0) {
      line_lengths[i] = line_length;
    }
  }

  auto min_it = std::min_element(line_lengths.begin(), line_lengths.end());
  if (*min_it == std::numeric_limits<double>::infinity()) {
    throw std::runtime_error("Only invalid closed contour line lengths");
  }

  size_t min_index = std::distance(line_lengths.begin(), min_it);
  std::vector<cv::Point2i> centerline;
  for (const auto & pt : closed_contours[min_index]) {
    centerline.push_back(cv::Point2i(pt.x, pt.y));
  }

  return centerline;
}

std::vector<cv::Point2f> smooth_centerline(const std::vector<cv::Point2i> & centerline)
{
  // Simple moving average smoothing (Savitzky-Golay would require additional library)
  size_t centerline_length = centerline.size();
  int filter_length = 21;
  if (centerline_length > 2000) {
    filter_length = static_cast<int>(centerline_length / 200) * 10 + 1;
  } else if (centerline_length > 1000) {
    filter_length = 81;
  } else if (centerline_length > 500) {
    filter_length = 41;
  }

  std::vector<cv::Point2f> smoothed(centerline.size());
  int half_window = filter_length / 2;

  for (size_t i = 0; i < centerline.size(); ++i) {
    float sum_x = 0.0f, sum_y = 0.0f;
    int count = 0;
    for (int j = -half_window; j <= half_window; ++j) {
      int idx = (static_cast<int>(i) + j + static_cast<int>(centerline.size())) %
        static_cast<int>(centerline.size());
      sum_x += centerline[idx].x;
      sum_y += centerline[idx].y;
      count++;
    }
    smoothed[i] = cv::Point2f(sum_x / count, sum_y / count);
  }

  return smoothed;
}

std::pair<std::vector<Eigen::Vector2d>, std::vector<Eigen::Vector2d>> extract_track_bounds(
  const std::vector<cv::Point2i> & centerline,
  const cv::Mat & filtered_bw,
  bool /* map_editor_mode */,
  double map_resolution,
  const geometry_msgs::msg::Point & map_origin,
  const Eigen::Vector3d & initial_position,
  bool /* show_plots */)
{
  // Create black and white image of centerline
  cv::Mat cent_img = cv::Mat::zeros(filtered_bw.size(), CV_8UC1);
  std::vector<std::vector<cv::Point>> centerline_contour = {centerline};
  cv::drawContours(cent_img, centerline_contour, 0, cv::Scalar(255), 2, cv::LINE_8);

  // Create markers for watershed algorithm
  cv::Mat cent_markers;
  cv::connectedComponents(cent_img, cent_markers);
  cent_markers.convertTo(cent_markers, CV_32S);

  // Apply watershed algorithm
  // OpenCV watershed modifies markers in-place and requires 32SC1 type
  cv::Mat dist_transform;
  cv::distanceTransform(filtered_bw, dist_transform, cv::DIST_L2, 5);
  cv::Mat dist_neg;
  dist_transform.convertTo(dist_neg, CV_32F);
  dist_neg = -dist_neg;
  
  // Apply mask: set background pixels (where filtered_bw is 0) to -1 in markers
  // This is equivalent to mask parameter in skimage.watershed
  cv::Mat mask_inv;
  cv::bitwise_not(filtered_bw, mask_inv);
  cent_markers.setTo(-1, mask_inv);
  
  // watershed modifies cent_markers in-place
  // The result will be in cent_markers, with -1 for background/watershed lines
  cv::watershed(dist_neg, cent_markers);
  cv::Mat labels = cent_markers;

  std::vector<std::vector<cv::Point>> closed_contours;
  for (int label = 1; label <= cv::countNonZero(labels > 0); ++label) {
    cv::Mat mask = (labels == label);
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat hierarchy;
    cv::findContours(mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

    for (size_t i = 0; i < contours.size(); ++i) {
      int parent = hierarchy.at<cv::Vec4i>(i)[3];
      int child = hierarchy.at<cv::Vec4i>(i)[2];
      if (parent >= 0 || child >= 0) {
        closed_contours.push_back(contours[i]);
      }
    }
  }

  if (closed_contours.size() != 2) {
    throw std::runtime_error("More than two track bounds detected! Check input");
  }

  // Find longest and shortest contours
  auto longest_it = std::max_element(
    closed_contours.begin(), closed_contours.end(),
    [](const std::vector<cv::Point> & a, const std::vector<cv::Point> & b) {
      return a.size() < b.size();
    });
  auto shortest_it = std::min_element(
    closed_contours.begin(), closed_contours.end(),
    [](const std::vector<cv::Point> & a, const std::vector<cv::Point> & b) {
      return a.size() < b.size();
    });

  std::vector<Eigen::Vector2d> bound_long, bound_short;
  for (const auto & pt : *longest_it) {
    double x = pt.x * map_resolution + map_origin.x;
    double y = pt.y * map_resolution + map_origin.y;
    bound_long.push_back(Eigen::Vector2d(x, y));
  }
  for (const auto & pt : *shortest_it) {
    double x = pt.x * map_resolution + map_origin.x;
    double y = pt.y * map_resolution + map_origin.y;
    bound_short.push_back(Eigen::Vector2d(x, y));
  }

  // Determine which is right and left based on initial position
  double min_dist = std::numeric_limits<double>::max();
  size_t min_idx = 0;
  for (size_t i = 0; i < bound_long.size(); ++i) {
    double dist = (Eigen::Vector2d(bound_long[i].x() - initial_position[0],
                                    bound_long[i].y() - initial_position[1])).norm();
    if (dist < min_dist) {
      min_dist = dist;
      min_idx = i;
    }
  }

  size_t prev_idx = (min_idx == 0) ? bound_long.size() - 1 : min_idx - 1;
  double bound_direction = std::atan2(
    bound_long[min_idx].y() - bound_long[prev_idx].y(),
    bound_long[min_idx].x() - bound_long[prev_idx].x());

  double norm_angle_right = initial_position[2] - M_PI;
  if (norm_angle_right < -M_PI) {
    norm_angle_right += 2 * M_PI;
  }

  std::vector<Eigen::Vector2d> bound_right, bound_left;
  if (compare_direction(norm_angle_right, bound_direction)) {
    bound_right = bound_long;
    bound_left = bound_short;
  } else {
    bound_right = bound_short;
    bound_left = bound_long;
  }

  return {bound_right, bound_left};
}

std::pair<std::vector<double>, std::vector<double>> dist_to_bounds(
  const std::vector<Eigen::Vector2d> & trajectory,
  const std::vector<Eigen::Vector2d> & bound_r,
  const std::vector<Eigen::Vector2d> & bound_l,
  const std::vector<Eigen::Vector2d> & /* centerline */,
  double /* safety_width */,
  bool /* show_plots */,
  bool reverse)
{
  std::vector<double> dists_right(trajectory.size());
  std::vector<double> dists_left(trajectory.size());

  for (size_t i = 0; i < trajectory.size(); ++i) {
    double min_dist_r = std::numeric_limits<double>::max();
    double min_dist_l = std::numeric_limits<double>::max();

    for (const auto & pt : bound_r) {
      double dist = (trajectory[i] - pt).norm();
      if (dist < min_dist_r) {
        min_dist_r = dist;
      }
    }

    for (const auto & pt : bound_l) {
      double dist = (trajectory[i] - pt).norm();
      if (dist < min_dist_l) {
        min_dist_l = dist;
      }
    }

    dists_right[i] = min_dist_r;
    dists_left[i] = min_dist_l;
  }

  if (reverse) {
    return {dists_left, dists_right};
  } else {
    return {dists_right, dists_left};
  }
}

std::vector<Eigen::Vector4d> add_dist_to_cent(
  const std::vector<cv::Point2f> & centerline_smooth,
  const std::vector<Eigen::Vector2d> & centerline_meter,
  double map_resolution,
  double safety_width,
  bool show_plots,
  const cv::Mat & dist_transform,
  const std::vector<Eigen::Vector2d> & bound_r,
  const std::vector<Eigen::Vector2d> & bound_l,
  bool reverse)
{
  std::vector<Eigen::Vector4d> centerline_comp(centerline_meter.size());
  const double min_track_width = std::max(safety_width * 2.0, 0.5);  // Minimum 0.5m or 2*safety_width

  if (!dist_transform.empty()) {
    for (size_t i = 0; i < centerline_smooth.size(); ++i) {
      int y = static_cast<int>(centerline_smooth[i].y);
      int x = static_cast<int>(centerline_smooth[i].x);
      if (y >= 0 && y < dist_transform.rows && x >= 0 && x < dist_transform.cols) {
        double width = dist_transform.at<float>(y, x) * map_resolution;
        // Ensure minimum track width
        if (width < min_track_width) {
          width = min_track_width;
        }
        centerline_comp[i] = Eigen::Vector4d(
          centerline_meter[i].x(), centerline_meter[i].y(), width, width);
      } else {
        // Out of bounds, use minimum width
        centerline_comp[i] = Eigen::Vector4d(
          centerline_meter[i].x(), centerline_meter[i].y(), min_track_width, min_track_width);
      }
    }
  } else if (!bound_r.empty() && !bound_l.empty()) {
    auto [d_right, d_left] = dist_to_bounds(
      centerline_meter, bound_r, bound_l, centerline_meter, safety_width, show_plots, reverse);
    for (size_t i = 0; i < centerline_meter.size(); ++i) {
      // Ensure minimum distances
      double d_r = std::max(d_right[i], min_track_width / 2.0);
      double d_l = std::max(d_left[i], min_track_width / 2.0);
      centerline_comp[i] = Eigen::Vector4d(
        centerline_meter[i].x(), centerline_meter[i].y(), d_r, d_l);
    }
  } else {
    // Fallback: use minimum track width for all points
    for (size_t i = 0; i < centerline_meter.size(); ++i) {
      centerline_comp[i] = Eigen::Vector4d(
        centerline_meter[i].x(), centerline_meter[i].y(), min_track_width, min_track_width);
    }
  }

  return centerline_comp;
}

std::pair<f110_msgs::msg::WpntArray, visualization_msgs::msg::MarkerArray> write_centerline(
  const std::vector<Eigen::Vector4d> & centerline,
  bool sp_bool)
{
  f110_msgs::msg::WpntArray centerline_wpnts;
  visualization_msgs::msg::MarkerArray centerline_markers;

  std::string filename = sp_bool ? "map_centerline_2.csv" : "map_centerline.csv";
  std::filesystem::path home_dir = std::filesystem::path(std::getenv("HOME"));
  std::filesystem::path csv_path = home_dir / ".ros" / filename;

  std::ofstream file(csv_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << csv_path << std::endl;
    return {centerline_wpnts, centerline_markers};
  }

  for (size_t i = 0; i < centerline.size(); ++i) {
    double x_m = centerline[i][0];
    double y_m = centerline[i][1];
    double width_tr_right_m = centerline[i][2];
    double width_tr_left_m = centerline[i][3];

    file << x_m << "," << y_m << "," << width_tr_right_m << "," << width_tr_left_m << "\n";

    visualization_msgs::msg::Marker cent_marker;
    cent_marker.header.frame_id = "map";
    cent_marker.type = visualization_msgs::msg::Marker::SPHERE;
    cent_marker.scale.x = 0.05;
    cent_marker.scale.y = 0.05;
    cent_marker.scale.z = 0.05;
    cent_marker.color.a = 1.0;
    cent_marker.color.b = 1.0;
    cent_marker.id = static_cast<int>(i);
    cent_marker.pose.position.x = x_m;
    cent_marker.pose.position.y = y_m;
    cent_marker.pose.orientation.w = 1.0;
    centerline_markers.markers.push_back(cent_marker);

    f110_msgs::msg::Wpnt wpnt;
    wpnt.id = static_cast<int>(i);
    wpnt.x_m = x_m;
    wpnt.d_right = width_tr_right_m;
    wpnt.d_left = width_tr_left_m;
    wpnt.y_m = y_m;
    centerline_wpnts.wpnts.push_back(wpnt);
  }

  file.close();
  return {centerline_wpnts, centerline_markers};
}

visualization_msgs::msg::MarkerArray publish_track_bounds(
  const std::vector<Eigen::Vector2d> & bound_r,
  const std::vector<Eigen::Vector2d> & bound_l,
  bool reverse)
{
  visualization_msgs::msg::MarkerArray bounds_markers;
  int id_cnt = 0;

  const auto & bound_r_real = reverse ? bound_l : bound_r;
  const auto & bound_l_real = reverse ? bound_r : bound_l;

  for (const auto & pnt_r : bound_r_real) {
    visualization_msgs::msg::Marker bnd_r_mrk;
    bnd_r_mrk.header.frame_id = "map";
    bnd_r_mrk.type = visualization_msgs::msg::Marker::SPHERE;
    bnd_r_mrk.scale.x = 0.05;
    bnd_r_mrk.scale.y = 0.05;
    bnd_r_mrk.scale.z = 0.05;
    bnd_r_mrk.color.a = 1.0;
    bnd_r_mrk.color.b = 0.5;
    bnd_r_mrk.color.r = 0.5;
    bnd_r_mrk.id = id_cnt++;
    bnd_r_mrk.pose.position.x = pnt_r.x();
    bnd_r_mrk.pose.position.y = pnt_r.y();
    bnd_r_mrk.pose.orientation.w = 1.0;
    bounds_markers.markers.push_back(bnd_r_mrk);
  }

  for (const auto & pnt_l : bound_l_real) {
    visualization_msgs::msg::Marker bnd_l_mrk;
    bnd_l_mrk.header.frame_id = "map";
    bnd_l_mrk.type = visualization_msgs::msg::Marker::SPHERE;
    bnd_l_mrk.scale.x = 0.05;
    bnd_l_mrk.scale.y = 0.05;
    bnd_l_mrk.scale.z = 0.05;
    bnd_l_mrk.color.a = 1.0;
    bnd_l_mrk.color.r = 0.5;
    bnd_l_mrk.color.g = 1.0;
    bnd_l_mrk.id = id_cnt++;
    bnd_l_mrk.pose.position.x = pnt_l.x();
    bnd_l_mrk.pose.position.y = pnt_l.y();
    bnd_l_mrk.pose.orientation.w = 1.0;
    bounds_markers.markers.push_back(bnd_l_mrk);
  }

  return bounds_markers;
}

std::pair<f110_msgs::msg::WpntArray, visualization_msgs::msg::MarkerArray> create_wpnts_markers(
  const std::vector<Eigen::VectorXd> & trajectory,
  const std::vector<double> & d_right,
  const std::vector<double> & d_left,
  bool second_traj)
{
  f110_msgs::msg::WpntArray global_wpnts;
  visualization_msgs::msg::MarkerArray global_markers;

  double max_vx_mps = 0.0;
  for (const auto & traj : trajectory) {
    if (traj.size() > 5 && traj[5] > max_vx_mps) {
      max_vx_mps = traj[5];
    }
  }

  for (size_t i = 0; i < trajectory.size(); ++i) {
    const auto & pnt = trajectory[i];
    if (pnt.size() < 7) {
      continue;
    }

    f110_msgs::msg::Wpnt global_wpnt;
    global_wpnt.id = static_cast<int>(i);
    global_wpnt.s_m = pnt[0];
    global_wpnt.x_m = pnt[1];
    global_wpnt.d_right = d_right[i];
    global_wpnt.d_left = d_left[i];
    global_wpnt.y_m = pnt[2];
    global_wpnt.psi_rad = conv_psi(pnt[3]);
    global_wpnt.kappa_radpm = pnt[4];
    global_wpnt.vx_mps = pnt[5];
    global_wpnt.ax_mps2 = pnt[6];
    global_wpnts.wpnts.push_back(global_wpnt);

    visualization_msgs::msg::Marker global_marker;
    global_marker.header.frame_id = "map";
    global_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    global_marker.scale.x = 0.1;
    global_marker.scale.y = 0.1;
    global_marker.scale.z = max_vx_mps > 0 ? pnt[5] / max_vx_mps : 0.0;
    global_marker.color.a = 1.0;
    global_marker.color.r = 1.0;
    global_marker.color.g = second_traj ? 1.0 : 0.0;
    global_marker.id = static_cast<int>(i);
    global_marker.pose.position.x = pnt[1];
    global_marker.pose.position.y = pnt[2];
    global_marker.pose.position.z = max_vx_mps > 0 ? pnt[5] / max_vx_mps / 2 : 0.0;
    global_marker.pose.orientation.w = 1.0;
    global_markers.markers.push_back(global_marker);
  }

  return {global_wpnts, global_markers};
}

double conv_psi(double psi)
{
  double new_psi = psi + M_PI / 2.0;
  if (new_psi > M_PI) {
    new_psi -= 2 * M_PI;
  }
  return new_psi;
}

bool compare_direction(double alpha, double beta)
{
  double delta_theta = std::abs(alpha - beta);
  if (delta_theta > M_PI) {
    delta_theta = 2 * M_PI - delta_theta;
  }
  return delta_theta < M_PI / 2.0;
}

}  // namespace global_planner
