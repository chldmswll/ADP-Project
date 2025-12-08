#include "global_planner/readwrite_global_waypoints.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <nlohmann/json.hpp>
#include <tuple>

using json = nlohmann::json;

namespace global_planner
{

// Helper function to convert ROS2 message to JSON
template<typename T>
json message_to_json(const T & msg);

// Specializations for specific message types
template<>
json message_to_json<std_msgs::msg::String>(const std_msgs::msg::String & msg)
{
  json j;
  j["data"] = msg.data;
  return j;
}

template<>
json message_to_json<std_msgs::msg::Float32>(const std_msgs::msg::Float32 & msg)
{
  json j;
  j["data"] = msg.data;
  return j;
}

// Convert MarkerArray to JSON
json marker_array_to_json(const visualization_msgs::msg::MarkerArray & msg)
{
  json j;
  j["markers"] = json::array();
  for (const auto & marker : msg.markers) {
    json m;
    m["header"]["stamp"]["sec"] = marker.header.stamp.sec;
    m["header"]["stamp"]["nanosec"] = marker.header.stamp.nanosec;
    m["header"]["frame_id"] = marker.header.frame_id;
    m["ns"] = marker.ns;
    m["id"] = marker.id;
    m["type"] = marker.type;
    m["action"] = marker.action;
    m["pose"]["position"]["x"] = marker.pose.position.x;
    m["pose"]["position"]["y"] = marker.pose.position.y;
    m["pose"]["position"]["z"] = marker.pose.position.z;
    m["pose"]["orientation"]["x"] = marker.pose.orientation.x;
    m["pose"]["orientation"]["y"] = marker.pose.orientation.y;
    m["pose"]["orientation"]["z"] = marker.pose.orientation.z;
    m["pose"]["orientation"]["w"] = marker.pose.orientation.w;
    m["scale"]["x"] = marker.scale.x;
    m["scale"]["y"] = marker.scale.y;
    m["scale"]["z"] = marker.scale.z;
    m["color"]["r"] = marker.color.r;
    m["color"]["g"] = marker.color.g;
    m["color"]["b"] = marker.color.b;
    m["color"]["a"] = marker.color.a;
    j["markers"].push_back(m);
  }
  return j;
}

// Convert WpntArray to JSON
json wpnt_array_to_json(const f110_msgs::msg::WpntArray & msg)
{
  json j;
  j["wpnts"] = json::array();
  for (const auto & wpnt : msg.wpnts) {
    json w;
    w["id"] = wpnt.id;
    w["s_m"] = wpnt.s_m;
    w["x_m"] = wpnt.x_m;
    w["y_m"] = wpnt.y_m;
    w["psi_rad"] = wpnt.psi_rad;
    w["kappa_radpm"] = wpnt.kappa_radpm;
    w["vx_mps"] = wpnt.vx_mps;
    w["ax_mps2"] = wpnt.ax_mps2;
    w["d_right"] = wpnt.d_right;
    w["d_left"] = wpnt.d_left;
    j["wpnts"].push_back(w);
  }
  return j;
}

// Convert JSON to MarkerArray
visualization_msgs::msg::MarkerArray json_to_marker_array(const json & j)
{
  visualization_msgs::msg::MarkerArray msg;
  if (j.contains("markers") && j["markers"].is_array()) {
    for (const auto & m : j["markers"]) {
      visualization_msgs::msg::Marker marker;
      if (m.contains("header")) {
        if (m["header"].contains("stamp")) {
          marker.header.stamp.sec = m["header"]["stamp"].value("sec", 0);
          marker.header.stamp.nanosec = m["header"]["stamp"].value("nanosec", 0u);
        }
        marker.header.frame_id = m["header"].value("frame_id", "map");
      }
      marker.ns = m.value("ns", "");
      marker.id = m.value("id", 0);
      marker.type = m.value("type", 0);
      marker.action = m.value("action", 0);
      if (m.contains("pose")) {
        marker.pose.position.x = m["pose"]["position"].value("x", 0.0);
        marker.pose.position.y = m["pose"]["position"].value("y", 0.0);
        marker.pose.position.z = m["pose"]["position"].value("z", 0.0);
        marker.pose.orientation.x = m["pose"]["orientation"].value("x", 0.0);
        marker.pose.orientation.y = m["pose"]["orientation"].value("y", 0.0);
        marker.pose.orientation.z = m["pose"]["orientation"].value("z", 0.0);
        marker.pose.orientation.w = m["pose"]["orientation"].value("w", 1.0);
      }
      if (m.contains("scale")) {
        marker.scale.x = m["scale"].value("x", 0.0);
        marker.scale.y = m["scale"].value("y", 0.0);
        marker.scale.z = m["scale"].value("z", 0.0);
      }
      if (m.contains("color")) {
        marker.color.r = m["color"].value("r", 0.0);
        marker.color.g = m["color"].value("g", 0.0);
        marker.color.b = m["color"].value("b", 0.0);
        marker.color.a = m["color"].value("a", 1.0);
      }
      msg.markers.push_back(marker);
    }
  }
  return msg;
}

// Convert JSON to WpntArray
f110_msgs::msg::WpntArray json_to_wpnt_array(const json & j)
{
  f110_msgs::msg::WpntArray msg;
  if (j.contains("wpnts") && j["wpnts"].is_array()) {
    for (const auto & w : j["wpnts"]) {
      f110_msgs::msg::Wpnt wpnt;
      wpnt.id = w.value("id", 0);
      wpnt.s_m = w.value("s_m", 0.0);
      wpnt.x_m = w.value("x_m", 0.0);
      wpnt.y_m = w.value("y_m", 0.0);
      wpnt.psi_rad = w.value("psi_rad", 0.0);
      wpnt.kappa_radpm = w.value("kappa_radpm", 0.0);
      wpnt.vx_mps = w.value("vx_mps", 0.0);
      wpnt.ax_mps2 = w.value("ax_mps2", 0.0);
      wpnt.d_right = w.value("d_right", 0.0);
      wpnt.d_left = w.value("d_left", 0.0);
      msg.wpnts.push_back(wpnt);
    }
  }
  return msg;
}

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
  const visualization_msgs::msg::MarkerArray & trackbounds_markers)
{
  std::filesystem::path path = std::filesystem::path(map_dir) / "global_waypoints.json";
  std::cout << "[INFO] WRITE_GLOBAL_WAYPOINTS: Writing global waypoints to " << path << std::endl;

  json j;
  j["map_info_str"]["data"] = map_info_str;
  j["est_lap_time"]["data"] = est_lap_time.data;
  j["centerline_markers"] = marker_array_to_json(centerline_markers);
  j["centerline_waypoints"] = wpnt_array_to_json(centerline_waypoints);
  j["global_traj_markers_iqp"] = marker_array_to_json(global_traj_markers_iqp);
  j["global_traj_wpnts_iqp"] = wpnt_array_to_json(global_traj_wpnts_iqp);
  j["global_traj_markers_sp"] = marker_array_to_json(global_traj_markers_sp);
  j["global_traj_wpnts_sp"] = wpnt_array_to_json(global_traj_wpnts_sp);
  j["trackbounds_markers"] = marker_array_to_json(trackbounds_markers);

  std::ofstream file(path);
  if (file.is_open()) {
    file << j.dump(2);
    file.close();
  } else {
    std::cerr << "[ERROR] Failed to open file for writing: " << path << std::endl;
  }
}

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
> read_global_waypoints(const std::string & map_dir)
{
  std::filesystem::path path = std::filesystem::path(map_dir) / "global_waypoints.json";
  std::cout << "[INFO] READ_GLOBAL_WAYPOINTS: Reading global waypoints from " << path << std::endl;

  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + path.string());
  }

  json j;
  file >> j;
  file.close();

  std_msgs::msg::String map_info_str;
  map_info_str.data = j["map_info_str"].value("data", std::string(""));

  std_msgs::msg::Float32 est_lap_time;
  est_lap_time.data = j["est_lap_time"].value("data", 0.0f);

  visualization_msgs::msg::MarkerArray centerline_markers = json_to_marker_array(j["centerline_markers"]);
  f110_msgs::msg::WpntArray centerline_waypoints = json_to_wpnt_array(j["centerline_waypoints"]);
  visualization_msgs::msg::MarkerArray global_traj_markers_iqp = json_to_marker_array(j["global_traj_markers_iqp"]);
  f110_msgs::msg::WpntArray global_traj_wpnts_iqp = json_to_wpnt_array(j["global_traj_wpnts_iqp"]);
  visualization_msgs::msg::MarkerArray global_traj_markers_sp = json_to_marker_array(j["global_traj_markers_sp"]);
  f110_msgs::msg::WpntArray global_traj_wpnts_sp = json_to_wpnt_array(j["global_traj_wpnts_sp"]);
  visualization_msgs::msg::MarkerArray trackbounds_markers = json_to_marker_array(j["trackbounds_markers"]);

  return std::make_tuple(
    map_info_str, est_lap_time,
    centerline_markers, centerline_waypoints,
    global_traj_markers_iqp, global_traj_wpnts_iqp,
    global_traj_markers_sp, global_traj_wpnts_sp,
    trackbounds_markers
  );
}

}  // namespace global_planner
