//글로벌 플래너 노드 헤더

#ifndef GLOBAL_PLANNER_NODE_HPP
#define GLOBAL_PLANNER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "centerline_extractor.hpp"
#include "curvature_planner.hpp"
#include "velocity_planner.hpp"
#include "time_optimal_planner.hpp"
#include <string>
#include <fstream>

class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode();
private:
    // 구독자 콜백 함수
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void car_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    // 헬퍼 함수
    visualization_msgs::msg::MarkerArray create_trackbounds_markers();
    visualization_msgs::msg::MarkerArray create_waypoints_markers(const f110_msgs::msg::WpntArray& waypoints);
    f110_msgs::msg::WpntArray generate_shortest_path(const std::vector<f110_msgs::msg::Wpnt>& centerline);
    f110_msgs::msg::WpntArray create_centerline_waypoints(const std::vector<f110_msgs::msg::Wpnt>& centerline);
    double calculate_estimated_lap_time(const f110_msgs::msg::WpntArray& waypoints);
    double calculate_max_speed(const f110_msgs::msg::WpntArray& waypoints);
    std::string create_map_info_string(const f110_msgs::msg::WpntArray& time_optimal_path, 
                                       const f110_msgs::msg::WpntArray& shortest_path);
    void write_global_waypoints_json(const std::string& map_dir,
                                     const std::string& map_info_str,
                                     float est_lap_time,
                                     const visualization_msgs::msg::MarkerArray& centerline_markers,
                                     const f110_msgs::msg::WpntArray& centerline_waypoints,
                                     const visualization_msgs::msg::MarkerArray& global_traj_markers_iqp,
                                     const f110_msgs::msg::WpntArray& global_traj_wpnts_iqp,
                                     const visualization_msgs::msg::MarkerArray& global_traj_markers_sp,
                                     const f110_msgs::msg::WpntArray& global_traj_wpnts_sp,
                                     const visualization_msgs::msg::MarkerArray& trackbounds_markers);
    
    // 구독자 및 발행자
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_state_sub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_pub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr shortest_path_pub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr centerline_waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr shortest_path_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trackbounds_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_infos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr est_lap_time_pub_;

    // 플래너 객체들
    std::shared_ptr<CenterlineExtractor> centerline_extractor_;
    std::shared_ptr<CurvaturePlanner> curvature_planner_;
    std::shared_ptr<VelocityPlanner> velocity_planner_;
    std::shared_ptr<TimeOptimalPlanner> time_optimal_planner_;
    
    // 파라미터
    std::string map_name_;
    std::string map_dir_;
    bool map_processed_;
};

#endif  // GLOBAL_PLANNER_NODE_HPP
