//글로벌 플래너 노드 헤더

#ifndef GLOBAL_PLANNER_NODE_HPP
#define GLOBAL_PLANNER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "centerline_extractor.hpp"
#include "curvature_planner.hpp"
#include "velocity_planner.hpp"
#include "time_optimal_planner.hpp"
#include "file_writer.hpp"

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
    
    // 구독자 및 발행자
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_state_sub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_pub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr shortest_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr shortest_path_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trackbounds_markers_pub_;

    // 플래너 객체들
    std::shared_ptr<CenterlineExtractor> centerline_extractor_;
    std::shared_ptr<CurvaturePlanner> curvature_planner_;
    std::shared_ptr<VelocityPlanner> velocity_planner_;
    std::shared_ptr<TimeOptimalPlanner> time_optimal_planner_;
    std::shared_ptr<FileWriter> file_writer_;
    
    // 파일 저장 경로
    std::string map_dir_;
    bool save_json_enabled_;
};

#endif  // GLOBAL_PLANNER_NODE_HPP
