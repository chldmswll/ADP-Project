//메인 노드 : 경로 생성, 속도 제한 계산 및 발행

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "global_planner_node.hpp"
#include "centerline_extractor.hpp"
#include "curvature_planner.hpp"
#include "velocity_planner.hpp"
#include "time_optimal_planner.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;

GlobalPlannerNode::GlobalPlannerNode() : Node("global_planner_node"), map_processed_(false) {
        RCLCPP_INFO(this->get_logger(), "Global planner node starting...");
        
        // 파라미터 읽기
        this->declare_parameter<std::string>("map_name", "");
        this->declare_parameter<std::string>("map_dir", "");
        this->get_parameter("map_name", map_name_);
        this->get_parameter("map_dir", map_dir_);
        
        if (map_dir_.empty() && !map_name_.empty()) {
            // 기본 경로 설정
            map_dir_ = "/home/misys/forza_ws/race_stack/stack_master/maps/" + map_name_;
        }
        
        RCLCPP_INFO(this->get_logger(), "Map name: %s, Map dir: %s", map_name_.c_str(), map_dir_.c_str());
        
        // 구독자 정의
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&GlobalPlannerNode::map_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /map");
        
        car_state_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/car_state/pose", 10, std::bind(&GlobalPlannerNode::car_state_callback, this, std::placeholders::_1));

        // 발행자 정의 (내부 토픽으로 발행, global_trajectory_publisher가 구독)
        global_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_planner/waypoints", 10);
        shortest_path_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_planner/shortest_path", 10);
        centerline_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_planner/centerline_waypoints", 10);
        waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/waypoints/markers", 10);
        shortest_path_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/shortest_path/markers", 10);
        centerline_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/centerline_waypoints/markers", 10);
        trackbounds_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/trackbounds/markers", 10);
        map_infos_pub_ = this->create_publisher<std_msgs::msg::String>("/global_planner/map_infos", 10);
        est_lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/global_planner/estimated_lap_time", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/waypoints");
        
        // 초기화
        centerline_extractor_ = std::make_shared<CenterlineExtractor>();
        curvature_planner_ = std::make_shared<CurvaturePlanner>();
        velocity_planner_ = std::make_shared<VelocityPlanner>();
        time_optimal_planner_ = std::make_shared<TimeOptimalPlanner>();
        
        RCLCPP_INFO(this->get_logger(), "Global planner node initialized, waiting for map...");
}

void GlobalPlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        if (map_processed_) {
            return; // 이미 처리했으면 다시 처리하지 않음
        }
        
        RCLCPP_INFO(this->get_logger(), "Received map: %dx%d, resolution: %.3f", 
                    msg->info.width, msg->info.height, msg->info.resolution);
        
        // 트랙 지도 받음 (centerline 추출)
        centerline_extractor_->extract_centerline(msg);
        const auto& centerline = centerline_extractor_->get_centerline();
        RCLCPP_INFO(this->get_logger(), "Extracted centerline with %zu points", centerline.size());

        if (centerline.empty()) {
            RCLCPP_WARN(this->get_logger(), "Centerline is empty, skipping path generation");
            return;
        }

        // Centerline waypoints 생성 및 발행
        auto centerline_waypoints = create_centerline_waypoints(centerline);
        centerline_waypoints.header.stamp = this->now();
        centerline_waypoints.header.frame_id = "map";
        centerline_waypoints_pub_->publish(centerline_waypoints);
        
        auto centerline_markers = create_waypoints_markers(centerline_waypoints);
        centerline_markers_pub_->publish(centerline_markers);

        // 최소곡률 경로 생성
        auto curvature_path = curvature_planner_->generate_minimum_curvature_path(centerline);
        RCLCPP_INFO(this->get_logger(), "Generated curvature path with %zu waypoints", curvature_path.wpnts.size());

        // 속도 제한 계산 및 경로 최적화
        auto velocity_profile = velocity_planner_->generate_velocity_profile(curvature_path);
        RCLCPP_INFO(this->get_logger(), "Generated velocity profile with %zu waypoints", velocity_profile.wpnts.size());

        // 최단시간 경로 계산
        auto time_optimal_path = time_optimal_planner_->optimize_time_path(velocity_profile);
        RCLCPP_INFO(this->get_logger(), "Generated time optimal path with %zu waypoints", time_optimal_path.wpnts.size());

        // 최단 경로 생성 (centerline 기반)
        auto shortest_path = generate_shortest_path(centerline);
        RCLCPP_INFO(this->get_logger(), "Generated shortest path with %zu waypoints", shortest_path.wpnts.size());

        // 결과 발행
        time_optimal_path.header.stamp = this->now();
        time_optimal_path.header.frame_id = "map";
        global_waypoints_pub_->publish(time_optimal_path);
        
        shortest_path.header.stamp = this->now();
        shortest_path.header.frame_id = "map";
        shortest_path_pub_->publish(shortest_path);
        
        // 마커 발행
        auto waypoints_markers = create_waypoints_markers(time_optimal_path);
        waypoints_markers_pub_->publish(waypoints_markers);
        
        auto shortest_path_markers = create_waypoints_markers(shortest_path);
        shortest_path_markers_pub_->publish(shortest_path_markers);
        
        auto trackbounds_markers = create_trackbounds_markers();
        trackbounds_markers_pub_->publish(trackbounds_markers);
        
        // Estimated lap time 계산 및 발행
        double est_lap_time = calculate_estimated_lap_time(time_optimal_path);
        std_msgs::msg::Float32 est_lap_time_msg;
        est_lap_time_msg.data = static_cast<float>(est_lap_time);
        est_lap_time_pub_->publish(est_lap_time_msg);
        
        // Map info 생성 및 발행
        std::string map_info_str = create_map_info_string(time_optimal_path, shortest_path);
        std_msgs::msg::String map_info_msg;
        map_info_msg.data = map_info_str;
        map_infos_pub_->publish(map_info_msg);
        
        // JSON 파일 저장
        if (!map_dir_.empty()) {
            write_global_waypoints_json(map_dir_, map_info_str, est_lap_time,
                                       centerline_markers, centerline_waypoints,
                                       waypoints_markers, time_optimal_path,
                                       shortest_path_markers, shortest_path,
                                       trackbounds_markers);
        }
        
        map_processed_ = true;
        RCLCPP_INFO(this->get_logger(), "Published waypoints, markers, trackbounds, and saved JSON file");
}

void GlobalPlannerNode::car_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 차량 상태 처리 (예: 현재 위치)
    }

visualization_msgs::msg::MarkerArray GlobalPlannerNode::create_trackbounds_markers() {
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.clear();
        
        const auto& left_boundary = centerline_extractor_->get_left_boundary();
        const auto& right_boundary = centerline_extractor_->get_right_boundary();
        
        int id = 0;
        
        // 우측 경계선 (파란색)
        for (const auto& point : right_boundary) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.first;
            marker.pose.position.y = point.second;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;
            marker.color.r = 0.5;
            marker.color.b = 0.5;
            markers.markers.push_back(marker);
        }
        
        // 좌측 경계선 (녹색)
        for (const auto& point : left_boundary) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.first;
            marker.pose.position.y = point.second;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.a = 1.0;
            marker.color.r = 0.5;
            marker.color.g = 1.0;
            markers.markers.push_back(marker);
        }
        
        return markers;
    }

visualization_msgs::msg::MarkerArray GlobalPlannerNode::create_waypoints_markers(const f110_msgs::msg::WpntArray& waypoints) {
        visualization_msgs::msg::MarkerArray markers;
        markers.markers.clear();
        
        if (waypoints.wpnts.empty()) {
            return markers;
        }
        
        // 속도에 따른 최대값 찾기
        double max_vx = 0.0;
        for (const auto& wpnt : waypoints.wpnts) {
            if (wpnt.vx_mps > max_vx) {
                max_vx = wpnt.vx_mps;
            }
        }
        
        if (max_vx < 1e-6) {
            max_vx = 1.0; // 0으로 나누기 방지
        }
        
        int id = 0;
        for (const auto& wpnt : waypoints.wpnts) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->now();
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wpnt.x_m;
            marker.pose.position.y = wpnt.y_m;
            marker.pose.position.z = wpnt.vx_mps / max_vx / 2.0; // 속도에 비례한 높이
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = wpnt.vx_mps / max_vx; // 속도에 비례한 높이
            marker.color.a = 1.0;
            marker.color.r = 1.0; // 빨간색
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            markers.markers.push_back(marker);
        }
        
        return markers;
    }

f110_msgs::msg::WpntArray GlobalPlannerNode::generate_shortest_path(const std::vector<f110_msgs::msg::Wpnt>& centerline) {
        f110_msgs::msg::WpntArray shortest_path;
        shortest_path.header.frame_id = "map";
        shortest_path.header.stamp = this->now();
        
        if (centerline.empty()) {
            return shortest_path;
        }
        
        // 최단 경로는 centerline을 따라 최대 속도로 주행하는 경로
        for (size_t i = 0; i < centerline.size(); i++) {
            f110_msgs::msg::Wpnt wpnt = centerline[i];
            wpnt.id = static_cast<int32_t>(i);
            wpnt.vx_mps = 10.0; // 최대 속도 (기본값)
            wpnt.kappa_radpm = 0.0; // 직선으로 가정
            shortest_path.wpnts.push_back(wpnt);
        }
        
        return shortest_path;
    }

f110_msgs::msg::WpntArray GlobalPlannerNode::create_centerline_waypoints(const std::vector<f110_msgs::msg::Wpnt>& centerline) {
        f110_msgs::msg::WpntArray centerline_waypoints;
        centerline_waypoints.header.frame_id = "map";
        centerline_waypoints.header.stamp = this->now();
        
        for (size_t i = 0; i < centerline.size(); i++) {
            centerline_waypoints.wpnts.push_back(centerline[i]);
        }
        
        return centerline_waypoints;
    }

double GlobalPlannerNode::calculate_estimated_lap_time(const f110_msgs::msg::WpntArray& waypoints) {
        if (waypoints.wpnts.empty()) {
            return 0.0;
        }
        
        double total_time = 0.0;
        for (size_t i = 0; i < waypoints.wpnts.size() - 1; i++) {
            const auto& p1 = waypoints.wpnts[i];
            const auto& p2 = waypoints.wpnts[i + 1];
            
            double dx = p2.x_m - p1.x_m;
            double dy = p2.y_m - p1.y_m;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            double avg_speed = (p1.vx_mps + p2.vx_mps) / 2.0;
            if (avg_speed > 1e-6) {
                total_time += dist / avg_speed;
            }
        }
        
        return total_time;
    }

double GlobalPlannerNode::calculate_max_speed(const f110_msgs::msg::WpntArray& waypoints) {
        double max_speed = 0.0;
        for (const auto& wpnt : waypoints.wpnts) {
            if (wpnt.vx_mps > max_speed) {
                max_speed = wpnt.vx_mps;
            }
        }
        return max_speed;
    }

std::string GlobalPlannerNode::create_map_info_string(const f110_msgs::msg::WpntArray& time_optimal_path, 
                                                       const f110_msgs::msg::WpntArray& shortest_path) {
        std::ostringstream oss;
        
        double est_lap_time_iqp = calculate_estimated_lap_time(time_optimal_path);
        double max_speed_iqp = calculate_max_speed(time_optimal_path);
        double est_lap_time_sp = calculate_estimated_lap_time(shortest_path);
        double max_speed_sp = calculate_max_speed(shortest_path);
        
        oss << std::fixed << std::setprecision(4);
        oss << "IQP estimated lap time: " << est_lap_time_iqp << "s; ";
        oss << "IQP maximum speed: " << max_speed_iqp << "m/s; ";
        oss << "SP estimated lap time: " << est_lap_time_sp << "s; ";
        oss << "SP maximum speed: " << max_speed_sp << "m/s; ";
        
        return oss.str();
    }

void GlobalPlannerNode::write_global_waypoints_json(const std::string& map_dir,
                                                     const std::string& map_info_str,
                                                     float est_lap_time,
                                                     const visualization_msgs::msg::MarkerArray& centerline_markers,
                                                     const f110_msgs::msg::WpntArray& centerline_waypoints,
                                                     const visualization_msgs::msg::MarkerArray& global_traj_markers_iqp,
                                                     const f110_msgs::msg::WpntArray& global_traj_wpnts_iqp,
                                                     const visualization_msgs::msg::MarkerArray& global_traj_markers_sp,
                                                     const f110_msgs::msg::WpntArray& global_traj_wpnts_sp,
                                                     const visualization_msgs::msg::MarkerArray& trackbounds_markers) {
        std::string json_path = map_dir + "/global_waypoints.json";
        RCLCPP_INFO(this->get_logger(), "Writing global waypoints to %s", json_path.c_str());
        
        // 디렉토리 생성 (필요시)
        // 디렉토리는 launch 파일에서 이미 생성되어 있다고 가정
        
        std::ofstream file(json_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing: %s", json_path.c_str());
            return;
        }
        
        // 간단한 JSON 생성 (Python의 message_to_ordereddict와 유사한 구조)
        file << "{\n";
        file << "  \"map_info_str\": {\"data\": \"" << map_info_str << "\"},\n";
        file << "  \"est_lap_time\": {\"data\": " << est_lap_time << "},\n";
        
        // Centerline markers (간단한 표현)
        file << "  \"centerline_markers\": {\"markers\": []},\n";
        
        // Centerline waypoints
        file << "  \"centerline_waypoints\": {\n";
        file << "    \"header\": {\"frame_id\": \"" << centerline_waypoints.header.frame_id << "\"},\n";
        file << "    \"wpnts\": [\n";
        for (size_t i = 0; i < centerline_waypoints.wpnts.size(); i++) {
            const auto& wpnt = centerline_waypoints.wpnts[i];
            file << "      {\"id\": " << wpnt.id 
                 << ", \"x_m\": " << wpnt.x_m 
                 << ", \"y_m\": " << wpnt.y_m
                 << ", \"psi_rad\": " << wpnt.psi_rad
                 << ", \"kappa_radpm\": " << wpnt.kappa_radpm
                 << ", \"vx_mps\": " << wpnt.vx_mps << "}";
            if (i < centerline_waypoints.wpnts.size() - 1) {
                file << ",\n";
            } else {
                file << "\n";
            }
        }
        file << "    ]\n";
        file << "  },\n";
        
        // Global trajectory markers (간단한 표현)
        file << "  \"global_traj_markers_iqp\": {\"markers\": []},\n";
        
        // Global trajectory waypoints
        file << "  \"global_traj_wpnts_iqp\": {\n";
        file << "    \"header\": {\"frame_id\": \"" << global_traj_wpnts_iqp.header.frame_id << "\"},\n";
        file << "    \"wpnts\": [\n";
        for (size_t i = 0; i < global_traj_wpnts_iqp.wpnts.size(); i++) {
            const auto& wpnt = global_traj_wpnts_iqp.wpnts[i];
            file << "      {\"id\": " << wpnt.id 
                 << ", \"x_m\": " << wpnt.x_m 
                 << ", \"y_m\": " << wpnt.y_m
                 << ", \"psi_rad\": " << wpnt.psi_rad
                 << ", \"kappa_radpm\": " << wpnt.kappa_radpm
                 << ", \"vx_mps\": " << wpnt.vx_mps << "}";
            if (i < global_traj_wpnts_iqp.wpnts.size() - 1) {
                file << ",\n";
            } else {
                file << "\n";
            }
        }
        file << "    ]\n";
        file << "  },\n";
        
        // Shortest path markers (간단한 표현)
        file << "  \"global_traj_markers_sp\": {\"markers\": []},\n";
        
        // Shortest path waypoints
        file << "  \"global_traj_wpnts_sp\": {\n";
        file << "    \"header\": {\"frame_id\": \"" << global_traj_wpnts_sp.header.frame_id << "\"},\n";
        file << "    \"wpnts\": [\n";
        for (size_t i = 0; i < global_traj_wpnts_sp.wpnts.size(); i++) {
            const auto& wpnt = global_traj_wpnts_sp.wpnts[i];
            file << "      {\"id\": " << wpnt.id 
                 << ", \"x_m\": " << wpnt.x_m 
                 << ", \"y_m\": " << wpnt.y_m
                 << ", \"psi_rad\": " << wpnt.psi_rad
                 << ", \"kappa_radpm\": " << wpnt.kappa_radpm
                 << ", \"vx_mps\": " << wpnt.vx_mps << "}";
            if (i < global_traj_wpnts_sp.wpnts.size() - 1) {
                file << ",\n";
            } else {
                file << "\n";
            }
        }
        file << "    ]\n";
        file << "  },\n";
        
        // Trackbounds markers (간단한 표현)
        file << "  \"trackbounds_markers\": {\"markers\": []}\n";
        
        file << "}\n";
        file.close();
        
        RCLCPP_INFO(this->get_logger(), "Successfully wrote global_waypoints.json");
    }


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
