//메인 노드 : 경로 생성, 속도 제한 계산 및 발행

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
#include "file_writer.hpp"
#include <cmath>
#include <filesystem>
#include <iostream>
#include <exception>

using namespace std::chrono_literals;

GlobalPlannerNode::GlobalPlannerNode() : Node("global_planner_node") {
        RCLCPP_INFO(this->get_logger(), "Global planner node starting...");
        
        // 구독자 정의
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&GlobalPlannerNode::map_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /map");
        
        car_state_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/car_state/pose", 10, std::bind(&GlobalPlannerNode::car_state_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscribed to /car_state/pose");

        // 발행자 정의 (내부 토픽으로 발행, global_trajectory_publisher가 구독)
        global_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_planner/waypoints", 10);
        shortest_path_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_planner/shortest_path", 10);
        centerline_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_planner/centerline_waypoints", 10);
        waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/waypoints/markers", 10);
        shortest_path_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/shortest_path/markers", 10);
        centerline_waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/centerline_waypoints/markers", 10);
        trackbounds_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_planner/trackbounds/markers", 10);
        map_infos_pub_ = this->create_publisher<std_msgs::msg::String>("/global_planner/map_infos", 10);
        est_lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/global_planner/estimated_lap_time", 10);
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/waypoints");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/shortest_path");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/centerline_waypoints");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/waypoints/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/shortest_path/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/centerline_waypoints/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/trackbounds/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/map_infos");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_planner/estimated_lap_time");
        
        // 초기화
        centerline_extractor_ = std::make_shared<CenterlineExtractor>();
        curvature_planner_ = std::make_shared<CurvaturePlanner>();
        velocity_planner_ = std::make_shared<VelocityPlanner>();
        time_optimal_planner_ = std::make_shared<TimeOptimalPlanner>();
        file_writer_ = std::make_shared<FileWriter>();
        
        // 파일 저장 설정 (파라미터 또는 환경변수에서 가져올 수 있음)
        this->declare_parameter<std::string>("map_dir", "");
        this->declare_parameter<bool>("save_json", true);
        this->get_parameter("map_dir", map_dir_);
        this->get_parameter("save_json", save_json_enabled_);
        
        if (map_dir_.empty()) {
            // 기본값: 현재 작업 디렉토리
            map_dir_ = std::filesystem::current_path().string();
        }
        
        RCLCPP_INFO(this->get_logger(), "Global planner node initialized, waiting for map...");
        RCLCPP_INFO(this->get_logger(), "JSON save enabled: %s, map_dir: %s", 
                    save_json_enabled_ ? "true" : "false", map_dir_.c_str());
}

void GlobalPlannerNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
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

        // centerline을 WpntArray로 변환
        f110_msgs::msg::WpntArray centerline_array;
        centerline_array.header.frame_id = "map";
        centerline_array.header.stamp = this->now();
        for (size_t i = 0; i < centerline.size(); ++i) {
            f110_msgs::msg::Wpnt wpnt = centerline[i];
            wpnt.id = static_cast<int32_t>(i);
            centerline_array.wpnts.push_back(wpnt);
        }

        // 결과 발행
        time_optimal_path.header.stamp = this->now();
        time_optimal_path.header.frame_id = "map";
        global_waypoints_pub_->publish(time_optimal_path);
        
        shortest_path.header.stamp = this->now();
        shortest_path.header.frame_id = "map";
        shortest_path_pub_->publish(shortest_path);
        
        centerline_waypoints_pub_->publish(centerline_array);
        
        // 마커 발행
        auto waypoints_markers = create_waypoints_markers(time_optimal_path);
        waypoints_markers_pub_->publish(waypoints_markers);
        
        auto shortest_path_markers = create_waypoints_markers(shortest_path);
        shortest_path_markers_pub_->publish(shortest_path_markers);
        
        auto centerline_markers = create_waypoints_markers(centerline_array);
        centerline_waypoints_markers_pub_->publish(centerline_markers);
        
        auto trackbounds_markers = create_trackbounds_markers();
        trackbounds_markers_pub_->publish(trackbounds_markers);
        
        // map_infos 및 estimated_lap_time 발행
        std_msgs::msg::String map_info_str;
        map_info_str.data = "Generated by global_planner (C++)";
        map_infos_pub_->publish(map_info_str);
        
        std_msgs::msg::Float32 est_lap_time;
        est_lap_time.data = 0.0f; // TODO: 실제 랩 타임 계산 필요
        est_lap_time_pub_->publish(est_lap_time);
        
        RCLCPP_INFO(this->get_logger(), "Published waypoints, markers, trackbounds, centerline, map_infos, and estimated_lap_time");
        
        // global_waypoints.json 파일 저장
        if (save_json_enabled_) {
            // map_info_str과 est_lap_time 준비 (기본값)
            std_msgs::msg::String map_info_str;
            map_info_str.data = "Generated by global_planner (C++)";
            
            std_msgs::msg::Float32 est_lap_time;
            est_lap_time.data = 0.0f; // 나중에 계산 가능
            
            // JSON 파일 저장
            bool success = file_writer_->write_global_waypoints(
                map_dir_,
                map_info_str,
                est_lap_time,
                centerline_array,
                time_optimal_path,
                shortest_path
            );
            
            if (success) {
                RCLCPP_INFO(this->get_logger(), "Successfully saved global_waypoints.json to %s", map_dir_.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to save global_waypoints.json");
            }
            
            // speed_scaling.yaml 파일 저장
            std::string speed_scaling_path = map_dir_;
            if (!speed_scaling_path.empty() && speed_scaling_path.back() != '/') {
                speed_scaling_path += "/";
            }
            speed_scaling_path += "speed_scaling.yaml";
            
            bool speed_scaling_success = file_writer_->save_speed_scaling(
                time_optimal_path,
                speed_scaling_path,
                9,  // n_sectors: 기본값 9개 섹터
                0.8f  // global_limit: 기본값 0.8
            );
            
            if (speed_scaling_success) {
                RCLCPP_INFO(this->get_logger(), "Successfully saved speed_scaling.yaml to %s", speed_scaling_path.c_str());
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to save speed_scaling.yaml");
            }
        }
    }

void GlobalPlannerNode::car_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 차량 상태 처리 (예: 현재 위치)
        RCLCPP_DEBUG(this->get_logger(), "Received car state: x=%.2f, y=%.2f", 
                     msg->pose.position.x, msg->pose.position.y);
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


int main(int argc, char * argv[]) {
    try {
        rclcpp::init(argc, argv);
        
        auto node = std::make_shared<GlobalPlannerNode>();
        RCLCPP_INFO(node->get_logger(), "Starting global_planner node...");
        
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception in global_planner: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Unknown exception in global_planner" << std::endl;
        return 1;
    }
}
