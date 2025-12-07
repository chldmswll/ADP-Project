//메인 노드 : 경로 생성, 속도 제한 계산 및 발행

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "centerline_extractor.hpp"
#include "curvature_planner.hpp"
#include "velocity_planner.hpp"
#include "time_optimal_planner.hpp"

using namespace std::chrono_literals;

class GlobalPlannerNode : public rclcpp::Node {
public:
    GlobalPlannerNode() : Node("global_planner_node") {
        // 구독자 정의
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&GlobalPlannerNode::map_callback, this, std::placeholders::_1));
        car_state_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/car_state/pose", 10, std::bind(&GlobalPlannerNode::car_state_callback, this, std::placeholders::_1));

        // 발행자 정의 (내부 토픽으로 발행, global_trajectory_publisher가 구독)
        global_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_planner/waypoints", 10);
        
        // 초기화
        centerline_extractor_ = std::make_shared<CenterlineExtractor>();
        curvature_planner_ = std::make_shared<CurvaturePlanner>();
        velocity_planner_ = std::make_shared<VelocityPlanner>();
        time_optimal_planner_ = std::make_shared<TimeOptimalPlanner>();
    }

private:
    // 구독자 콜백 함수
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // 트랙 지도 받음 (centerline 추출)
        centerline_extractor_->extract_centerline(msg);

        // 최소곡률 경로 생성
        auto curvature_path = curvature_planner_->generate_minimum_curvature_path(centerline_extractor_->get_centerline());

        // 속도 제한 계산 및 경로 최적화
        auto velocity_profile = velocity_planner_->generate_velocity_profile(curvature_path);

        // 최단시간 경로 계산
        auto time_optimal_path = time_optimal_planner_->optimize_time_path(velocity_profile);

        // 결과 발행
        global_waypoints_pub_->publish(time_optimal_path);
    }

    void car_state_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 차량 상태 처리 (예: 현재 위치)
    }

    // 구독자 및 발행자
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr car_state_sub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_pub_;

    // 플래너 객체들
    std::shared_ptr<CenterlineExtractor> centerline_extractor_;
    std::shared_ptr<CurvaturePlanner> curvature_planner_;
    std::shared_ptr<VelocityPlanner> velocity_planner_;
    std::shared_ptr<TimeOptimalPlanner> time_optimal_planner_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
