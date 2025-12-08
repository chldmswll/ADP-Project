/**
 * 스탠리 노드 (Stanley Node)
 * 
 * 역할: 스탠리 컨트롤러를 사용하여 조향 각도를 계산
 * 
 * 의존성:
 *   - pp.hpp: 타입 정의만 사용 (PP_Controller::Pose3, WpRow, Fren4 등)
 *   - ROS2 표준 메시지: geometry_msgs, nav_msgs, std_msgs, f110_msgs
 *   - 외부 프로젝트 파일과는 연결되지 않음 (독립적)
 * 
 * 구독 토픽:
 *   - /car_state/pose: 차량 위치 및 자세
 *   - /local_waypoints: 로컬 웨이포인트 배열
 *   - /car_state/frenet/odom: Frenet 좌표계 차량 상태
 *   - /car_state/odom: 차량 속도
 *   - /control/nearest_waypoint_idx: 최근접 웨이포인트 인덱스
 * 
 * 발행 토픽:
 *   - /control/stanley_angle: 계산된 스탠리 조향 각도
 */

#include <memory>
#include <array>
#include <vector>
#include <optional>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "pp.hpp"  // PP_Controller의 타입 정의 사용

using namespace std::chrono_literals;

class StanleyNode : public rclcpp::Node {
public:
  StanleyNode() : Node("stanley_node") {
    // 발행자
    stanley_angle_pub_ = create_publisher<std_msgs::msg::Float64>("/control/stanley_angle", 10);

    // 구독자
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/car_state/pose", 10,
        std::bind(&StanleyNode::pose_callback, this, std::placeholders::_1));

    waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
        "/local_waypoints", 10,
        std::bind(&StanleyNode::waypoints_callback, this, std::placeholders::_1));

    frenet_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/frenet/odom", 10,
        std::bind(&StanleyNode::frenet_callback, this, std::placeholders::_1));

    speed_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/car_state/odom", 10,
        std::bind(&StanleyNode::speed_callback, this, std::placeholders::_1));

    nearest_wp_idx_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/control/nearest_waypoint_idx", 10,
        std::bind(&StanleyNode::nearest_wp_idx_callback, this, std::placeholders::_1));

    // 타이머로 주기적으로 스탠리 각도 계산 및 발행
    timer_ = create_wall_timer(25ms, std::bind(&StanleyNode::compute_and_publish, this));

    RCLCPP_INFO(get_logger(), "Stanley node started");
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Quaternion을 yaw로 변환
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    pose_[0] = msg->pose.position.x;
    pose_[1] = msg->pose.position.y;
    pose_[2] = yaw;
    pose_received_ = true;
  }

  void waypoints_callback(const f110_msgs::msg::WpntArray::SharedPtr msg) {
    waypoints_.clear();
    waypoints_.reserve(msg->wpnts.size());
    for (const auto &w : msg->wpnts) {
      double share = 0.0;
      if (w.d_right + w.d_left != 0.0)
        share = std::min(w.d_left, w.d_right) / (w.d_right + w.d_left);
      
      PP_Controller::WpRow row;
      row[0] = w.x_m;
      row[1] = w.y_m;
      row[2] = w.vx_mps;
      row[3] = share;
      row[4] = w.s_m;
      row[5] = w.kappa_radpm;
      row[6] = w.psi_rad;
      row[7] = w.ax_mps2;
      waypoints_.push_back(row);
    }
    waypoints_received_ = true;
  }

  void frenet_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    frenet_[0] = msg->pose.pose.position.x;  // s
    frenet_[1] = msg->pose.pose.position.y;  // d
    frenet_[2] = msg->twist.twist.linear.x;   // vs
    frenet_[3] = msg->twist.twist.linear.y;  // vd
    frenet_received_ = true;
  }

  void speed_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    speed_now_ = msg->twist.twist.linear.x;
  }

  void nearest_wp_idx_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    nearest_wp_idx_ = msg->data;
  }

  void compute_and_publish() {
    // 필요한 데이터가 모두 수신되었는지 확인
    if (!pose_received_ || !waypoints_received_ || !frenet_received_ || waypoints_.empty()) {
      return;
    }

    if (nearest_wp_idx_ < 0 || nearest_wp_idx_ >= static_cast<int>(waypoints_.size())) {
      return;
    }

    // pp.cpp의 calc_stanley_angle 로직을 재구현
    const double yaw = pose_[2];
    const double map_heading = waypoints_[nearest_wp_idx_][6]; // WpRow[6] = psi (heading)

    // 헤딩 오차 계산
    const double dpsi = std::atan2(std::sin(yaw - map_heading), std::cos(yaw - map_heading));
    
    // 측방 오차 (Frenet d)
    const double lat_error = frenet_[1];

    // 스탠리 각도 계산 (pp.cpp의 calc_stanley_angle 로직)
    constexpr double stanley_softening = 1.2;   // pp.cpp와 동일한 값
    const double vel = std::max(speed_now_, 0.1);
    const double correction = std::atan2(lat_error, stanley_softening + vel);

    double stanley_angle = dpsi + correction;

    // 스탠리 각도에 rate limiting 추가 (pp.cpp와 동일한 로직)
    constexpr double stanley_rate_limit = 0.06;  // pp.cpp와 동일한 값
    const double stanley_diff = stanley_angle - stanley_prev_;
    if (std::abs(stanley_diff) > stanley_rate_limit) {
      stanley_angle = stanley_prev_ + (stanley_diff > 0 ? stanley_rate_limit : -stanley_rate_limit);
    }
    stanley_prev_ = stanley_angle;

    // 발행
    std_msgs::msg::Float64 msg;
    msg.data = stanley_angle;
    stanley_angle_pub_->publish(msg);
  }

  // 상태 변수
  PP_Controller::Pose3 pose_{0, 0, 0};
  std::vector<PP_Controller::WpRow> waypoints_;
  PP_Controller::Fren4 frenet_{0, 0, 0, 0};
  double speed_now_{0.0};
  int nearest_wp_idx_{-1};

  bool pose_received_{false};
  bool waypoints_received_{false};
  bool frenet_received_{false};

  // 스탠리 rate limiting을 위한 이전 값
  double stanley_prev_{0.0};

  // ROS2 인터페이스
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr stanley_angle_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr frenet_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr nearest_wp_idx_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StanleyNode>());
  rclcpp::shutdown();
  return 0;
}
