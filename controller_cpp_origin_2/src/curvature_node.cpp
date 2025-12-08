/**
 * 곡률 노드 (Curvature Node)
 * 
 * 역할: 웨이포인트의 곡률을 분석하여 스탠리 가중치 계산
 * 
 * 의존성:
 *   - pp.hpp: 타입 정의만 사용 (PP_Controller::WpRow 등)
 *   - ROS2 표준 메시지: std_msgs, f110_msgs
 *   - 외부 프로젝트 파일과는 연결되지 않음 (독립적)
 * 
 * 구독 토픽:
 *   - /local_waypoints: 로컬 웨이포인트 배열
 *   - /control/nearest_waypoint_idx: 최근접 웨이포인트 인덱스
 * 
 * 발행 토픽:
 *   - /control/curvature_weight: 계산된 스탠리 가중치 (0.0 ~ 1.0)
 */

#include <memory>
#include <array>
#include <vector>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include "pp.hpp"  // PP_Controller의 타입 정의 사용

using namespace std::chrono_literals;

class CurvatureNode : public rclcpp::Node {
public:
  CurvatureNode() : Node("curvature_node") {
    // 발행자
    curvature_weight_pub_ = create_publisher<std_msgs::msg::Float64>("/control/curvature_weight", 10);

    // 구독자
    waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
        "/local_waypoints", 10,
        std::bind(&CurvatureNode::waypoints_callback, this, std::placeholders::_1));

    nearest_wp_idx_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/control/nearest_waypoint_idx", 10,
        std::bind(&CurvatureNode::nearest_wp_idx_callback, this, std::placeholders::_1));

    // 타이머로 주기적으로 곡률 가중치 계산 및 발행
    timer_ = create_wall_timer(25ms, std::bind(&CurvatureNode::compute_and_publish, this));

    RCLCPP_INFO(get_logger(), "Curvature node started");
  }

private:
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

  void nearest_wp_idx_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    nearest_wp_idx_ = msg->data;
  }

  // pp.cpp의 clip 함수와 동일한 로직
  static inline double clip(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  // pp.cpp의 estimate_future_curvature_change 로직 재구현
  double estimate_future_curvature_change() {
    if (waypoints_.size() < 3 || nearest_wp_idx_ < 0) {
      return 0.0;
    }

    constexpr double curvature_delta_horizon = 5.0; // [m] 미래 구간 길이
    constexpr int points_per_meter = 10; // 0.1m 해상도 가정
    const int horizon_idx = static_cast<int>(curvature_delta_horizon * points_per_meter);

    const int start_idx = nearest_wp_idx_;
    const int mid_idx = std::min(start_idx + horizon_idx / 2, static_cast<int>(waypoints_.size()) - 1);
    const int end_idx = std::min(start_idx + horizon_idx, static_cast<int>(waypoints_.size()) - 1);

    const double curv_now = waypoints_[start_idx][5]; // WpRow[5] = kappa
    const double curv_mid = waypoints_[mid_idx][5];
    const double curv_end = waypoints_[end_idx][5];

    // 현재-중간-미래 곡률의 변화 추세를 단순 미분으로 근사
    return (curv_end - curv_mid) + (curv_mid - curv_now);
  }

  void compute_and_publish() {
    // 필요한 데이터가 모두 수신되었는지 확인
    if (!waypoints_received_ || waypoints_.empty()) {
      return;
    }

    if (nearest_wp_idx_ < 0 || nearest_wp_idx_ >= static_cast<int>(waypoints_.size())) {
      return;
    }

    // pp.cpp의 곡률 계산 로직 재구현
    // 현재 곡률 평균 계산
    double curvature_now = 0.0;
    if (static_cast<int>(waypoints_.size()) - nearest_wp_idx_ > 2) {
      double sum = 0.0;
      int cnt = 0;
      for (int i = nearest_wp_idx_; i < static_cast<int>(waypoints_.size()); ++i) {
        sum += std::abs(waypoints_[i][5]); // WpRow[5] = kappa
        ++cnt;
      }
      curvature_now = (cnt > 0) ? (sum / cnt) : 0.0;
    }

    // 미래 곡률 변화량 추정
    const double curvature_future = estimate_future_curvature_change();

    // 곡률 기반 가중치 계산 (pp.cpp와 동일한 로직)
    constexpr double curvature_norm_scale = 0.65; // pp.cpp의 기존 값 유지
    const double w_curv = clip(curvature_now / curvature_norm_scale, 0.0, 1.0);
    const double w_delta = clip(std::abs(curvature_future) / curvature_norm_scale, 0.0, 1.0);
    double stanley_weight_raw = clip(0.5 * w_curv + 0.5 * w_delta, 0.0, 1.0);

    // 가중치 자체에 smoothing 추가 (pp.cpp와 동일한 로직)
    constexpr double weight_alpha = 0.9;  // pp.cpp와 동일한 값
    double stanley_weight = weight_alpha * stanley_weight_prev_ + (1.0 - weight_alpha) * stanley_weight_raw;
    stanley_weight_prev_ = stanley_weight;

    // 스탠리 가중치 최대값 제한 (pp.cpp와 동일)
    stanley_weight = clip(stanley_weight, 0.0, 0.35);

    // 발행
    std_msgs::msg::Float64 msg;
    msg.data = stanley_weight;
    curvature_weight_pub_->publish(msg);
  }

  // 상태 변수
  std::vector<PP_Controller::WpRow> waypoints_;
  int nearest_wp_idx_{-1};
  bool waypoints_received_{false};

  // 가중치 smoothing을 위한 이전 값
  double stanley_weight_prev_{0.0};

  // ROS2 인터페이스
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr curvature_weight_pub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr nearest_wp_idx_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurvatureNode>());
  rclcpp::shutdown();
  return 0;
}
