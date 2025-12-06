#include "teb_local_planner/local_planner_node.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include <Eigen/Dense>
#include <cmath>

namespace teb_local_planner
{

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("teb_local_planner", options)
{
  load_parameters();
  initialize_publishers();
  initialize_subscriptions();

  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / config_.refresh_hz)),
    std::bind(&LocalPlannerNode::on_timer, this));

  RCLCPP_INFO(get_logger(), "Local planner node initialized with %.2f Hz timer.", config_.refresh_hz);
  RCLCPP_INFO(get_logger(), "Waiting for required messages...");
}

void LocalPlannerNode::load_parameters()
{
  config_.refresh_hz = declare_parameter<double>("refresh_hz", config_.refresh_hz);
  config_.horizon_length = declare_parameter<double>("horizon_length", config_.horizon_length);
  config_.safety_margin = declare_parameter<double>("safety_margin", config_.safety_margin);

  // 새로 추가할 파라미터들
  config_.from_bag = declare_parameter<bool>("from_bag", config_.from_bag);
  config_.measure = declare_parameter<bool>("measure", config_.measure);
  
  config_.pre_apex_0 = declare_parameter<double>("pre_apex_0", std::abs(config_.pre_apex_0));
  config_.pre_apex_1 = declare_parameter<double>("pre_apex_1", std::abs(config_.pre_apex_1));
  config_.pre_apex_2 = declare_parameter<double>("pre_apex_2", std::abs(config_.pre_apex_2));
  config_.post_apex_0 = declare_parameter<double>("post_apex_0", config_.post_apex_0);
  config_.post_apex_1 = declare_parameter<double>("post_apex_1", config_.post_apex_1);
  config_.post_apex_2 = declare_parameter<double>("post_apex_2", config_.post_apex_2);
  
  config_.evasion_dist = declare_parameter<double>("evasion_dist", config_.evasion_dist);
  config_.obs_traj_tresh = declare_parameter<double>("obs_traj_tresh", config_.obs_traj_tresh);
  config_.spline_bound_mindist = declare_parameter<double>("spline_bound_mindist", config_.spline_bound_mindist);
  config_.fixed_pred_time = declare_parameter<double>("fixed_pred_time", config_.fixed_pred_time);
  config_.kd_obs_pred = declare_parameter<double>("kd_obs_pred", config_.kd_obs_pred);
  config_.lookahead = declare_parameter<double>("lookahead", config_.lookahead);
  
  // pre_apex는 음수로 저장 (Python 코드 참고)
  config_.pre_apex_0 = -std::abs(config_.pre_apex_0);
  config_.pre_apex_1 = -std::abs(config_.pre_apex_1);
  config_.pre_apex_2 = -std::abs(config_.pre_apex_2);
}

void LocalPlannerNode::initialize_publishers()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planner/avoidance/markers", qos);

  otwpnts_pub_ = create_publisher<f110_msgs::msg::OTWpntArray>(
    "/planner/avoidance/otwpnts", qos);

  considered_obs_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "/planner/avoidance/considered_OBS", qos);

  propagated_obs_pub_ = create_publisher<visualization_msgs::msg::Marker>(
    "/planner/avoidance/propagated_obs", qos);

  latency_pub_ = create_publisher<std_msgs::msg::Float32>(
    "/planner/avoidance/latency", qos);
  
  // Race line visualization publisher for rviz
  // 기존 global_republisher 대신 ADP-Project 플래너가 발행
  race_line_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/global_waypoints/markers", qos);
}

void LocalPlannerNode::initialize_subscriptions()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  obstacle_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
    "/perception/obstacles", qos,
    std::bind(&LocalPlannerNode::obstaclesCallback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", qos,
    std::bind(&LocalPlannerNode::odomCallback, this, std::placeholders::_1));

  global_wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints", qos,
    std::bind(&LocalPlannerNode::globalWaypointsCallback, this, std::placeholders::_1));

  global_wpnts_scaled_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints_scaled", qos,
    std::bind(&LocalPlannerNode::scaledWaypointsCallback, this, std::placeholders::_1));
}

void LocalPlannerNode::on_timer()
{
  // 메시지가 준비될 때까지 대기
  if (!messages_ready_) {
    messages_ready_ = check_messages_ready();
    if (!messages_ready_) {
      return;  // 아직 메시지가 없으면 리턴
    }
    RCLCPP_INFO(get_logger(), "All required messages received. Starting planner...");
  }

  auto start_time = std::chrono::high_resolution_clock::now();

  // 입력 데이터 유효성 검사
  if (!obstacles_ || !global_wpnts_scaled_ || !current_state_){
    return;
  }
  
  std::vector<f110_msgs::msg::Obstacle> close_obstacles = filter_obstacles();

  f110_msgs::msg::OTWpntArray wpnts;
  visualization_msgs::msg::MarkerArray markers;

  if (!close_obstacles.empty()){
    auto closest_obs = find_closest_obstacle(close_obstacles);

    std::tie(wpnts, markers) = generate_evasion_trajectory(closest_obs);
  }else{
    markers = create_delete_all_marker();
  }

  otwpnts_pub_->publish(wpnts);
  markers_pub_->publish(markers);

  if(config_.measure){
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count() / 1000.0;

    std_msgs::msg::Float32 latency_msg;
    latency_msg.data = static_cast<float>(duration);
    latency_pub_->publish(latency_msg);
  }
}

std::vector<f110_msgs::msg::Obstacle> LocalPlannerNode::filter_obstacles(){
  std::vector<f110_msgs::msg::Obstacle> close_obstacles;

  if(!obstacles_ || obstacles_->obstacles.empty()){
    return close_obstacles;
  }

  for(const auto& obs : obstacles_->obstacles){
    if(std::abs(obs.d_center) > config_.obs_traj_tresh){
      continue;
    }

    auto predicted_obs = predict_obstacle_movement(obs);

    double dist_in_front = (predicted_obs.s_center - cur_s_);
    if(dist_in_front < 0){
      dist_in_front += gb_max_s_;
    }

    if(dist_in_front < config_.lookahead && dist_in_front > 0){
      close_obstacles.push_back(predicted_obs);
    }
  }

  return close_obstacles;
}

f110_msgs::msg::Obstacle LocalPlannerNode::find_closest_obstacle(const std::vector<f110_msgs::msg::Obstacle>& obstacles){
  if(obstacles.empty()){
    return f110_msgs::msg::Obstacle();
  }

  auto closest = std::min_element(obstacles.begin(), obstacles.end(),
   [this](const f110_msgs::msg::Obstacle& a, const f110_msgs::msg::Obstacle& b){
      double dist_a = (a.s_center - cur_s_);
      if(dist_a < 0) dist_a += gb_max_s_;

      double dist_b = (b.s_center - cur_s_);
      if(dist_b < 0) dist_b += gb_max_s_;

      return dist_a < dist_b;
    });

  return *closest;
}

f110_msgs::msg::Obstacle LocalPlannerNode::predict_obstacle_movement(
  const f110_msgs::msg::Obstacle& obs)
{
  f110_msgs::msg::Obstacle predicted = obs;
  
  // 간단한 constant velocity 모델 사용
  if ((obs.s_center - cur_s_) < 10.0) {  // TODO: 파라미터로 만들기
    double delta_s = config_.fixed_pred_time * obs.vs;
    double delta_d = config_.fixed_pred_time * obs.vd;
    
    predicted.s_start += delta_s;
    predicted.s_center += delta_s;
    predicted.s_end += delta_s;
    
    // Wraparound 처리
    predicted.s_start = std::fmod(predicted.s_start + gb_max_s_, gb_max_s_);
    predicted.s_center = std::fmod(predicted.s_center + gb_max_s_, gb_max_s_);
    predicted.s_end = std::fmod(predicted.s_end + gb_max_s_, gb_max_s_);
    
    predicted.d_left += delta_d;
    predicted.d_center += delta_d;
    predicted.d_right += delta_d;
    
    // 예측된 장애물 위치 마커 발행
    // TODO: 시각화 마커 생성
  }
  
  return predicted;
}

std::pair<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray>
LocalPlannerNode::generate_evasion_trajectory(
  const f110_msgs::msg::Obstacle& obstacle)
{
  f110_msgs::msg::OTWpntArray wpnts;
  visualization_msgs::msg::MarkerArray markers;
  
  wpnts.header.stamp = now();
  wpnts.header.frame_id = "map";
  
  // 1. 초기 경로 생성 (전역 경로에서 일부 가져오기)
  std::vector<Eigen::Vector2d> initial_path = get_initial_path_from_global(obstacle);
  
  if (initial_path.empty()) {
    return std::make_pair(wpnts, markers);
  }
  
  // 2. 장애물을 포인트로 변환
  std::vector<Eigen::Vector2d> obstacle_points = convert_obstacle_to_points(obstacle);
  
  // 3. TEB 파라미터 설정
  double v_max = gb_vmax_;
  double a_max = 2.0;  // TODO: 파라미터로 만들기
  double rho_min = 1.0;  // TODO: 파라미터로 만들기
  double wheelbase = 0.33;  // F1Tenth 차량 휠베이스
  
  // 4. TEB 객체 생성 및 최적화
  TEB teb(initial_path, obstacle_points, v_max, a_max, rho_min, wheelbase);
  teb.optimize(50);  // 50번 반복 최적화
  
  // 5. 최적화된 경로를 waypoint로 변환
  auto path_with_velocities = teb.get_path_with_velocities();
  
  for (size_t i = 0; i < path_with_velocities.size(); i++) {
    const auto& [pos, vel] = path_with_velocities[i];
    
    // Frenet 좌표로 변환 (간단한 근사)
    double s = cur_s_ + i * 0.1;  // TODO: 실제 Frenet 변환 사용
    double d = 0.0;  // TODO: 실제 Frenet 변환 사용
    
    f110_msgs::msg::Wpnt wpnt;
    wpnt.id = static_cast<int>(i);
    wpnt.x_m = pos.x();
    wpnt.y_m = pos.y();
    wpnt.s_m = s;
    wpnt.d_m = d;
    wpnt.vx_mps = vel;
    
    wpnts.wpnts.push_back(wpnt);
    
    // 시각화 마커 생성
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = vel / gb_vmax_ * 0.5;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = vel / gb_vmax_;
    marker.color.a = 1.0;
    marker.color.r = 0.75;
    marker.color.b = 0.75;
    if (config_.from_bag) {
      marker.color.g = 0.75;
    }
    markers.markers.push_back(marker);
  }
  
  // 6. 가장 가까운 장애물 마커 발행
  visualization_msgs::msg::Marker obs_marker;
  obs_marker.header.stamp = now();
  obs_marker.header.frame_id = "map";
  obs_marker.type = visualization_msgs::msg::Marker::SPHERE;
  obs_marker.action = visualization_msgs::msg::Marker::ADD;
  // TODO: 장애물 위치를 Cartesian으로 변환
  obs_marker.pose.position.x = 0.0;  // TODO: 실제 변환
  obs_marker.pose.position.y = 0.0;  // TODO: 실제 변환
  obs_marker.scale.x = 0.5;
  obs_marker.scale.y = 0.5;
  obs_marker.scale.z = 0.5;
  obs_marker.color.a = 0.8;
  obs_marker.color.r = 1.0;
  obs_marker.color.g = 0.65;
  obs_marker.color.b = 0.65;
  considered_obs_pub_->publish(obs_marker);
  
  return std::make_pair(wpnts, markers);
}

std::vector<Eigen::Vector2d> LocalPlannerNode::get_initial_path_from_global(
  const f110_msgs::msg::Obstacle& obstacle)
{
  std::vector<Eigen::Vector2d> path;
  
  if (!global_wpnts_scaled_ || global_wpnts_scaled_->wpnts.empty()) {
    return path;
  }
  
  // 장애물 앞뒤로 경로 추출
  double obs_s = obstacle.s_center;
  double start_s = std::max(0.0, obs_s - 5.0);  // 장애물 앞 5m
  double end_s = std::min(gb_max_s_, obs_s + 5.0);  // 장애물 뒤 5m
  
  for (const auto& wpnt : global_wpnts_scaled_->wpnts) {
    if (wpnt.s_m >= start_s && wpnt.s_m <= end_s) {
      path.emplace_back(wpnt.x_m, wpnt.y_m);
    }
  }
  
  return path;
}

std::vector<Eigen::Vector2d> LocalPlannerNode::convert_obstacle_to_points(
  const f110_msgs::msg::Obstacle& obstacle)
{
  std::vector<Eigen::Vector2d> points;
  
  // 장애물을 간단한 사각형으로 근사 (4개 점)
  // TODO: Frenet → Cartesian 변환 사용
  // 지금은 간단히 s, d 좌표를 사용
  double s = obstacle.s_center;
  double d_left = obstacle.d_left;
  double d_right = obstacle.d_right;
  // TODO: s_start, s_end를 사용하여 장애물의 길이 방향도 고려
  // double s_start = obstacle.s_start;
  // double s_end = obstacle.s_end;
  
  // 간단한 근사: 전역 경로에서 해당 s 위치의 x, y 찾기
  if (global_wpnts_scaled_ && !global_wpnts_scaled_->wpnts.empty()) {
    for (const auto& wpnt : global_wpnts_scaled_->wpnts) {
      if (std::abs(wpnt.s_m - s) < 0.1) {
        // 장애물의 4개 모서리 점 생성
        points.emplace_back(wpnt.x_m + d_left * std::cos(wpnt.psi_rad + M_PI/2),
                           wpnt.y_m + d_left * std::sin(wpnt.psi_rad + M_PI/2));
        points.emplace_back(wpnt.x_m + d_right * std::cos(wpnt.psi_rad + M_PI/2),
                           wpnt.y_m + d_right * std::sin(wpnt.psi_rad + M_PI/2));
        break;
      }
    }
  }
  
  return points;
}

visualization_msgs::msg::MarkerArray LocalPlannerNode::create_delete_all_marker()
{
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker del_mrk;
  
  del_mrk.header.stamp = now();
  del_mrk.header.frame_id = "map";
  del_mrk.action = visualization_msgs::msg::Marker::DELETEALL;
  
  markers.markers.push_back(del_mrk);
  return markers;
}

visualization_msgs::msg::MarkerArray LocalPlannerNode::create_race_line_markers(
  const f110_msgs::msg::WpntArray::SharedPtr& waypoints)
{
  visualization_msgs::msg::MarkerArray markers;
  
  if (!waypoints || waypoints->wpnts.empty()) {
    return markers;
  }
  
  // 최대 속도 계산 (이미 globalWaypointsCallback에서 계산했지만 안전을 위해 다시 계산)
  double max_vx_mps = 0.0;
  for (const auto& wpnt : waypoints->wpnts) {
    if (wpnt.vx_mps > max_vx_mps) {
      max_vx_mps = wpnt.vx_mps;
    }
  }
  
  // max_vx_mps가 0이면 기본값 사용
  if (max_vx_mps < 0.1) {
    max_vx_mps = 1.0;
  }
  
  // 각 waypoint를 마커로 변환
  for (size_t i = 0; i < waypoints->wpnts.size(); i++) {
    const auto& wpnt = waypoints->wpnts[i];
    
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = "map";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 위치 설정
    marker.pose.position.x = wpnt.x_m;
    marker.pose.position.y = wpnt.y_m;
    marker.pose.position.z = wpnt.vx_mps / max_vx_mps / 2.0;  // 속도에 비례한 높이
    
    // 방향 설정
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    
    // 크기 설정
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = wpnt.vx_mps / max_vx_mps;  // 속도에 비례한 높이
    
    // 색상 설정 (빨간색, 속도에 따라 밝기 조절)
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    markers.markers.push_back(marker);
  }
  
  return markers;
}

void LocalPlannerNode::obstaclesCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  obstacles_ = msg;
}

void LocalPlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_state_ = msg;
  // Frenet 좌표계에서 s, d, vs 추출
  cur_s_ = msg->pose.pose.position.x;   // s 좌표
  cur_d_ = msg->pose.pose.position.y;    // d 좌표
  cur_vs_ = msg->twist.twist.linear.x;   // 속도
}

void LocalPlannerNode::globalWaypointsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  global_wpnts_ = msg;

  if (msg && !msg->wpnts.empty()) {
    // 최대 속도 계산
    gb_vmax_ = 0.0;
    for (const auto& wpnt : msg->wpnts) {
      if (wpnt.vx_mps > gb_vmax_) {
        gb_vmax_ = wpnt.vx_mps;
      }
    }
    
    // 최대 인덱스와 최대 s 값
    gb_max_idx_ = msg->wpnts.back().id;
    gb_max_s_ = msg->wpnts.back().s_m;
    
    // 레이스라인 시각화 마커 발행
    auto race_line_markers = create_race_line_markers(msg);
    race_line_markers_pub_->publish(race_line_markers);
  }
}

void LocalPlannerNode::scaledWaypointsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  global_wpnts_scaled_ = msg;
}

bool LocalPlannerNode::check_messages_ready()
{
  static bool state_logged = false;
  static bool gb_logged = false;
  static bool scaled_gb_logged = false;
  
  bool state_ready = (current_state_ != nullptr);
  bool gb_ready = (global_wpnts_ != nullptr && !global_wpnts_->wpnts.empty());
  bool scaled_gb_ready = (global_wpnts_scaled_ != nullptr);
  
  if (state_ready && !state_logged) {
    RCLCPP_INFO(get_logger(), "Received State message.");
    state_logged = true;
  }
  
  if (gb_ready && !gb_logged) {
    RCLCPP_INFO(get_logger(), "Received Global Waypoints message.");
    gb_logged = true;
  }
  
  if (scaled_gb_ready && !scaled_gb_logged) {
    RCLCPP_INFO(get_logger(), "Received Scaled Global Waypoints message.");
    scaled_gb_logged = true;
  }
  
  return state_ready && gb_ready && scaled_gb_ready;
}

}  // namespace teb_local_planner

RCLCPP_COMPONENTS_REGISTER_NODE(teb_local_planner::LocalPlannerNode)

