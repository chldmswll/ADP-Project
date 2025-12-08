#include "spline_planner_cpp/spline_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <numeric>
#include <memory>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "spline_planner_cpp/fuzzy_controller.hpp"

using namespace std::chrono_literals;

// Helper function for positive modulo
double fmod_pos(double a, double m) {
  double r = std::fmod(a, m);
  if (r < 0) r += m;
  return r;
}

// Helper function for clipping
double clip(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

ObstacleSpliner::ObstacleSpliner()
  : Node("spliner_node"),
    gb_vmax_(0.0),
    gb_max_idx_(0),
    gb_max_s_(0.0),
    cur_s_(0.0),
    cur_d_(0.0),
    cur_vs_(0.0),
    lookahead_(10.0),
    from_bag_(false),
    measuring_(false),
    pre_apex_0_(-4.0),
    pre_apex_1_(-3.0),
    pre_apex_2_(-1.5),
    post_apex_0_(2.0),
    post_apex_1_(3.0),
    post_apex_2_(4.0),
    evasion_dist_(0.77),
    obs_traj_tresh_(0.3),
    spline_bound_mindist_(0.1),
    fixed_pred_time_(0.15),
    kd_obs_pred_(1.0)
{
  // Initialize last_switch_time
  last_switch_time_ = this->now();
  last_ot_side_ = "";

  // Declare parameters
  this->declare_parameter("from_bag", false);
  this->declare_parameter("measure", false);
  
  from_bag_ = this->get_parameter("from_bag").as_bool();
  measuring_ = this->get_parameter("measure").as_bool();

  // Declare dynamic parameters with descriptors
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = 0.1;
  range.to_value = 8.0;
  range.step = 0.001;

  rcl_interfaces::msg::ParameterDescriptor pd;
  pd.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  pd.floating_point_range = {range};

  this->declare_parameter("pre_apex_0", std::abs(pre_apex_0_), pd);
  this->declare_parameter("pre_apex_1", std::abs(pre_apex_1_), pd);
  this->declare_parameter("pre_apex_2", std::abs(pre_apex_2_), pd);
  this->declare_parameter("post_apex_0", post_apex_0_, pd);
  this->declare_parameter("post_apex_1", post_apex_1_, pd);
  this->declare_parameter("post_apex_2", post_apex_2_, pd);
  this->declare_parameter("evasion_dist", evasion_dist_, pd);
  this->declare_parameter("obs_traj_tresh", obs_traj_tresh_, pd);
  this->declare_parameter("spline_bound_mindist", spline_bound_mindist_, pd);
  this->declare_parameter("fixed_pred_time", fixed_pred_time_, pd);
  this->declare_parameter("kd_obs_pred", kd_obs_pred_, pd);

  // Setup subscriptions
  obs_sub_ = this->create_subscription<f110_msgs::msg::ObstacleArray>(
    "/perception/obstacles", 10,
    std::bind(&ObstacleSpliner::obs_cb, this, std::placeholders::_1));
  
  state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", 10,
    std::bind(&ObstacleSpliner::state_cb, this, std::placeholders::_1));
  
  gb_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints", 10,
    std::bind(&ObstacleSpliner::gb_cb, this, std::placeholders::_1));
  
  gb_scaled_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints_scaled", 10,
    std::bind(&ObstacleSpliner::gb_scaled_cb, this, std::placeholders::_1));

  // Setup publishers
  mrks_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planner/avoidance/markers", 10);
  
  evasion_pub_ = this->create_publisher<f110_msgs::msg::OTWpntArray>(
    "/planner/avoidance/otwpnts", 10);
  
  closest_obs_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/planner/avoidance/considered_OBS", 10);
  
  pub_propagated_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "/planner/avoidance/propagated_obs", 10);

  if (measuring_) {
    latency_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/planner/avoidance/latency", 10);
  }

  // Setup parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ObstacleSpliner::dyn_param_cb, this, std::placeholders::_1));

  // Wait for messages
  wait_for_messages();

  // Initialize converter
  initialize_converter();

  // Create timer (20 Hz)
  timer_ = this->create_wall_timer(
    50ms, std::bind(&ObstacleSpliner::spliner_loop, this));
}

void ObstacleSpliner::obs_cb(const f110_msgs::msg::ObstacleArray::SharedPtr msg) {
  obs_ = *msg;
}

void ObstacleSpliner::state_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  cur_s_ = msg->pose.pose.position.x;
  cur_d_ = msg->pose.pose.position.y;
  cur_vs_ = msg->twist.twist.linear.x;
}

void ObstacleSpliner::gb_cb(const f110_msgs::msg::WpntArray::SharedPtr msg) {
  gb_wpnts_ = msg;
  if (gb_vmax_ == 0.0 && !msg->wpnts.empty()) {
    double max_v = 0.0;
    for (const auto& wpnt : msg->wpnts) {
      if (wpnt.vx_mps > max_v) {
        max_v = wpnt.vx_mps;
      }
    }
    gb_vmax_ = max_v;
    gb_max_idx_ = msg->wpnts.back().id;
    gb_max_s_ = msg->wpnts.back().s_m;
  }
}

void ObstacleSpliner::gb_scaled_cb(const f110_msgs::msg::WpntArray::SharedPtr msg) {
  gb_scaled_wpnts_ = msg;
}

rcl_interfaces::msg::SetParametersResult ObstacleSpliner::dyn_param_cb(
  const std::vector<rclcpp::Parameter> &params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto& param : params) {
    const std::string& name = param.get_name();
    
    if (name == "pre_apex_0") {
      pre_apex_0_ = -1.0 * param.as_double();
    } else if (name == "pre_apex_1") {
      pre_apex_1_ = -1.0 * param.as_double();
    } else if (name == "pre_apex_2") {
      pre_apex_2_ = -1.0 * param.as_double();
    } else if (name == "post_apex_0") {
      post_apex_0_ = param.as_double();
    } else if (name == "post_apex_1") {
      post_apex_1_ = param.as_double();
    } else if (name == "post_apex_2") {
      post_apex_2_ = param.as_double();
    } else if (name == "evasion_dist") {
      evasion_dist_ = param.as_double();
    } else if (name == "obs_traj_tresh") {
      obs_traj_tresh_ = param.as_double();
    } else if (name == "spline_bound_mindist") {
      spline_bound_mindist_ = param.as_double();
    } else if (name == "fixed_pred_time") {
      fixed_pred_time_ = param.as_double();
    } else if (name == "kd_obs_pred") {
      kd_obs_pred_ = param.as_double();
    }
  }

  // Ensure ascending order
  if (pre_apex_1_ < pre_apex_0_) {
    RCLCPP_INFO(this->get_logger(), "Adjusting pre_apex_1 to ensure ascending order.");
    pre_apex_1_ = pre_apex_0_;
  }
  if (pre_apex_2_ < pre_apex_1_) {
    RCLCPP_INFO(this->get_logger(), "Adjusting pre_apex_2 to ensure ascending order.");
    pre_apex_2_ = pre_apex_1_;
  }
  if (post_apex_0_ < pre_apex_2_) {
    RCLCPP_INFO(this->get_logger(), "Adjusting post_apex_0 to ensure ascending order.");
    post_apex_0_ = pre_apex_2_;
  }
  if (post_apex_1_ < post_apex_0_) {
    RCLCPP_INFO(this->get_logger(), "Adjusting post_apex_1 to ensure ascending order.");
    post_apex_1_ = post_apex_0_;
  }
  if (post_apex_2_ < post_apex_1_) {
    RCLCPP_INFO(this->get_logger(), "Adjusting post_apex_2 to ensure ascending order.");
    post_apex_2_ = post_apex_1_;
  }

  std::vector<double> spline_params = {
    pre_apex_0_, pre_apex_1_, pre_apex_2_, 0.0,
    post_apex_0_, post_apex_1_, post_apex_2_
  };

  RCLCPP_INFO(this->get_logger(),
    "Dynamic reconf triggered new spline params: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f] [m],\n"
    "evasion apex distance: %.3f [m],\n"
    "obstacle trajectory threshold: %.3f [m]\n"
    "obstacle prediction k_d: %.3f, obstacle prediction constant time: %.3f [s]",
    spline_params[0], spline_params[1], spline_params[2], spline_params[3],
    spline_params[4], spline_params[5], spline_params[6],
    evasion_dist_, obs_traj_tresh_, kd_obs_pred_, fixed_pred_time_);

  return result;
}

void ObstacleSpliner::wait_for_messages() {
  RCLCPP_INFO(this->get_logger(), "Spliner node waiting for messages...");
  
  bool state_received = false;
  bool gb_received = false;
  bool scaled_gb_received = false;

  rclcpp::Rate rate(10); // 10 Hz
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(this->get_node_base_interface());
  
  while (rclcpp::ok() && (!state_received || !gb_received || !scaled_gb_received)) {
    executor.spin_some();
    
    if (cur_s_ != 0.0 && !state_received) {
      RCLCPP_INFO(this->get_logger(), "Received State message.");
      state_received = true;
    }
    if (gb_wpnts_ != nullptr && !gb_received) {
      RCLCPP_INFO(this->get_logger(), "Received Global Waypoints message.");
      gb_received = true;
    }
    if (gb_scaled_wpnts_ != nullptr && !scaled_gb_received) {
      RCLCPP_INFO(this->get_logger(), "Received Scaled Global Waypoints message.");
      scaled_gb_received = true;
    }
    
    rate.sleep();
  }
  RCLCPP_INFO(this->get_logger(), "All required messages received. Continuing...");
}

void ObstacleSpliner::initialize_converter() {
  if (!gb_wpnts_ || gb_wpnts_->wpnts.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot initialize converter: no waypoints available");
    return;
  }

  std::vector<double> waypoints_x, waypoints_y, waypoints_psi;
  for (const auto& wpnt : gb_wpnts_->wpnts) {
    waypoints_x.push_back(wpnt.x_m);
    waypoints_y.push_back(wpnt.y_m);
    waypoints_psi.push_back(wpnt.psi_rad);
  }

  converter_ = std::make_unique<FrenetConverter>(
    waypoints_x, waypoints_y, waypoints_psi);
}

void ObstacleSpliner::spliner_loop() {
  auto start = std::chrono::high_resolution_clock::now();

  if (!gb_scaled_wpnts_ || gb_scaled_wpnts_->wpnts.empty()) {
    return;
  }

  f110_msgs::msg::OTWpntArray wpnts;
  visualization_msgs::msg::MarkerArray mrks;

  // If obstacles exist, do splining
  if (!obs_.obstacles.empty()) {
    auto result = do_spline(obs_, gb_scaled_wpnts_->wpnts);
    wpnts = result.first;
    mrks = result.second;
  } else {
    // Delete spline markers
    visualization_msgs::msg::Marker del_mrk;
    del_mrk.header.stamp = this->now();
    del_mrk.action = visualization_msgs::msg::Marker::DELETEALL;
    mrks.markers.push_back(del_mrk);
  }

  // Publish
  if (measuring_) {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std_msgs::msg::Float32 latency_msg;
    latency_msg.data = duration.count() / 1000000.0;
    latency_pub_->publish(latency_msg);
  }

  evasion_pub_->publish(wpnts);
  mrks_pub_->publish(mrks);
}

f110_msgs::msg::Obstacle ObstacleSpliner::predict_obs_movement(
  const f110_msgs::msg::Obstacle& obs, const std::string& mode)
{
  f110_msgs::msg::Obstacle predicted_obs = obs;
  
  double dist_to_obs = fmod_pos(predicted_obs.s_center - cur_s_, gb_max_s_);
  
  if (dist_to_obs < 10.0) {  // TODO make param
    double delta_s = 0.0;
    double delta_d = 0.0;
    
    if (mode == "constant") {
      delta_s = fixed_pred_time_ * predicted_obs.vs;
      delta_d = fixed_pred_time_ * predicted_obs.vd;
    } else if (mode == "adaptive") {
      double ot_distance = fmod_pos(predicted_obs.s_center - cur_s_, gb_max_s_);
      int idx = static_cast<int>(cur_s_ * 10) % static_cast<int>(gb_scaled_wpnts_->wpnts.size());
      double ego_speed = gb_scaled_wpnts_->wpnts[idx].vx_mps;
      double rel_speed = clip(ego_speed - predicted_obs.vs, 0.1, 10.0);
      double ot_time_distance = clip(ot_distance / rel_speed, 0.0, 5.0) * 0.5;
      
      delta_s = ot_time_distance * predicted_obs.vs;
      delta_d = ot_time_distance * predicted_obs.vd;
      delta_d = -(predicted_obs.d_center + delta_d) * 
                std::exp(-std::abs(kd_obs_pred_ * predicted_obs.d_center));
    } else if (mode == "adaptive_velheuristic") {
      double opponent_scaler = 0.7;
      int idx = static_cast<int>(cur_s_ * 10) % static_cast<int>(gb_scaled_wpnts_->wpnts.size());
      double ego_speed = gb_scaled_wpnts_->wpnts[idx].vx_mps;
      double ot_distance = fmod_pos(predicted_obs.s_center - cur_s_, gb_max_s_);
      double rel_speed = (1.0 - opponent_scaler) * ego_speed;
      double ot_time_distance = clip(ot_distance / rel_speed, 0.0, 5.0);
      
      delta_s = ot_time_distance * opponent_scaler * ego_speed;
      delta_d = -(predicted_obs.d_center) * 
                std::exp(-std::abs(kd_obs_pred_ * predicted_obs.d_center));
    } else if (mode == "heuristic") {
      double ot_distance = fmod_pos(predicted_obs.s_center - cur_s_, gb_max_s_);
      double rel_speed = 3.0;
      double ot_time_distance = ot_distance / rel_speed;
      
      delta_d = ot_time_distance * predicted_obs.vd;
      delta_d = -(predicted_obs.d_center + delta_d) * 
                std::exp(-std::abs(kd_obs_pred_ * predicted_obs.d_center));
    }
    
    // Update obstacle position
    predicted_obs.s_start += delta_s;
    predicted_obs.s_center += delta_s;
    predicted_obs.s_end += delta_s;
    predicted_obs.s_start = fmod_pos(predicted_obs.s_start, gb_max_s_);
    predicted_obs.s_center = fmod_pos(predicted_obs.s_center, gb_max_s_);
    predicted_obs.s_end = fmod_pos(predicted_obs.s_end, gb_max_s_);
    
    predicted_obs.d_left += delta_d;
    predicted_obs.d_center += delta_d;
    predicted_obs.d_right += delta_d;
    
    // Publish propagated marker
    if (converter_) {
      std::vector<double> s_vec = {predicted_obs.s_center};
      std::vector<double> d_vec = {predicted_obs.d_center};
      auto resp = converter_->get_cartesian(s_vec, d_vec);
      auto marker = xy_to_point(resp.first[0], resp.second[0], true);
      pub_propagated_->publish(marker);
    }
  }
  
  return predicted_obs;
}

bool ObstacleSpliner::check_ot_side_possible(const std::string& more_space) {
  if (std::abs(cur_d_) > 0.5 && more_space != last_ot_side_) {
    RCLCPP_INFO(this->get_logger(), 
      "Can't switch sides, because we are not on the raceline");
    return false;
  }
  return true;
}

std::pair<std::string, double> ObstacleSpliner::more_space(
  const f110_msgs::msg::Obstacle& obstacle,
  const std::vector<f110_msgs::msg::Wpnt>& gb_wpnts,
  const std::vector<int>& gb_idxs)
{
  if (gb_idxs.empty() || gb_wpnts.empty()) {
    return {"left", 0.0};
  }
  
  int idx = gb_idxs[0];
  double left_gap = std::abs(gb_wpnts[idx].d_left - obstacle.d_left);
  double right_gap = std::abs(gb_wpnts[idx].d_right + obstacle.d_right);
  double min_space = evasion_dist_ + spline_bound_mindist_;
  
  if (right_gap > min_space && left_gap < min_space) {
    double d_apex_right = obstacle.d_right - evasion_dist_;
    if (d_apex_right > 0) {
      d_apex_right = 0;
    }
    return {"right", d_apex_right};
  } else if (left_gap > min_space && right_gap < min_space) {
    double d_apex_left = obstacle.d_left + evasion_dist_;
    if (d_apex_left < 0) {
      d_apex_left = 0;
    }
    return {"left", d_apex_left};
  } else {
    double candidate_d_apex_left = obstacle.d_left + evasion_dist_;
    double candidate_d_apex_right = obstacle.d_right - evasion_dist_;
    
    if (std::abs(candidate_d_apex_left) <= std::abs(candidate_d_apex_right)) {
      if (candidate_d_apex_left < 0) {
        candidate_d_apex_left = 0;
      }
      return {"left", candidate_d_apex_left};
    } else {
      if (candidate_d_apex_right > 0) {
        candidate_d_apex_right = 0;
      }
      return {"right", candidate_d_apex_right};
    }
  }
}

std::pair<f110_msgs::msg::OTWpntArray, visualization_msgs::msg::MarkerArray>
ObstacleSpliner::do_spline(
  const f110_msgs::msg::ObstacleArray& obstacles,
  const std::vector<f110_msgs::msg::Wpnt>& gb_wpnts)
{
  f110_msgs::msg::OTWpntArray wpnts;
  visualization_msgs::msg::MarkerArray mrks;
  
  if (gb_wpnts.size() < 2) {
    return {wpnts, mrks};
  }
  
  // Get spacing between waypoints
  double wpnt_dist = gb_wpnts[1].s_m - gb_wpnts[0].s_m;
  
  // Filter obstacles
  auto close_obs = obs_filtering(obstacles);
  
  if (!close_obs.empty()) {
    // Get closest obstacle
    auto closest_obs_it = std::min_element(close_obs.begin(), close_obs.end(),
      [this](const f110_msgs::msg::Obstacle& a, const f110_msgs::msg::Obstacle& b) {
        return fmod_pos(a.s_center - cur_s_, gb_max_s_) < 
               fmod_pos(b.s_center - cur_s_, gb_max_s_);
      });
    
    f110_msgs::msg::Obstacle closest_obs = *closest_obs_it;
    
    // Get apex
    double s_apex;
    if (closest_obs.s_end < closest_obs.s_start) {
      s_apex = fmod_pos((closest_obs.s_end + gb_max_s_ + closest_obs.s_start) / 2.0, gb_max_s_);
    } else {
      s_apex = (closest_obs.s_end + closest_obs.s_start) / 2.0;
    }
    
    // Approximate next 20 indexes
    std::vector<int> gb_idxs;
    for (int i = 0; i < 20; ++i) {
      int idx = static_cast<int>(s_apex / wpnt_dist + i) % gb_max_idx_;
      gb_idxs.push_back(idx);
    }
    
    // Compute outside direction
    double kappa_sum = 0.0;
    for (int idx : gb_idxs) {
      if (idx < static_cast<int>(gb_wpnts.size())) {
        kappa_sum += gb_wpnts[idx].kappa_radpm;
      }
    }
    std::string outside = (kappa_sum < 0) ? "left" : "right";
    
    // Choose side and compute apex distance
    auto [ot_side, d_apex] = more_space(closest_obs, gb_wpnts, gb_idxs);
    
    // Publish closest obstacle marker
    if (!gb_idxs.empty() && gb_idxs[0] < static_cast<int>(gb_wpnts.size())) {
      auto mrk = xy_to_point(gb_wpnts[gb_idxs[0]].x_m, 
                            gb_wpnts[gb_idxs[0]].y_m, false);
      closest_obs_pub_->publish(mrk);
    }
    
    // TEB 로직으로 경로 생성
    bool danger_flag = false;
    
    if (!check_ot_side_possible(ot_side)) {
      danger_flag = true;
    }
    
    if (!danger_flag && converter_) {
      // 1. 초기 경로 생성 (전역 경로에서 일부 가져오기)
      std::vector<Eigen::Vector2d> initial_path = get_initial_path_from_global(closest_obs);
      
      RCLCPP_DEBUG(this->get_logger(), "Initial path size: %zu", initial_path.size());
      
      if (!initial_path.empty()) {
        // 2. 장애물을 포인트로 변환
        std::vector<Eigen::Vector2d> obstacle_points = convert_obstacle_to_points(closest_obs);
        
        // 3. TEB 파라미터 설정
        double v_max = gb_vmax_;
        double a_max = 2.0;  // TODO: 파라미터로 만들기
        double rho_min = 1.0;  // TODO: 파라미터로 만들기
        double wheelbase = 0.33;  // F1Tenth 차량 휠베이스
        
        // 4. TEB 객체 생성
        teb_local_planner::TEB teb(initial_path, obstacle_points, v_max, a_max, rho_min, wheelbase);
        
        // 5. FLC-TEB: Narrowness와 Turning Complexity 계산
        double narrowness = teb.compute_narrowness();
        double turning_complexity = teb.compute_turning_complexity();
        
        RCLCPP_DEBUG(this->get_logger(), "Narrowness: %.3f, Turning Complexity: %.3f", 
                     narrowness, turning_complexity);
        
        // 6. Fuzzy Logic Controller로 가중치 계산
        teb_local_planner::FuzzyController flc;
        auto weights = flc.compute_weights(narrowness, turning_complexity);
        
        RCLCPP_DEBUG(this->get_logger(), 
                     "FLC Weights - obs: %.1f, sm: %.1f, v: %.1f, acc: %.1f, jerk: %.1f",
                     weights.sigma_obs, weights.sigma_sm, weights.sigma_v, 
                     weights.sigma_acc, weights.sigma_j);
        
        // 7. TEB에 가중치 적용
        teb.set_weights(weights.sigma_obs, weights.sigma_sm, weights.sigma_v, 
                        weights.sigma_acc, weights.sigma_j);
        
        // 8. 최적화 수행
        teb.optimize(10);  // 10번 반복 최적화 (속도 개선)
        
        // 9. 최적화된 경로를 waypoint로 변환
        auto path_with_velocities = teb.get_path_with_velocities();
        
        RCLCPP_DEBUG(this->get_logger(), "TEB generated path with %zu points", path_with_velocities.size());
        
        if (path_with_velocities.empty()) {
          RCLCPP_WARN(this->get_logger(), "TEB path is empty!");
        }
        
        for (size_t i = 0; i < path_with_velocities.size(); i++) {
          const auto& [pos, vel] = path_with_velocities[i];
          
          // Cartesian에서 Frenet로 변환
          std::vector<double> x_vec = {pos.x()};
          std::vector<double> y_vec = {pos.y()};
          auto frenet_coords = converter_->get_frenet(x_vec, y_vec);
          double s = frenet_coords.first[0];
          double d = frenet_coords.second[0];
          
          // Wraparound 처리
          s = fmod_pos(s, gb_max_s_);
          
          // Track bounds 체크
          int gb_wpnt_i = static_cast<int>((s / wpnt_dist)) % gb_max_idx_;
          if (gb_wpnt_i < static_cast<int>(gb_wpnts.size())) {
            if (std::abs(d) > 0.1) {  // spline_resolution
              double tb_dist = (ot_side == "left") ? 
                gb_wpnts[gb_wpnt_i].d_left : gb_wpnts[gb_wpnt_i].d_right;
              
              if (std::abs(d) > std::abs(tb_dist) - spline_bound_mindist_) {
                RCLCPP_INFO(this->get_logger(),
                  "Evasion trajectory too close to TRACKBOUNDS, aborting evasion");
                danger_flag = true;
                break;
              }
            }
            
            // Get velocity from global waypoints
            double vi = (outside == ot_side) ? 
              gb_wpnts[gb_wpnt_i].vx_mps : gb_wpnts[gb_wpnt_i].vx_mps * 0.9;
            vi = std::min(vi, vel);  // TEB에서 계산한 속도와 비교하여 작은 값 사용
            
            f110_msgs::msg::Wpnt wpnt;
            wpnt.id = static_cast<int>(i);
            wpnt.x_m = pos.x();
            wpnt.y_m = pos.y();
            wpnt.s_m = s;
            wpnt.d_m = d;
            wpnt.vx_mps = vi;
            
            wpnts.wpnts.push_back(wpnt);
            
            // 시각화 마커 생성
            auto mrk = xyv_to_markers(pos.x(), pos.y(), vi, mrks.markers.size());
            mrks.markers.push_back(mrk);
          }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Generated %zu waypoints and %zu markers", 
                     wpnts.wpnts.size(), mrks.markers.size());
        
        // LINE_STRIP 마커 추가 (초록색 선으로 표시)
        if (!wpnts.wpnts.empty()) {
          visualization_msgs::msg::Marker line_marker;
          line_marker.header.frame_id = "map";
          line_marker.header.stamp = this->now();
          line_marker.action = visualization_msgs::msg::Marker::ADD;
          line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          line_marker.id = 9999;  // 고유 ID
          line_marker.scale.x = 0.15;  // 선 두께
          line_marker.color.a = 1.0;
          line_marker.color.r = 0.0;
          line_marker.color.g = 1.0;  // 초록색
          line_marker.color.b = 0.0;
          
          for (const auto& wpnt : wpnts.wpnts) {
            geometry_msgs::msg::Point pt;
            pt.x = wpnt.x_m;
            pt.y = wpnt.y_m;
            pt.z = 0.1;  // 약간 위에 표시
            line_marker.points.push_back(pt);
          }
          
          mrks.markers.push_back(line_marker);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Initial path is empty, cannot generate TEB trajectory");
      }
    } else {
      if (danger_flag) {
        RCLCPP_DEBUG(this->get_logger(), "Danger flag set, skipping trajectory generation");
      }
      if (!converter_) {
        RCLCPP_WARN(this->get_logger(), "Frenet converter not initialized");
      }
    }
    
    // Fill OTWpntArray header and metadata
    wpnts.header.stamp = this->now();
    wpnts.header.frame_id = "map";
    
    if (!danger_flag) {
      wpnts.ot_side = ot_side;
      wpnts.ot_line = outside;
      wpnts.side_switch = (last_ot_side_ != ot_side);
      wpnts.last_switch_time = last_switch_time_;
      
      if (last_ot_side_ != ot_side) {
        last_switch_time_ = this->now();
      }
      last_ot_side_ = ot_side;
    } else {
      wpnts.wpnts.clear();
      mrks.markers.clear();
      wpnts.side_switch = true;
      last_switch_time_ = this->now();
    }
  }
  
  return {wpnts, mrks};
}

std::vector<f110_msgs::msg::Obstacle> ObstacleSpliner::obs_filtering(
  const f110_msgs::msg::ObstacleArray& obstacles)
{
  std::vector<f110_msgs::msg::Obstacle> obs_on_traj;
  
  // Filter obstacles on trajectory
  for (const auto& obs : obstacles.obstacles) {
    if (std::abs(obs.d_center) < obs_traj_tresh_) {
      obs_on_traj.push_back(obs);
    }
  }
  
  // Filter obstacles within lookahead
  std::vector<f110_msgs::msg::Obstacle> close_obs;
  for (auto obs : obs_on_traj) {
    obs = predict_obs_movement(obs);
    double dist_in_front = fmod_pos(obs.s_center - cur_s_, gb_max_s_);
    
    if (dist_in_front < lookahead_) {
      close_obs.push_back(obs);
    }
  }
  
  return close_obs;
}

visualization_msgs::msg::Marker ObstacleSpliner::xyv_to_markers(
  double x, double y, double v, size_t id)
{
  visualization_msgs::msg::Marker mrk;
  mrk.header.frame_id = "map";
  mrk.header.stamp = this->now();
  mrk.action = visualization_msgs::msg::Marker::ADD;
  mrk.type = visualization_msgs::msg::Marker::CYLINDER;
  mrk.scale.x = 0.1;
  mrk.scale.y = 0.1;
  mrk.scale.z = v / gb_vmax_;
  mrk.color.a = 1.0;
  mrk.color.b = 0.75;
  mrk.color.r = 0.75;
  if (from_bag_) {
    mrk.color.g = 0.75;
  } else {
    mrk.color.g = 1.0;  // 초록색으로 표시
  }
  mrk.id = static_cast<int>(id);
  mrk.pose.position.x = x;
  mrk.pose.position.y = y;
  mrk.pose.position.z = v / (gb_vmax_ / 2.0);
  mrk.pose.orientation.w = 1.0;
  
  return mrk;
}

visualization_msgs::msg::Marker ObstacleSpliner::xy_to_point(
  double x, double y, bool opponent)
{
  visualization_msgs::msg::Marker mrk;
  mrk.header.frame_id = "map";
  mrk.header.stamp = this->now();
  mrk.type = visualization_msgs::msg::Marker::SPHERE;
  mrk.scale.x = 0.5;
  mrk.scale.y = 0.5;
  mrk.scale.z = 0.5;
  mrk.color.a = 0.8;
  mrk.color.b = 0.65;
  mrk.color.r = opponent ? 1.0 : 0.0;
  mrk.color.g = 0.65;
  mrk.pose.position.x = x;
  mrk.pose.position.y = y;
  mrk.pose.position.z = 0.01;
  mrk.pose.orientation.w = 1.0;
  
  return mrk;
}

f110_msgs::msg::Wpnt ObstacleSpliner::xyv_to_wpnts(
  double s, double d, double x, double y, double v, size_t id)
{
  f110_msgs::msg::Wpnt wpnt;
  wpnt.id = static_cast<int>(id);
  wpnt.x_m = x;
  wpnt.y_m = y;
  wpnt.s_m = s;
  wpnt.d_m = d;
  wpnt.vx_mps = v;
  
  return wpnt;
}

std::vector<Eigen::Vector2d> ObstacleSpliner::get_initial_path_from_global(
  const f110_msgs::msg::Obstacle& obstacle)
{
  std::vector<Eigen::Vector2d> path;
  
  if (!gb_scaled_wpnts_ || gb_scaled_wpnts_->wpnts.empty()) {
    RCLCPP_WARN(this->get_logger(), "No scaled waypoints available for initial path");
    return path;
  }
  
  // 장애물 앞뒤로 경로 추출 (더 넓은 범위)
  double obs_s = fmod_pos(obstacle.s_center, gb_max_s_);
  double cur_s_wrapped = fmod_pos(cur_s_, gb_max_s_);
  
  // 현재 위치에서 장애물까지의 거리 계산
  double dist_to_obs = fmod_pos(obs_s - cur_s_wrapped, gb_max_s_);
  if (dist_to_obs > gb_max_s_ / 2.0) {
    dist_to_obs = gb_max_s_ - dist_to_obs;
  }
  
  // 장애물 앞 10m부터 뒤 10m까지 경로 추출
  double start_s = fmod_pos(obs_s - 10.0, gb_max_s_);
  double end_s = fmod_pos(obs_s + 10.0, gb_max_s_);
  
  // Wraparound 처리
  bool wraps_around = (end_s < start_s);
  
  for (const auto& wpnt : gb_scaled_wpnts_->wpnts) {
    double s_wrapped = fmod_pos(wpnt.s_m, gb_max_s_);
    bool in_range = false;
    
    if (wraps_around) {
      // 경로가 wraparound하는 경우
      in_range = (s_wrapped >= start_s || s_wrapped <= end_s);
    } else {
      // 일반적인 경우
      in_range = (s_wrapped >= start_s && s_wrapped <= end_s);
    }
    
    if (in_range) {
      path.emplace_back(wpnt.x_m, wpnt.y_m);
    }
  }
  
  // 경로가 비어있으면 현재 위치 주변에서 경로 추출
  if (path.empty()) {
    RCLCPP_WARN(this->get_logger(), "Initial path empty, using current position region");
    double start_s2 = fmod_pos(cur_s_wrapped - 5.0, gb_max_s_);
    double end_s2 = fmod_pos(cur_s_wrapped + 15.0, gb_max_s_);
    bool wraps2 = (end_s2 < start_s2);
    
    for (const auto& wpnt : gb_scaled_wpnts_->wpnts) {
      double s_wrapped = fmod_pos(wpnt.s_m, gb_max_s_);
      bool in_range = wraps2 ? (s_wrapped >= start_s2 || s_wrapped <= end_s2) :
                               (s_wrapped >= start_s2 && s_wrapped <= end_s2);
      if (in_range) {
        path.emplace_back(wpnt.x_m, wpnt.y_m);
      }
    }
  }
  
  RCLCPP_DEBUG(this->get_logger(), "Extracted initial path with %zu points (obs_s=%.2f, cur_s=%.2f)", 
               path.size(), obs_s, cur_s_wrapped);
  
  return path;
}

std::vector<Eigen::Vector2d> ObstacleSpliner::convert_obstacle_to_points(
  const f110_msgs::msg::Obstacle& obstacle)
{
  std::vector<Eigen::Vector2d> points;
  
  if (!converter_ || !gb_scaled_wpnts_ || gb_scaled_wpnts_->wpnts.empty()) {
    return points;
  }
  
  // 장애물의 s, d 좌표를 Cartesian으로 변환
  double s = obstacle.s_center;
  double d_left = obstacle.d_left;
  double d_right = obstacle.d_right;
  
  // 간단한 근사: 전역 경로에서 해당 s 위치의 x, y 찾기
  for (const auto& wpnt : gb_scaled_wpnts_->wpnts) {
    if (std::abs(fmod_pos(wpnt.s_m, gb_max_s_) - fmod_pos(s, gb_max_s_)) < 0.1) {
      // 장애물의 좌우 경계점 생성
      double cos_psi = std::cos(wpnt.psi_rad + M_PI/2);
      double sin_psi = std::sin(wpnt.psi_rad + M_PI/2);
      
      points.emplace_back(
        wpnt.x_m + d_left * cos_psi,
        wpnt.y_m + d_left * sin_psi);
      points.emplace_back(
        wpnt.x_m + d_right * cos_psi,
        wpnt.y_m + d_right * sin_psi);
      break;
    }
  }
  
  return points;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleSpliner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

