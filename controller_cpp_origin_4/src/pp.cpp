#include "pp.hpp"
#include "stanley.hpp"
#include "curvature.hpp"
#include <limits>

/**
 * 순수추종(Pure Pursuit) + 스탠리(Stanley) 하이브리드 컨트롤러
 * - 곡률이 커지거나 곡률 변화량이 커질수록 스탠리 비중이 증가한다.
 */

PP_Controller::PP_Controller(
    double t_clip_min,
    double t_clip_max,
    double m_l1,
    double q_l1,
    double speed_lookahead,
    double lat_err_coeff,
    double acc_scaler_for_steer,
    double dec_scaler_for_steer,
    double start_scale_speed,
    double end_scale_speed,
    double downscale_factor,
    double speed_lookahead_for_steer,

    bool   prioritize_dyn,
    double trailing_gap,
    double trailing_p_gain,
    double trailing_i_gain,
    double trailing_d_gain,
    double blind_trailing_speed,
    double trailing_to_gbtrack_speed_scale,

    double loop_rate,
    double wheelbase,

    Logger logger_info,
    Logger logger_warn)

  : t_clip_min(t_clip_min), t_clip_max(t_clip_max),
    m_l1(m_l1), q_l1(q_l1),
    speed_lookahead(speed_lookahead), lat_err_coeff(lat_err_coeff),
    acc_scaler_for_steer(acc_scaler_for_steer), dec_scaler_for_steer(dec_scaler_for_steer),
    start_scale_speed(start_scale_speed), end_scale_speed(end_scale_speed),
    downscale_factor(downscale_factor), speed_lookahead_for_steer(speed_lookahead_for_steer),
    prioritize_dyn(prioritize_dyn),
    trailing_gap(trailing_gap), trailing_p_gain(trailing_p_gain),
    trailing_i_gain(trailing_i_gain), trailing_d_gain(trailing_d_gain),
    blind_trailing_speed(blind_trailing_speed),
    trailing_to_gbtrack_speed_scale(trailing_to_gbtrack_speed_scale),
    loop_rate(loop_rate), wheelbase(wheelbase),
    logger_info(logger_info), logger_warn(logger_warn) {}

PP_Controller::Output
PP_Controller::main_loop(const std::string& state,
                         const std::optional<Pose3>& position_in_map,
                         const std::vector<WpRow>& waypoint_array_in_map,
                         double speed_now,
                         const std::optional<Opp5>& opponent,
                         const std::optional<Fren4>& position_in_map_frenet,
                         const std::vector<double>& acc_now,
                         double track_length)
{
  // 업데이트
  state_         = state;
  if (position_in_map) pose_ = *position_in_map;
  wp_            = waypoint_array_in_map;
  speed_now_     = speed_now;
  opp_           = opponent;
  frenet_        = position_in_map_frenet;
  acc_now_       = acc_now;
  track_length_  = track_length;

  // PREPROCESS
  const double yaw = pose_[2];
  const Vec2 v{ std::cos(yaw)*speed_now_, std::sin(yaw)*speed_now_ };

  // lateral error (from frenet d)
  auto [lat_e_norm, lateral_error] = calc_lateral_error_norm();
  auto [L1_point, L1_distance] = calc_L1_point(lateral_error);


  // LONGITUDINAL
  const double adv_ts_sp = speed_lookahead;
  const Vec2 la_pos{ pose_[0] + v[0]*adv_ts_sp, pose_[1] + v[1]*adv_ts_sp };
  const int idx_la = nearest_waypoint(la_pos, wp_);
  double global_speed = (idx_la >= 0) ? wp_[idx_la][2] : 0.0;

  if (state_ == "StateType.TRAILING" && opp_ && frenet_) {
    speed_command_ = trailing_controller(global_speed);
  } else if (state_ == "StateType.TRAILING_TO_GBTRACK") {
    speed_command_ = global_speed * trailing_to_gbtrack_speed_scale;
  } else {
    i_gap_ = 0.0;
    speed_command_ = global_speed;
  }

  // 곡률 최신값을 반영해 감속
  speed_command_ = speed_adjust_lat_err(speed_command_, lat_e_norm);
  speed_command_ = speed_adjust_heading(speed_command_);

  // POSTPROCESS ...
  double speed = std::isfinite(speed_command_) ? std::max(speed_command_, 0.0) : 0.0;

  // LATERAL (하이브리드: 순수추종 + 스탠리)
  double steering_angle = calc_hybrid_steering(L1_point, L1_distance, yaw, lat_e_norm, v);

  return {speed, 0.0, 0.0, steering_angle, L1_point, L1_distance, idx_nearest_wp_};
}

int PP_Controller::nearest_waypoint(const Vec2& position, const std::vector<WpRow>& waypoints) const {
  if (waypoints.empty()) return -1;
  int best = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int i=0;i<(int)waypoints.size();++i) {
    double dx = position[0] - waypoints[i][0];
    double dy = position[1] - waypoints[i][1];
    double d2 = dx*dx + dy*dy;
    if (d2 < best_d2) { best_d2 = d2; best = i; }
  }
  return best;
}

PP_Controller::Vec2
PP_Controller::waypoint_at_distance_before_car(double distance,
                                               const std::vector<WpRow>& waypoints,
                                               int idx_waypoint_behind_car) const
{
  if (distance <= 0.0) distance = t_clip_min;
  const double waypoints_distance = 0.1; // 동일 가정
  const int d_index = static_cast<int>(distance/waypoints_distance + 0.5);
  const int idx = std::min((int)waypoints.size()-1, idx_waypoint_behind_car + d_index);
  return Vec2{ waypoints[idx][0], waypoints[idx][1] };
}

std::pair<double,double> PP_Controller::calc_lateral_error_norm() const {
  // frenet d 사용
  double lateral_error = 0.0;
  if (frenet_) lateral_error = std::abs((*frenet_)[1]); // d
  const double max_lat_e = 0.5, min_lat_e = 0.0;
  const double lat_e_clip = PP_Controller::clip(lateral_error, min_lat_e, max_lat_e);
  const double lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e));
  return {lat_e_norm, lateral_error};
}

double PP_Controller::speed_adjust_lat_err(double global_speed, double lat_e_norm) const {
  constexpr double curvature_norm_scale = 0.65;  // calc_hybrid_steering과 동일한 값 사용
  const double lat_e_coeff = clip(lat_err_coeff, 0.0, 1.0);
  const double lat_e = lat_e_norm * 2.0;
  const double curv = clip(curvature_waypoints_ / curvature_norm_scale, 0.0, 1.0);
  return global_speed * (1.0 - lat_e_coeff + lat_e_coeff*std::exp(-lat_e*curv));
}

double PP_Controller::speed_adjust_heading(double speed_command) const {
  if (wp_.empty()) return speed_command;

  // 최근접 웨이포인트 인덱스를 내부에서 계산 (순서 의존성 제거)
  const int idx = nearest_waypoint({pose_[0], pose_[1]}, wp_);
  if (idx < 0 || idx >= static_cast<int>(wp_.size())) return speed_command;

  const double heading     = pose_[2];
  const double map_heading = wp_[idx][6]; // WpRow[6] = psi (heading)

  // 래핑된 각도 차이 dpsi ∈ [-pi, pi]
  const double dpsi = std::atan2(std::sin(heading - map_heading), std::cos(heading - map_heading));
  const double heading_error = std::abs(dpsi);

  // 임계값 및 최소 스케일
  const double ok_thresh   = M_PI / 9.0; // ≈ 20 degrees
  const double hard_thresh = M_PI / 2.0;  // 90 degrees
  const double min_scale   = 0.7;         // 최소 70%

  double scale = 1.0;
  if (heading_error <= ok_thresh) {
    scale = 1.0;
  } else if (heading_error < hard_thresh) {
    // 20°~90° 구간에서 1.0 → 0.5 선형 감소
    const double t = (heading_error - ok_thresh) / (hard_thresh - ok_thresh); // ∈ (0,1)
    scale = min_scale + (1.0 - min_scale) * (1.0 - t);
  } else {
    scale = min_scale;
  }

  return speed_command * scale;
}

double PP_Controller::acc_scaling(double steer) const {
  double mean_acc = 0.0;
  if (!acc_now_.empty()) {
    for (double a : acc_now_) mean_acc += a;
    mean_acc /= (double)acc_now_.size();
  }
  if (mean_acc >= 1.0)      steer *= acc_scaler_for_steer;
  else if (mean_acc <= -1.0)steer *= dec_scaler_for_steer;
  return steer;
}

double PP_Controller::speed_steer_scaling(double steer, double speed) const {
  const double speed_diff = std::max(0.1, end_scale_speed - start_scale_speed);
  const double factor = 1.0 - clip((speed - start_scale_speed)/speed_diff, 0.0, 1.0) * downscale_factor;
  return steer * factor;
}

double PP_Controller::trailing_controller(double global_speed) {
  // opp_[2]=vs, frenet_[0]=s, [2]=vs
  const double opp_s = (*opp_)[0];
  const double opp_vs= (*opp_)[2];
  const bool opp_visible = ((*opp_)[4] > 0.5);

  const double my_s  = (*frenet_)[0];
  const double my_vs = (*frenet_)[2];

  gap_ = std::fmod(opp_s - my_s + track_length_, track_length_);
  gap_actual_ = gap_;
  gap_should_ = trailing_gap;
  gap_error_  = gap_should_ - gap_actual_;
  v_diff_     = my_vs - opp_vs;
  i_gap_ = clip(i_gap_ + gap_error_/loop_rate, -10.0, 10.0);

  const double p_value = gap_error_ * trailing_p_gain;
  const double d_value = v_diff_    * trailing_d_gain;
  const double i_value = i_gap_     * trailing_i_gain;

  trailing_command_ = clip(opp_vs - p_value - i_value - d_value, 0.0, global_speed);
  if (!opp_visible && gap_actual_ > gap_should_) {
    trailing_command_ = std::max(blind_trailing_speed, trailing_command_);
  }
  return trailing_command_;
}

std::pair<PP_Controller::Vec2,double>
PP_Controller::calc_L1_point(double lateral_error) {
  idx_nearest_wp_ = nearest_waypoint({pose_[0], pose_[1]}, wp_);
  if (idx_nearest_wp_ < 0) idx_nearest_wp_ = 0;

  if ((int)wp_.size() - idx_nearest_wp_ > 2) {
    // 앞쪽 5m 곡률 평균
    constexpr double curvature_horizon_m = 5.0;   // 필요하면 7.0 등으로 조절
    constexpr double waypoint_res_m = 0.1;        // 웨이포인트 간격 0.1m 가정
    const int max_points_ahead = static_cast<int>(curvature_horizon_m / waypoint_res_m);
    
    double sum = 0.0;
    int cnt = 0;
    const int last_idx = std::min(
        idx_nearest_wp_ + max_points_ahead,
        static_cast<int>(wp_.size()) - 1
    );
    
    for (int i = idx_nearest_wp_; i <= last_idx; ++i) {
      sum += std::abs(wp_[i][5]);  // WpRow[5] = kappa
      ++cnt;
    }
    curvature_waypoints_ = (cnt > 0) ? (sum / static_cast<double>(cnt)) : 0.0;
  }

  double L1_distance = q_l1 + speed_now_ * m_l1;
  const double lower_bound = std::max(t_clip_min, std::sqrt(2.0)*std::abs(lateral_error));
  L1_distance = clip(L1_distance, lower_bound, t_clip_max);

  Vec2 L1_point = waypoint_at_distance_before_car(L1_distance, wp_, idx_nearest_wp_);
  return {L1_point, L1_distance};
}

/** 하이브리드 스티어 계산: 순수추종 + 스탠리 */
double PP_Controller::calc_hybrid_steering(const Vec2& L1_point,
                                          double L1_distance,
                                          double yaw,
                                          double lat_e_norm,
                                          const Vec2& v)
{
  // 곡률 정규화 스케일 (0.4 ~ 0.8 사이에서 튜닝 가능, 기본값 0.65)
  constexpr double curvature_norm_scale = 0.45;

  // 순수추종 스티어
  const double pp = calc_pp_steering_angle(L1_point, L1_distance, yaw, lat_e_norm, v);

  // 스탠리 스티어
  const double stanley = calc_stanley_angle(lat_e_norm, yaw);

  // 곡률/곡률 변화 기반 가중치 계산
  const double curvature_now = curvature_waypoints_;
  const double curvature_future = estimate_future_curvature_change();

  // 곡률이 커질수록 스탠리 가중 ↑, 변화량 커지면 추가 가중
  const double w_curv = clip(curvature_now / curvature_norm_scale, 0.0, 1.0);
  const double w_delta = clip(std::abs(curvature_future) / curvature_norm_scale, 0.0, 1.0);
  double stanley_weight_raw = clip(0.5 * w_curv + 0.5 * w_delta, 0.0, 1.0);

  // 가중치 smoothing: 반응 빠르게 (0.9 -> 0.6)
  static double stanley_weight_prev = 0.0;
  constexpr double weight_alpha = 0.6;  // 0.9 -> 0.6 : 반응 더 빠르게
  double stanley_weight = weight_alpha * stanley_weight_prev + (1.0 - weight_alpha) * stanley_weight_raw;
  stanley_weight_prev = stanley_weight;

  // 스탠리 비중 상한 올려서 예민하게 (0.35 -> 0.5, 필요하면 0.6으로 조절)
  stanley_weight = clip(stanley_weight, 0.0, 0.5);
  
  // linear interpolation
  double hybrid_steer = stanley_weight * stanley + (1.0 - stanley_weight) * pp;

  // Low-pass filter: 조향 반응 빠르게 (0.97 -> 0.85)
  static double steer_prev = 0.0;
  const double alpha = 0.85;   // 0.97 -> 0.85 : 더 예민한 조향

  double steer_filtered = alpha * steer_prev + (1.0 - alpha) * hybrid_steer;
  steer_prev = steer_filtered;

  return steer_filtered;
}

/** 외부에서 스탠리 각도와 가중치를 받아서 하이브리드 스티어 계산 (노드 분리용) */
double PP_Controller::calc_hybrid_steering_external(double pp_steer, double stanley_angle, double stanley_weight)
{
  // linear interpolation
  double hybrid_steer = stanley_weight * stanley_angle + (1.0 - stanley_weight) * pp_steer;

  // Low-pass filter: 조향 반응 빠르게 (0.97 -> 0.85)
  static double steer_prev = 0.0;
  const double alpha = 0.85;   // 0.97 -> 0.85 : 더 예민한 조향

  double steer_filtered = alpha * steer_prev + (1.0 - alpha) * hybrid_steer;
  steer_prev = steer_filtered;

  return steer_filtered;
} 

/** 순수추종 스티어링 각도 계산 (기존 로직 유지) */
double PP_Controller::calc_pp_steering_angle(const Vec2& L1_point,
                                             double L1_distance,
                                             double yaw,
                                             double lat_e_norm,
                                             const Vec2& v)  
{
  // speed lookahead for steering
  double speed_la_for_lu = 0.0;
  if (state_ == "StateType.TRAILING" && opp_) {
    speed_la_for_lu = speed_now_;
  } else {
    const double adv_ts_st = speed_lookahead_for_steer;
    const Vec2 la_pos{ pose_[0] + v[0]*adv_ts_st, pose_[1] + v[1]*adv_ts_st };
    const int idx = nearest_waypoint(la_pos, wp_);
    speed_la_for_lu = (idx>=0) ? wp_[idx][2] : speed_now_;
  }
  const double speed_for_lu = speed_adjust_lat_err(speed_la_for_lu, lat_e_norm);

  // eta 계산: 표준 Pure Pursuit 공식 사용
  const Vec2 L1_vec{ L1_point[0] - pose_[0], L1_point[1] - pose_[1] };
  double eta = 0.0;
  if (std::hypot(L1_vec[0], L1_vec[1]) <= 1e-8) {
    logger_warn(std::string("[PP] norm(L1_vector)==0, eta=0"));
  } else {
    // 표준 Pure Pursuit: atan2를 사용한 eta 계산
    double alpha = std::atan2(L1_vec[1], L1_vec[0]) - yaw;
    eta = alpha;
  }

  // 표준 Pure Pursuit 조향각 계산
  double steer = std::atan2(2.0 * wheelbase * std::sin(eta), L1_distance);

  steer = acc_scaling(steer);
  steer = speed_steer_scaling(steer, speed_for_lu);

  // velocity-based scaling
  steer *= clip(1.0 + (speed_now_/10.0), 1.0, 1.25);

  // rate limit
  const double threshold = 0.42;
  if (std::abs(steer - curr_steering_angle_) > threshold) {
    // logger_info("[PP] steering angle clipped");
  }
  steer = clip(steer, curr_steering_angle_ - threshold, curr_steering_angle_ + threshold);
  curr_steering_angle_ = steer;

  return steer;
}

/** 스탠리 각도 계산 (stanley 함수 사용) */
double PP_Controller::calc_stanley_angle(double lat_e_norm, double yaw)
{
  return Stanley::calc_angle(
    yaw, lat_e_norm, idx_nearest_wp_, wp_, frenet_, speed_now_, logger_warn
  );
}

/** 미래 곡률 변화량 추정 (curvature 함수 사용) */
double PP_Controller::estimate_future_curvature_change()
{
  return Curvature::estimate_future_curvature_change(idx_nearest_wp_, wp_);
}

// 기존 calc_steering_angle은 하이브리드 버전으로 대체
double PP_Controller::calc_steering_angle(const Vec2& L1_point, double L1_distance,
                                          double yaw, double lat_e_norm,
                                          const Vec2& v)
{
  // 하이브리드 스티어링 사용
  return calc_hybrid_steering(L1_point, L1_distance, yaw, lat_e_norm, v);
}
