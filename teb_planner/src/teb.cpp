#include "teb_planner/teb.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace teb_local_planner
{

TEB::TEB(
  const std::vector<Eigen::Vector2d>& initial_path,
  const std::vector<Eigen::Vector2d>& obstacles,
  double v_max,
  double a_max,
  double rho_min,
  double wheelbase)
: obstacles_(obstacles)
, v_max_(v_max)
, a_max_(a_max)
, rho_min_(rho_min)
, wheelbase_(wheelbase)
{
  // 초기 경로를 TEBPose로 변환
  for (size_t i = 0; i < initial_path.size(); i++) {
    double radius = compute_rho(initial_path[i]);
    double theta = 0.0;
    if (i > 0) {
      Eigen::Vector2d dir = initial_path[i] - initial_path[i-1];
      theta = std::atan2(dir.y(), dir.x());
    }
    double delta_t = 0.1;  // 초기 시간 간격
    poses_.emplace_back(initial_path[i], theta, delta_t, radius);
  }
  
  maintain_overlap();
}

double TEB::compute_rho(const Eigen::Vector2d& position) const
{
  // 간단한 SDF: 가장 가까운 장애물까지의 거리
  double min_dist = std::numeric_limits<double>::max();
  
  for (const auto& obs : obstacles_) {
    double dist = (position - obs).norm();
    if (dist < min_dist) {
      min_dist = dist;
    }
  }
  
  // 반경 제한
  const double MAX_RADIUS = 100.0;
  const double MIN_RADIUS = 10.0;
  min_dist = std::max(MIN_RADIUS, std::min(MAX_RADIUS, min_dist));
  
  return min_dist;
}

Eigen::Vector2d TEB::contraction_force(int i) const
{
  // Elastic Bands 기본: 인접 포즈들을 가깝게
  if (i == 0 || i == static_cast<int>(poses_.size()) - 1) {
    return Eigen::Vector2d::Zero();
  }
  
  const auto& prev = poses_[i - 1].pos;
  const auto& next = poses_[i + 1].pos;
  const auto& current = poses_[i].pos;
  
  Eigen::Vector2d dir_prev = (prev - current);
  double norm_prev = dir_prev.norm();
  if (norm_prev > 1e-6) {
    dir_prev /= norm_prev;
  }
  
  Eigen::Vector2d dir_next = (next - current);
  double norm_next = dir_next.norm();
  if (norm_next > 1e-6) {
    dir_next /= norm_next;
  }
  
  return kc_ * (dir_prev + dir_next);
}

Eigen::Vector2d TEB::repulsive_force(int i) const
{
  // Elastic Bands 기본: 장애물로부터 멀어지게
  const auto& b = poses_[i].pos;
  double rho = poses_[i].radius;
  
  if (rho >= rho0_) {
    return Eigen::Vector2d::Zero();
  }
  
  // 그래디언트 근사 (finite difference)
  double h = step_size_;
  Eigen::Vector2d dx(h, 0);
  Eigen::Vector2d dy(0, h);
  
  double grad_x = (compute_rho(b - dx) - compute_rho(b + dx)) / (2.0 * h);
  double grad_y = (compute_rho(b - dy) - compute_rho(b + dy)) / (2.0 * h);
  Eigen::Vector2d grad(grad_x, grad_y);
  
  return kr_repulsive_ * (rho0_ - rho) * grad;
}

Eigen::Vector2d TEB::velocity_constraint_force(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return Eigen::Vector2d::Zero();
  }
  
  double v = compute_velocity(i);
  if (v > v_max_) {
    // 속도 초과 시 방향 반대로 힘 적용
    Eigen::Vector2d dir = (poses_[i + 1].pos - poses_[i].pos);
    double norm = dir.norm();
    if (norm > 1e-6) {
      dir /= norm;
    }
    return -kv_ * (v - v_max_) * dir;
  }
  
  return Eigen::Vector2d::Zero();
}

Eigen::Vector2d TEB::acceleration_constraint_force(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return Eigen::Vector2d::Zero();
  }
  
  double a = compute_acceleration(i);
  if (std::abs(a) > a_max_) {
    Eigen::Vector2d dir = (poses_[i + 1].pos - poses_[i].pos);
    double norm = dir.norm();
    if (norm > 1e-6) {
      dir /= norm;
    }
    double sign = (a > 0) ? 1.0 : -1.0;
    return -ka_ * sign * (std::abs(a) - a_max_) * dir;
  }
  
  return Eigen::Vector2d::Zero();
}

Eigen::Vector2d TEB::nonholonomic_constraint_force(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return Eigen::Vector2d::Zero();
  }
  
  double violation = compute_nonholonomic_violation(i);
  if (std::abs(violation) < 1e-6) {
    return Eigen::Vector2d::Zero();
  }
  
  // 제약 위반 방향으로 힘 적용
  Eigen::Vector2d dir = (poses_[i + 1].pos - poses_[i].pos);
  double norm = dir.norm();
  if (norm > 1e-6) {
    dir /= norm;
  }
  
  return -knh_ * violation * dir;
}

Eigen::Vector2d TEB::turning_radius_constraint_force(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return Eigen::Vector2d::Zero();
  }
  
  double rho = compute_turning_radius(i);
  if (rho < rho_min_) {
    // 최소 반경 미만 시 곡률을 줄이는 방향으로 힘
    Eigen::Vector2d dir = (poses_[i + 1].pos - poses_[i].pos);
    double norm = dir.norm();
    if (norm > 1e-6) {
      dir /= norm;
    }
    return -kr_radius_ * (rho_min_ - rho) * dir;
  }
  
  return Eigen::Vector2d::Zero();
}

double TEB::time_optimization_cost(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return 0.0;
  }
  
  // 시간 최소화 목표: ΔT^2
  return kt_ * poses_[i].delta_t * poses_[i].delta_t;
}

double TEB::compute_velocity(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return 0.0;
  }
  
  double dist = (poses_[i + 1].pos - poses_[i].pos).norm();
  return dist / (poses_[i].delta_t + 1e-6);
}

double TEB::compute_acceleration(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 2) {
    return 0.0;
  }
  
  double v1 = compute_velocity(i);
  double v2 = compute_velocity(i + 1);
  double dt = poses_[i].delta_t;
  
  return (v2 - v1) / (dt + 1e-6);
}

double TEB::compute_turning_radius(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return std::numeric_limits<double>::max();
  }
  
  // Equation (8): ρ_k = ||d_k||_2 / (2 * sin(Δβ_k / 2))
  Eigen::Vector2d d = poses_[i + 1].pos - poses_[i].pos;
  double delta_beta = poses_[i + 1].theta - poses_[i].theta;
  
  // 각도 정규화
  while (delta_beta > M_PI) delta_beta -= 2.0 * M_PI;
  while (delta_beta < -M_PI) delta_beta += 2.0 * M_PI;
  
  double d_norm = d.norm();
  if (d_norm < 1e-6) {
    return std::numeric_limits<double>::max();
  }
  
  double sin_half = std::sin(std::abs(delta_beta) / 2.0);
  if (sin_half < 1e-6) {
    // 직선에 가까우면 큰 반경 반환
    return std::numeric_limits<double>::max();
  }
  
  return d_norm / (2.0 * sin_half);
}

double TEB::compute_nonholonomic_violation(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return 0.0;
  }
  
  // Equation (7): 비홀로노믹 제약
  Eigen::Vector2d d = poses_[i + 1].pos - poses_[i].pos;
  Eigen::Vector2d dir_i(std::cos(poses_[i].theta), std::sin(poses_[i].theta));
  Eigen::Vector2d dir_i1(std::cos(poses_[i + 1].theta), std::sin(poses_[i + 1].theta));
  
  // 제약: (dir_i + dir_i1) · d = 0 (거의)
  Eigen::Vector2d sum_dir = dir_i + dir_i1;
  double violation = std::abs(sum_dir.dot(d));
  
  return violation;
}

double TEB::optimize_time_interval(int i) const
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return 0.1;
  }
  
  // 시간 최적화: 속도 제약을 만족하는 최소 시간
  double dist = (poses_[i + 1].pos - poses_[i].pos).norm();
  double optimal_dt = dist / v_max_;
  
  // 최소 시간 제한
  const double MIN_DT = 0.05;
  return std::max(MIN_DT, optimal_dt);
}

void TEB::update_orientation(int i)
{
  if (i >= static_cast<int>(poses_.size()) - 1) {
    return;
  }
  
  // 방향을 이동 방향에 맞춤
  Eigen::Vector2d dir = poses_[i + 1].pos - poses_[i].pos;
  double norm = dir.norm();
  if (norm > 1e-6) {
    poses_[i].theta = std::atan2(dir.y(), dir.x());
  }
}

void TEB::update_poses()
{
  std::vector<TEBPose> new_poses;
  
  for (size_t i = 0; i < poses_.size(); i++) {
    if (i == 0 || i == poses_.size() - 1) {
      // 시작/끝 포즈는 고정
      new_poses.push_back(poses_[i]);
      continue;
    }
    
    // 모든 힘 합산 (FLC-TEB: smoothness와 jerk objectives 포함)
    Eigen::Vector2d f_total = 
      contraction_force(i) +
      repulsive_force(i) +
      velocity_constraint_force(i) +
      acceleration_constraint_force(i) +
      nonholonomic_constraint_force(i) +
      turning_radius_constraint_force(i);
    
    // Smoothness objective 추가 (각도 변화 최소화)
    if (i > 0 && i < poses_.size() - 1 && ksm_ > 0.0) {
      double delta_beta_i = poses_[i].theta - poses_[i-1].theta;
      double delta_beta_i1 = poses_[i+1].theta - poses_[i].theta;
      
      // 각도 정규화
      while (delta_beta_i > M_PI) delta_beta_i -= 2.0 * M_PI;
      while (delta_beta_i < -M_PI) delta_beta_i += 2.0 * M_PI;
      while (delta_beta_i1 > M_PI) delta_beta_i1 -= 2.0 * M_PI;
      while (delta_beta_i1 < -M_PI) delta_beta_i1 += 2.0 * M_PI;
      
      // 각도 변화를 최소화하는 방향으로 힘 추가
      double smoothness_gradient = 2.0 * (delta_beta_i1 - delta_beta_i);
      Eigen::Vector2d dir = poses_[i+1].pos - poses_[i-1].pos;
      double dir_norm = dir.norm();
      if (dir_norm > 1e-6) {
        Eigen::Vector2d perp_dir(-dir.y() / dir_norm, dir.x() / dir_norm);
        f_total += ksm_ * smoothness_gradient * perp_dir;
      }
    }
    
    // 접선 방향 제거 (Elastic Bands 방식)
    if (i > 0 && i < poses_.size() - 1) {
      Eigen::Vector2d v = poses_[i - 1].pos - poses_[i + 1].pos;
      double v_norm = v.norm();
      if (v_norm > 1e-6) {
        Eigen::Vector2d v_unit = v / v_norm;
        f_total = f_total - f_total.dot(v_unit) * v_unit;
      }
    }
    
    // 적응형 스텝 크기
    double alpha = poses_[i].radius;
    Eigen::Vector2d new_pos = poses_[i].pos + alpha * f_total;
    
    // 위치 제한 (필요시)
    // new_pos = new_pos.cwiseMax(Eigen::Vector2d(0, 0));
    
    // 새로운 반경 계산
    double new_radius = compute_rho(new_pos);
    
    // 시간 간격 최적화
    double new_dt = optimize_time_interval(i);
    
    // 방향 업데이트
    TEBPose new_pose(new_pos, poses_[i].theta, new_dt, new_radius);
    update_orientation(i);
    new_pose.theta = poses_[i].theta;
    
    new_poses.push_back(new_pose);
  }
  
  poses_ = new_poses;
  maintain_overlap();
}

double TEB::smoothness_objective() const
{
  // Equation (10): f_sm = Σ(Δβ_{i+1} - Δβ_i)²
  double cost = 0.0;
  
  for (size_t i = 1; i < poses_.size() - 1; i++) {
    // Δβ_i 계산
    double delta_beta_i = poses_[i].theta - poses_[i-1].theta;
    double delta_beta_i1 = poses_[i+1].theta - poses_[i].theta;
    
    // 각도 정규화
    while (delta_beta_i > M_PI) delta_beta_i -= 2.0 * M_PI;
    while (delta_beta_i < -M_PI) delta_beta_i += 2.0 * M_PI;
    while (delta_beta_i1 > M_PI) delta_beta_i1 -= 2.0 * M_PI;
    while (delta_beta_i1 < -M_PI) delta_beta_i1 += 2.0 * M_PI;
    
    double diff = delta_beta_i1 - delta_beta_i;
    cost += diff * diff;
  }
  
  return cost;
}

double TEB::jerk_objective() const
{
  // Equation (11): f_jerk = Σ |(a_{i+1} - a_i) / (ΔT_i/4 + ΔT_{i+1}/2 + ΔT_{i+2}/4)|
  double cost = 0.0;
  
  for (size_t i = 0; i < poses_.size() - 3; i++) {
    double a_i = compute_acceleration(i);
    double a_i1 = compute_acceleration(i + 1);
    
    double dt_sum = poses_[i].delta_t / 4.0 + 
                    poses_[i+1].delta_t / 2.0 + 
                    poses_[i+2].delta_t / 4.0;
    
    if (dt_sum > 1e-6) {
      double jerk = std::abs((a_i1 - a_i) / dt_sum);
      cost += jerk;
    }
  }
  
  return cost;
}

double TEB::compute_narrowness() const
{
  // Equation (13): N_t = d_obs / ρ_min
  if (poses_.empty() || rho_min_ < 1e-6) {
    return 0.0;
  }

  // 평균 거리 계산
  double sum_d_obs = 0.0;
  for (const auto& pose : poses_) {
    sum_d_obs += pose.radius;  // radius는 이미 장애물까지의 거리
  }
  double d_obs = sum_d_obs / poses_.size();

  double n_t = d_obs / rho_min_;
  return std::max(0.0, std::min(4.0, n_t));  // [0, 4] 범위로 제한
}

double TEB::compute_turning_complexity() const
{
  // Equation (14): T_c = (n_turn + 2*n_sharp) / (n-1)
  if (poses_.size() < 2) {
    return 0.0;
  }

  double kappa_max = 1.0 / rho_min_;
  double kappa_turn = 0.2 * kappa_max;
  double kappa_sharp = 0.8 * kappa_max;

  int n_turn = 0;
  int n_sharp = 0;

  for (size_t i = 0; i < poses_.size() - 1; i++) {
    // 곡률 계산: κ_i = Δθ_i / Δs_i
    double delta_theta = poses_[i + 1].theta - poses_[i].theta;
    // 각도 정규화
    while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
    while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;

    double delta_s = (poses_[i + 1].pos - poses_[i].pos).norm();
    if (delta_s < 1e-6) {
      continue;
    }

    double kappa = std::abs(delta_theta / delta_s);

    if (kappa > kappa_sharp) {
      n_sharp++;
    } else if (kappa > kappa_turn) {
      n_turn++;
    }
  }

  double t_c = static_cast<double>(n_turn + 2 * n_sharp) / (poses_.size() - 1);
  return std::max(0.0, std::min(2.0, t_c));  // [0, 2] 범위로 제한
}

void TEB::maintain_overlap()
{
  // Bubble 겹침 제약 유지 (Elastic Bands)
  // Insert bubbles
  size_t i = 0;
  while (i < poses_.size() - 1) {
    const auto& bi = poses_[i];
    const auto& bj = poses_[i + 1];
    
    double dist = (bi.pos - bj.pos).norm();
    if (dist > lambda_ * (bi.radius + bj.radius)) {
      // 너무 멀면 중간에 포즈 추가
      Eigen::Vector2d new_pos = (bi.pos + bj.pos) / 2.0;
      double rho = compute_rho(new_pos);
      double theta = std::atan2(bj.pos.y() - bi.pos.y(), bj.pos.x() - bi.pos.x());
      double dt = (bi.delta_t + bj.delta_t) / 2.0;
      
      poses_.insert(poses_.begin() + i + 1, TEBPose(new_pos, theta, dt, rho));
      i += 2;
    } else {
      i++;
    }
  }
  
  // Delete redundant bubbles (but keep minimum number of poses for valid path)
  i = 1;
  const size_t min_total_poses = 20;  // 최소 총 포즈 수 (시작/끝 포함)
  while (i < poses_.size() - 1 && poses_.size() > min_total_poses) {
    const auto& prev = poses_[i - 1];
    const auto& next = poses_[i + 1];
    
    double dist = (prev.pos - next.pos).norm();
    double threshold = lambda_ * (prev.radius + next.radius);
    
    // 삭제 조건을 더 보수적으로: 매우 가까울 때만 삭제
    if (dist <= threshold * 0.5) {  // 원래 threshold의 50% 이하일 때만 삭제
      // 너무 가까우면 중간 포즈 삭제 (하지만 최소 포즈 수는 유지)
      poses_.erase(poses_.begin() + i);
      // 삭제 후 다시 체크하므로 i는 증가시키지 않음
    } else {
      i++;
    }
  }
}

void TEB::set_weights(double k_obs, double k_sm, double k_v, double k_acc, double k_jerk)
{
  // FLC에서 계산된 가중치를 TEB 파라미터에 적용
  // Note: 현재 구현에서는 일부 가중치만 사용
  // TODO: 모든 가중치를 objective function에 통합
  ksm_ = k_sm;
  kjerk_ = k_jerk;
  kv_ = k_v;
  ka_ = k_acc;
  
  // 장애물 거리 가중치는 repulsive_force에 반영
  // kr_repulsive_를 조정하여 장애물 회피 강도 조절
  if (k_obs > 0.0) {
    kr_repulsive_ = -0.1 * (k_obs / 100.0);  // 스케일링
  }
}

void TEB::optimize(int max_iter)
{
  for (int iter = 0; iter < max_iter; iter++) {
    size_t poses_before = poses_.size();
    update_poses();
    
    // 최소 포즈 수 보장 (시작/끝 포함 최소 3개, 이상적으로는 10개 이상)
    if (poses_.size() < 3) {
      // 포즈가 너무 적으면 최적화 중단
      break;
    }
  }
}

std::vector<std::pair<Eigen::Vector2d, double>> TEB::get_path_with_velocities() const
{
  std::vector<std::pair<Eigen::Vector2d, double>> path;
  
  for (size_t i = 0; i < poses_.size(); i++) {
    double v = 0.0;
    if (i < poses_.size() - 1) {
      v = compute_velocity(i);
      v = std::min(v, v_max_);  // 속도 제한
    }
    path.emplace_back(poses_[i].pos, v);
  }
  
  return path;
}

}  // namespace teb_local_planner

