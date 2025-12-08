#pragma once

#include <vector>
#include <Eigen/Dense>
#include <memory>

namespace teb_local_planner
{

/**
 * @brief TEB Pose - 위치, 방향, 시간 간격을 포함하는 포즈
 * Elastic Bands의 Bubble을 확장한 개념
 */
struct TEBPose
{
  Eigen::Vector2d pos;      // 위치 [x, y]
  double theta;              // 방향 β (rad)
  double delta_t;            // 시간 간격 ΔT (sec)
  double radius;             // 안전 반경 (장애물까지 거리)
  
  TEBPose()
    : pos(0.0, 0.0), theta(0.0), delta_t(0.1), radius(10.0) {}
  
  TEBPose(const Eigen::Vector2d& p, double t, double dt, double r)
    : pos(p), theta(t), delta_t(dt), radius(r) {}
};

/**
 * @brief Timed Elastic Band (TEB) 클래스
 * Elastic Bands 알고리즘을 기반으로 시간 최적화 및 차량 제약 추가
 */
class TEB
{
public:
  /**
   * @brief 생성자
   * @param initial_path 초기 경로 (2D 위치들)
   * @param obstacles 장애물 위치들
   * @param v_max 최대 속도
   * @param a_max 최대 가속도
   * @param rho_min 최소 회전 반경
   * @param wheelbase 차량 휠베이스
   */
  TEB(
    const std::vector<Eigen::Vector2d>& initial_path,
    const std::vector<Eigen::Vector2d>& obstacles,
    double v_max = 5.0,
    double a_max = 2.0,
    double rho_min = 1.0,
    double wheelbase = 0.33);

  /**
   * @brief 최적화 반복 수행
   * @param max_iter 최대 반복 횟수
   */
  void optimize(int max_iter = 50);

  /**
   * @brief FLC에서 계산된 가중치 설정
   */
  void set_weights(double k_obs, double k_sm, double k_v, double k_acc, double k_jerk);

  /**
   * @brief Narrowness 계산 (Equation 13)
   * N_t = d_obs / ρ_min
   */
  double compute_narrowness() const;

  /**
   * @brief Turning Complexity 계산 (Equation 14)
   * T_c = (n_turn + 2*n_sharp) / (n-1)
   */
  double compute_turning_complexity() const;

  /**
   * @brief 최적화된 포즈들 반환
   */
  const std::vector<TEBPose>& get_poses() const { return poses_; }

  /**
   * @brief 경로를 waypoint 배열로 변환
   */
  std::vector<std::pair<Eigen::Vector2d, double>> get_path_with_velocities() const;

private:
  // 포즈 시퀀스
  std::vector<TEBPose> poses_;
  
  // 장애물 정보
  std::vector<Eigen::Vector2d> obstacles_;
  
  // 파라미터들
  double v_max_;           // 최대 속도
  double a_max_;           // 최대 가속도
  double rho_min_;         // 최소 회전 반경
  double wheelbase_;       // 차량 휠베이스
  
  // Elastic Bands 파라미터들
  double rho0_{20.0};      // 최대 반발력 적용 거리
  double kc_{0.05};        // 수축력 게인
  double kr_repulsive_{-0.1};  // 반발력 게인
  double lambda_{0.7};     // 겹침 제약 인자
  double step_size_{3.0};  // 그래디언트 계산 스텝 크기
  
  // TEB 추가 파라미터들
  double kv_{1.0};         // 속도 제약 가중치
  double ka_{1.0};         // 가속도 제약 가중치
  double knh_{1.0};       // 비홀로노믹 제약 가중치
  double kr_radius_{1.0};  // 회전 반경 제약 가중치
  double kt_{1.0};         // 시간 최적화 가중치
  double ksm_{1.0};        // Smoothness 가중치 (FLC-TEB)
  double kjerk_{1.0};      // Jerk 가중치 (FLC-TEB)

  /**
   * @brief 위치에서 장애물까지의 거리 계산 (SDF)
   */
  double compute_rho(const Eigen::Vector2d& position) const;

  /**
   * @brief 수축력 계산 (Elastic Bands 기본)
   * 인접한 포즈들을 가깝게 만드는 힘
   */
  Eigen::Vector2d contraction_force(int i) const;

  /**
   * @brief 반발력 계산 (Elastic Bands 기본)
   * 장애물로부터 멀어지게 하는 힘
   */
  Eigen::Vector2d repulsive_force(int i) const;

  /**
   * @brief 속도 제약 힘 계산 (TEB 추가)
   */
  Eigen::Vector2d velocity_constraint_force(int i) const;

  /**
   * @brief 가속도 제약 힘 계산 (TEB 추가)
   */
  Eigen::Vector2d acceleration_constraint_force(int i) const;

  /**
   * @brief 비홀로노믹 제약 힘 계산 (TEB 추가)
   * 차량이 같은 곡률의 호 위에서 움직여야 함
   */
  Eigen::Vector2d nonholonomic_constraint_force(int i) const;

  /**
   * @brief 최소 회전 반경 제약 힘 계산 (TEB 추가)
   */
  Eigen::Vector2d turning_radius_constraint_force(int i) const;

  /**
   * @brief 시간 최적화 비용 계산
   */
  double time_optimization_cost(int i) const;

  /**
   * @brief Smoothness objective 계산 (Equation 10)
   * f_sm = Σ(Δβ_{i+1} - Δβ_i)²
   */
  double smoothness_objective() const;

  /**
   * @brief Jerk objective 계산 (Equation 11)
   * f_jerk = Σ |(a_{i+1} - a_i) / (ΔT_i/4 + ΔT_{i+1}/2 + ΔT_{i+2}/4)|
   */
  double jerk_objective() const;

  /**
   * @brief 포즈 업데이트
   */
  void update_poses();

  /**
   * @brief 방향 업데이트 (비홀로노믹 제약 고려)
   */
  void update_orientation(int i);

  /**
   * @brief 시간 간격 최적화
   */
  double optimize_time_interval(int i) const;

  /**
   * @brief 속도 계산
   */
  double compute_velocity(int i) const;

  /**
   * @brief 가속도 계산
   */
  double compute_acceleration(int i) const;

  /**
   * @brief 회전 반경 계산 (Equation 8)
   */
  double compute_turning_radius(int i) const;

  /**
   * @brief 비홀로노믹 제약 위반 계산
   */
  double compute_nonholonomic_violation(int i) const;

  /**
   * @brief 겹침 제약 유지 (Elastic Bands)
   */
  void maintain_overlap();
};

}  // namespace teb_local_planner

