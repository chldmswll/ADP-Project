#pragma once

#include <vector>
#include <string>

namespace teb_local_planner
{

/**
 * @brief Fuzzy Logic Controller for FLC-TEB
 * 논문의 Fuzzy Logic Controller 구현
 */
class FuzzyController
{
public:
  /**
   * @brief 생성자
   */
  FuzzyController();

  /**
   * @brief Narrowness와 Turning Complexity를 입력으로 받아 가중치 계산
   * @param narrowness 경로의 좁음 정도 [0, 4]
   * @param turning_complexity 회전 복잡도 [0, 2]
   * @return 계산된 가중치들
   */
  struct Weights {
    double sigma_obs;      // 장애물 거리 가중치 [0, 1000]
    double sigma_sm;       // Smoothness 가중치 [0, 100]
    double sigma_v;        // 속도 가중치 [0, 100]
    double sigma_acc;      // 가속도 가중치 [0, 100]
    double sigma_j;        // Jerk 가중치 [0, 100]
    double sigma_sp;       // 최단 경로 가중치 [0, 5]
    double sigma_to;        // 최적 시간 가중치 [0, 5]
  };

  Weights compute_weights(double narrowness, double turning_complexity);

private:
  /**
   * @brief Fuzzy membership function types
   */
  enum class FuzzySet {
    S,   // Small
    M,   // Middle
    L,   // Large
    VL,  // Very Low
    H,   // High
    VH   // Very High
  };

  /**
   * @brief Narrowness fuzzification
   * Domain: [0, 4], Fuzzy sets: [S, M, L]
   */
  std::vector<double> fuzzify_narrowness(double n_t) const;

  /**
   * @brief Turning complexity fuzzification
   * Domain: [0, 2], Fuzzy sets: [S, M, L]
   */
  std::vector<double> fuzzify_turning_complexity(double t_c) const;

  /**
   * @brief Fuzzy inference using rule table
   */
  std::vector<double> fuzzy_inference(
    const std::vector<double>& narrowness_membership,
    const std::vector<double>& turning_complexity_membership) const;

  /**
   * @brief Defuzzification using center of gravity
   */
  Weights defuzzify(const std::vector<double>& output_membership) const;

  /**
   * @brief Weight mapping (Equation 15)
   * σ' ∈ [-1, 0) → σ = 1 - σ'
   * σ' ∈ [0, 3]  → σ = 10^σ'
   */
  double map_weight(double sigma_prime, double min_val, double max_val) const;

  /**
   * @brief Triangular membership function
   */
  double triangular_membership(double x, double a, double b, double c) const;

  /**
   * @brief Trapezoidal membership function
   */
  double trapezoidal_membership(double x, double a, double b, double c, double d) const;
};

}  // namespace teb_local_planner

