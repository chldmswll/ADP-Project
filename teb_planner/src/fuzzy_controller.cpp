#include "teb_planner/fuzzy_controller.hpp"
#include <algorithm>
#include <cmath>
#include <map>

namespace teb_local_planner
{

FuzzyController::FuzzyController()
{
}

FuzzyController::Weights FuzzyController::compute_weights(
  double narrowness, double turning_complexity)
{
  // Step 1: Fuzzification
  auto narrowness_membership = fuzzify_narrowness(narrowness);
  auto turning_complexity_membership = fuzzify_turning_complexity(turning_complexity);

  // Step 2: Fuzzy inference
  auto output_membership = fuzzy_inference(narrowness_membership, turning_complexity_membership);

  // Step 3: Defuzzification
  return defuzzify(output_membership);
}

std::vector<double> FuzzyController::fuzzify_narrowness(double n_t) const
{
  // Domain: [0, 4], Fuzzy sets: [S, M, L]
  // Trapezoidal at boundaries, Triangular in middle
  std::vector<double> membership(3, 0.0);  // [S, M, L]

  // Small (S): [0, 2.0] with trapezoidal
  if (n_t <= 0.0) {
    membership[0] = 1.0;
  } else if (n_t < 1.0) {
    membership[0] = 1.0 - n_t / 1.0;  // Linear decrease
  } else if (n_t <= 2.0) {
    membership[0] = 0.0;
  } else {
    membership[0] = 0.0;
  }

  // Middle (M): [1.0, 3.0] triangular
  if (n_t >= 1.0 && n_t <= 2.0) {
    membership[1] = (n_t - 1.0) / 1.0;  // Increase
  } else if (n_t > 2.0 && n_t <= 3.0) {
    membership[1] = (3.0 - n_t) / 1.0;  // Decrease
  } else {
    membership[1] = 0.0;
  }

  // Large (L): [2.0, 4.0] with trapezoidal
  if (n_t <= 2.0) {
    membership[2] = 0.0;
  } else if (n_t < 3.0) {
    membership[2] = (n_t - 2.0) / 1.0;  // Linear increase
  } else {
    membership[2] = 1.0;
  }

  return membership;
}

std::vector<double> FuzzyController::fuzzify_turning_complexity(double t_c) const
{
  // Domain: [0, 2], Fuzzy sets: [S, M, L]
  std::vector<double> membership(3, 0.0);  // [S, M, L]

  // Small (S): [0, 1.0] with trapezoidal
  if (t_c <= 0.0) {
    membership[0] = 1.0;
  } else if (t_c < 0.5) {
    membership[0] = 1.0 - t_c / 0.5;  // Linear decrease
  } else if (t_c <= 1.0) {
    membership[0] = 0.0;
  } else {
    membership[0] = 0.0;
  }

  // Middle (M): [0.5, 1.5] triangular
  if (t_c >= 0.5 && t_c <= 1.0) {
    membership[1] = (t_c - 0.5) / 0.5;  // Increase
  } else if (t_c > 1.0 && t_c <= 1.5) {
    membership[1] = (1.5 - t_c) / 0.5;  // Decrease
  } else {
    membership[1] = 0.0;
  }

  // Large (L): [1.0, 2.0] with trapezoidal
  if (t_c <= 1.0) {
    membership[2] = 0.0;
  } else if (t_c < 1.5) {
    membership[2] = (t_c - 1.0) / 0.5;  // Linear increase
  } else {
    membership[2] = 1.0;
  }

  return membership;
}

std::vector<double> FuzzyController::fuzzy_inference(
  const std::vector<double>& narrowness_membership,
  const std::vector<double>& turning_complexity_membership) const
{
  // Table 3: Fuzzy rules
  // Output: [σ_obs, σ_sm, σ_v_linear, σ_v_angular, σ_acc_linear, σ_acc_angular,
  //          σ_j_linear, σ_j_angular, σ_sp, σ_to]
  // 각 출력에 대해 5개 레벨: [VL, L, M, H, VH]
  std::vector<double> output_membership(10 * 5, 0.0);  // 10 outputs × 5 levels

  // Rule 1: N_t=S, T_c=S → σ_obs=L, σ_sm=L, σ_v_linear=VL, σ_v_angular=VH, σ_sp=L, σ_to=L
  double rule1_strength = std::min(narrowness_membership[0], turning_complexity_membership[0]);
  if (rule1_strength > 0.0) {
    // σ_obs = L (index 1)
    output_membership[0 * 5 + 1] = std::max(output_membership[0 * 5 + 1], rule1_strength);
    // σ_sm = L (index 1)
    output_membership[1 * 5 + 1] = std::max(output_membership[1 * 5 + 1], rule1_strength);
    // σ_v_linear = VL (index 0)
    output_membership[2 * 5 + 0] = std::max(output_membership[2 * 5 + 0], rule1_strength);
    // σ_v_angular = VH (index 4)
    output_membership[3 * 5 + 4] = std::max(output_membership[3 * 5 + 4], rule1_strength);
    // σ_sp = L (index 1)
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule1_strength);
    // σ_to = L (index 1)
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule1_strength);
  }

  // Rule 2: N_t=S, T_c=M → σ_obs=L, σ_sm=L, σ_v_linear=VL, σ_v_angular=L, σ_sp=L, σ_to=L
  double rule2_strength = std::min(narrowness_membership[0], turning_complexity_membership[1]);
  if (rule2_strength > 0.0) {
    output_membership[0 * 5 + 1] = std::max(output_membership[0 * 5 + 1], rule2_strength);
    output_membership[1 * 5 + 1] = std::max(output_membership[1 * 5 + 1], rule2_strength);
    output_membership[2 * 5 + 0] = std::max(output_membership[2 * 5 + 0], rule2_strength);
    output_membership[3 * 5 + 1] = std::max(output_membership[3 * 5 + 1], rule2_strength);
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule2_strength);
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule2_strength);
  }

  // Rule 3: N_t=S, T_c=L → σ_obs=L, σ_sm=M, σ_v_linear=VL, σ_v_angular=L, σ_sp=L, σ_to=L
  double rule3_strength = std::min(narrowness_membership[0], turning_complexity_membership[2]);
  if (rule3_strength > 0.0) {
    output_membership[0 * 5 + 1] = std::max(output_membership[0 * 5 + 1], rule3_strength);
    output_membership[1 * 5 + 2] = std::max(output_membership[1 * 5 + 2], rule3_strength);
    output_membership[2 * 5 + 0] = std::max(output_membership[2 * 5 + 0], rule3_strength);
    output_membership[3 * 5 + 1] = std::max(output_membership[3 * 5 + 1], rule3_strength);
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule3_strength);
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule3_strength);
  }

  // Rule 4: N_t=M, T_c=S → σ_obs=L, σ_sm=VL, σ_v_linear=H, σ_v_angular=L, σ_sp=M, σ_to=M
  double rule4_strength = std::min(narrowness_membership[1], turning_complexity_membership[0]);
  if (rule4_strength > 0.0) {
    output_membership[0 * 5 + 1] = std::max(output_membership[0 * 5 + 1], rule4_strength);
    output_membership[1 * 5 + 0] = std::max(output_membership[1 * 5 + 0], rule4_strength);
    output_membership[2 * 5 + 3] = std::max(output_membership[2 * 5 + 3], rule4_strength);
    output_membership[3 * 5 + 1] = std::max(output_membership[3 * 5 + 1], rule4_strength);
    output_membership[8 * 5 + 2] = std::max(output_membership[8 * 5 + 2], rule4_strength);
    output_membership[9 * 5 + 2] = std::max(output_membership[9 * 5 + 2], rule4_strength);
  }

  // Rule 5: N_t=M, T_c=M → σ_obs=L, σ_sm=M, σ_v_linear=L, σ_v_angular=M, σ_sp=L, σ_to=L
  double rule5_strength = std::min(narrowness_membership[1], turning_complexity_membership[1]);
  if (rule5_strength > 0.0) {
    output_membership[0 * 5 + 1] = std::max(output_membership[0 * 5 + 1], rule5_strength);
    output_membership[1 * 5 + 2] = std::max(output_membership[1 * 5 + 2], rule5_strength);
    output_membership[2 * 5 + 1] = std::max(output_membership[2 * 5 + 1], rule5_strength);
    output_membership[3 * 5 + 2] = std::max(output_membership[3 * 5 + 2], rule5_strength);
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule5_strength);
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule5_strength);
  }

  // Rule 6: N_t=M, T_c=L → σ_obs=L, σ_sm=M, σ_v_linear=L, σ_v_angular=M, σ_sp=L, σ_to=L
  double rule6_strength = std::min(narrowness_membership[1], turning_complexity_membership[2]);
  if (rule6_strength > 0.0) {
    output_membership[0 * 5 + 1] = std::max(output_membership[0 * 5 + 1], rule6_strength);
    output_membership[1 * 5 + 2] = std::max(output_membership[1 * 5 + 2], rule6_strength);
    output_membership[2 * 5 + 1] = std::max(output_membership[2 * 5 + 1], rule6_strength);
    output_membership[3 * 5 + 2] = std::max(output_membership[3 * 5 + 2], rule6_strength);
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule6_strength);
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule6_strength);
  }

  // Rule 7: N_t=L, T_c=S → σ_obs=H, σ_sm=L, σ_v_linear=L, σ_v_angular=L, σ_sp=L, σ_to=L
  double rule7_strength = std::min(narrowness_membership[2], turning_complexity_membership[0]);
  if (rule7_strength > 0.0) {
    output_membership[0 * 5 + 3] = std::max(output_membership[0 * 5 + 3], rule7_strength);
    output_membership[1 * 5 + 1] = std::max(output_membership[1 * 5 + 1], rule7_strength);
    output_membership[2 * 5 + 1] = std::max(output_membership[2 * 5 + 1], rule7_strength);
    output_membership[3 * 5 + 1] = std::max(output_membership[3 * 5 + 1], rule7_strength);
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule7_strength);
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule7_strength);
  }

  // Rule 8: N_t=L, T_c=M → σ_obs=M, σ_sm=M, σ_v_linear=VL, σ_v_angular=L, σ_sp=L, σ_to=L
  double rule8_strength = std::min(narrowness_membership[2], turning_complexity_membership[1]);
  if (rule8_strength > 0.0) {
    output_membership[0 * 5 + 2] = std::max(output_membership[0 * 5 + 2], rule8_strength);
    output_membership[1 * 5 + 2] = std::max(output_membership[1 * 5 + 2], rule8_strength);
    output_membership[2 * 5 + 0] = std::max(output_membership[2 * 5 + 0], rule8_strength);
    output_membership[3 * 5 + 1] = std::max(output_membership[3 * 5 + 1], rule8_strength);
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule8_strength);
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule8_strength);
  }

  // Rule 9: N_t=L, T_c=L → σ_obs=H, σ_sm=M, σ_v_linear=VL, σ_v_angular=L, σ_sp=L, σ_to=L
  double rule9_strength = std::min(narrowness_membership[2], turning_complexity_membership[2]);
  if (rule9_strength > 0.0) {
    output_membership[0 * 5 + 3] = std::max(output_membership[0 * 5 + 3], rule9_strength);
    output_membership[1 * 5 + 2] = std::max(output_membership[1 * 5 + 2], rule9_strength);
    output_membership[2 * 5 + 0] = std::max(output_membership[2 * 5 + 0], rule9_strength);
    output_membership[3 * 5 + 1] = std::max(output_membership[3 * 5 + 1], rule9_strength);
    output_membership[8 * 5 + 1] = std::max(output_membership[8 * 5 + 1], rule9_strength);
    output_membership[9 * 5 + 1] = std::max(output_membership[9 * 5 + 1], rule9_strength);
  }

  return output_membership;
}

FuzzyController::Weights FuzzyController::defuzzify(
  const std::vector<double>& output_membership) const
{
  Weights weights;

  // Defuzzification using center of gravity
  // 각 출력에 대해 5개 레벨의 중심값: VL=-1, L=0, M=1, H=2, VH=3
  std::vector<double> level_centers = {-1.0, 0.0, 1.0, 2.0, 3.0};

  // σ_obs: [-1, 3] → [0, 1000]
  double sigma_obs_prime = 0.0;
  double sum_obs = 0.0;
  for (int i = 0; i < 5; i++) {
    sigma_obs_prime += output_membership[0 * 5 + i] * level_centers[i];
    sum_obs += output_membership[0 * 5 + i];
  }
  if (sum_obs > 1e-6) {
    sigma_obs_prime /= sum_obs;
  }
  weights.sigma_obs = map_weight(sigma_obs_prime, 0.0, 1000.0);

  // σ_sm, σ_v, σ_acc, σ_j: [-1, 2] → [0, 100]
  // σ_sm
  double sigma_sm_prime = 0.0;
  double sum_sm = 0.0;
  for (int i = 0; i < 5; i++) {
    sigma_sm_prime += output_membership[1 * 5 + i] * level_centers[i];
    sum_sm += output_membership[1 * 5 + i];
  }
  if (sum_sm > 1e-6) {
    sigma_sm_prime /= sum_sm;
  }
  weights.sigma_sm = map_weight(sigma_sm_prime, 0.0, 100.0);

  // σ_v_linear (same as σ_acc_linear, σ_j_linear)
  double sigma_v_linear_prime = 0.0;
  double sum_v_linear = 0.0;
  for (int i = 0; i < 5; i++) {
    sigma_v_linear_prime += output_membership[2 * 5 + i] * level_centers[i];
    sum_v_linear += output_membership[2 * 5 + i];
  }
  if (sum_v_linear > 1e-6) {
    sigma_v_linear_prime /= sum_v_linear;
  }
  weights.sigma_v = map_weight(sigma_v_linear_prime, 0.0, 100.0);
  weights.sigma_acc = weights.sigma_v;  // Same mapping
  weights.sigma_j = weights.sigma_v;     // Same mapping

  // σ_sp, σ_to: [0, 5] (not mapped, use directly)
  double sigma_sp_prime = 0.0;
  double sum_sp = 0.0;
  for (int i = 0; i < 5; i++) {
    sigma_sp_prime += output_membership[8 * 5 + i] * level_centers[i];
    sum_sp += output_membership[8 * 5 + i];
  }
  if (sum_sp > 1e-6) {
    sigma_sp_prime /= sum_sp;
  }
  weights.sigma_sp = std::max(0.0, std::min(5.0, sigma_sp_prime + 1.0));  // Shift to [0, 5]

  double sigma_to_prime = 0.0;
  double sum_to = 0.0;
  for (int i = 0; i < 5; i++) {
    sigma_to_prime += output_membership[9 * 5 + i] * level_centers[i];
    sum_to += output_membership[9 * 5 + i];
  }
  if (sum_to > 1e-6) {
    sigma_to_prime /= sum_to;
  }
  weights.sigma_to = std::max(0.0, std::min(5.0, sigma_to_prime + 1.0));  // Shift to [0, 5]

  return weights;
}

double FuzzyController::map_weight(double sigma_prime, double min_val, double max_val) const
{
  // Equation (15):
  // σ' ∈ [-1, 0) → σ = 1 - σ'
  // σ' ∈ [0, 3]  → σ = 10^σ'
  double sigma;
  if (sigma_prime < 0.0) {
    sigma = 1.0 - sigma_prime;
  } else {
    sigma = std::pow(10.0, sigma_prime);
  }

  // Scale to desired range
  // For σ_obs: [0, 1000], for others: [0, 100]
  double range = max_val - min_val;
  sigma = std::max(min_val, std::min(max_val, sigma * range / 10.0));

  return sigma;
}

double FuzzyController::triangular_membership(double x, double a, double b, double c) const
{
  if (x <= a || x >= c) {
    return 0.0;
  } else if (x < b) {
    return (x - a) / (b - a);
  } else {
    return (c - x) / (c - b);
  }
}

double FuzzyController::trapezoidal_membership(
  double x, double a, double b, double c, double d) const
{
  if (x <= a || x >= d) {
    return 0.0;
  } else if (x >= b && x <= c) {
    return 1.0;
  } else if (x < b) {
    return (x - a) / (b - a);
  } else {
    return (d - x) / (d - c);
  }
}

}  // namespace teb_local_planner

