#ifndef GLOBAL_PLANNER__TRAJECTORY_OPTIMIZER_WRAPPER_HPP_
#define GLOBAL_PLANNER__TRAJECTORY_OPTIMIZER_WRAPPER_HPP_

#include <vector>
#include <string>
#include <Eigen/Dense>

namespace global_planner
{

struct TrajectoryOptimizerResult
{
  std::vector<Eigen::VectorXd> trajectory;  // [s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2]
  std::vector<Eigen::Vector2d> bound_r;
  std::vector<Eigen::Vector2d> bound_l;
  double est_lap_time;
};

/**
 * Call Python trajectory_optimizer function
 * @param input_path Path to input directory
 * @param track_name Name of the track (CSV file name without extension)
 * @param curv_opt_type Optimization type: "shortest_path", "mincurv", "mincurv_iqp", "mintime"
 * @param safety_width Safety width for trajectory
 * @param plot Whether to plot results
 * @return TrajectoryOptimizerResult containing trajectory, bounds, and lap time
 */
TrajectoryOptimizerResult call_trajectory_optimizer(
  const std::string & input_path,
  const std::string & track_name,
  const std::string & curv_opt_type,
  double safety_width,
  bool plot = false);

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__TRAJECTORY_OPTIMIZER_WRAPPER_HPP_
