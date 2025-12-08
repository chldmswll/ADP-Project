#include "global_planner/trajectory_optimizer_wrapper.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <filesystem>
#include <cstdlib>
#include <cstdio>
#include <memory>
#include <array>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace global_planner
{

TrajectoryOptimizerResult call_trajectory_optimizer(
  const std::string & input_path,
  const std::string & track_name,
  const std::string & curv_opt_type,
  double safety_width,
  bool plot)
{
  TrajectoryOptimizerResult result;

  // Create a temporary Python script to call trajectory_optimizer
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::filesystem::path script_path = temp_dir / "trajectory_optimizer_call.py";
  std::filesystem::path result_path = temp_dir / "trajectory_optimizer_result.json";

  // Create Python script
  std::ofstream script_file(script_path);
  if (!script_file.is_open()) {
    throw std::runtime_error("Failed to create temporary Python script");
  }

  script_file << R"(
import sys
import os
import json
import numpy as np

# Add path to global_racetrajectory_optimization module
# Try installed location first (after colcon build)
try:
    from ament_index_python.packages import get_package_share_directory
    package_share = get_package_share_directory('global_planner')
    python_modules_path = os.path.join(package_share, 'python_modules')
    if os.path.exists(python_modules_path):
        sys.path.insert(0, python_modules_path)
except:
    pass

# Try source location - check multiple possible paths
possible_paths = [
    # Current workspace (planner directory)
    os.path.join(os.path.expanduser('~'), 'shared_dir', 'planner', 'global_planner', 'python_modules'),
    # Race stack workspace paths
    '/home/misys/forza_ws/race_stack/stack_master/src/race_stack/stack_master/planner/global_planner/python_modules',
    '/home/misys/forza_ws/race_stack/src/race_stack/stack_master/planner/global_planner/python_modules',
    '/home/misys/forza_ws/race_stack/install/global_planner/share/global_planner/python_modules',
    # Try relative to script location
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'planner', 'global_planner', 'python_modules'),
    # Try from current working directory
    os.path.join(os.getcwd(), 'global_planner', 'python_modules'),
    os.path.join(os.getcwd(), '..', 'global_planner', 'python_modules'),
    os.path.join(os.getcwd(), '..', '..', 'global_planner', 'python_modules'),
]

# Also check if PYTHONPATH is set
pythonpath = os.environ.get('PYTHONPATH', '')
if pythonpath:
    for path in pythonpath.split(os.pathsep):
        if path and os.path.exists(path):
            possible_paths.append(path)
            # Also check subdirectories
            possible_paths.append(os.path.join(path, 'global_planner', 'python_modules'))
            possible_paths.append(os.path.join(path, 'python_modules'))

module_found = False
for path in possible_paths:
    abs_path = os.path.abspath(path)
    if os.path.exists(abs_path):
        # Check if global_racetrajectory_optimization exists directly or as subdirectory
        module_path = abs_path
        if os.path.exists(os.path.join(abs_path, 'global_racetrajectory_optimization')):
            module_path = abs_path
        elif os.path.basename(abs_path) == 'global_racetrajectory_optimization':
            module_path = os.path.dirname(abs_path)
        else:
            continue
            
        if module_path not in sys.path:
            sys.path.insert(0, module_path)
        # Try to import to verify
        try:
            from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
            module_found = True
            break
        except ImportError:
            continue

if not module_found:
    # Last attempt: try installing from source if setup.py exists
    for path in possible_paths:
        abs_path = os.path.abspath(path)
        setup_py = os.path.join(abs_path, 'global_racetrajectory_optimization', 'setup.py')
        if os.path.exists(setup_py):
            import subprocess
            try:
                # Try to install in development mode
                result = subprocess.run(
                    [sys.executable, '-m', 'pip', 'install', '-e', os.path.dirname(setup_py)],
                    capture_output=True, timeout=30
                )
                if result.returncode == 0:
                    from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer
                    module_found = True
                    break
            except:
                pass

if not module_found:
    raise ImportError("Could not find global_racetrajectory_optimization module. Please install it with: pip install -e <path_to_global_racetrajectory_optimization>")

if __name__ == "__main__":
    input_path = sys.argv[1]
    track_name = sys.argv[2]
    curv_opt_type = sys.argv[3]
    safety_width = float(sys.argv[4])
    plot = sys.argv[5].lower() == 'true'
    result_path = sys.argv[6]
    
    # Try to find config file automatically
    veh_params_file = None
    possible_config_paths = [
        "/home/misys/forza_ws/race_stack/stack_master/config/global_planner/racecar_f110.ini",
        os.path.join(os.path.expanduser("~"), "forza_ws/race_stack/stack_master/config/global_planner/racecar_f110.ini"),
    ]
    
    # Also try to find in module params directory
    for path in possible_paths:
        abs_path = os.path.abspath(path)
        if os.path.exists(abs_path):
            module_path = abs_path
            if os.path.exists(os.path.join(abs_path, 'global_racetrajectory_optimization')):
                module_path = abs_path
            elif os.path.basename(abs_path) == 'global_racetrajectory_optimization':
                module_path = os.path.dirname(abs_path)
            else:
                continue
            
            params_path = os.path.join(module_path, 'global_racetrajectory_optimization', 'params', 'racecar_f110.ini')
            if os.path.exists(params_path):
                veh_params_file = params_path
                break
            
            # Also try racecar.ini as fallback
            params_path_fallback = os.path.join(module_path, 'global_racetrajectory_optimization', 'params', 'racecar.ini')
            if os.path.exists(params_path_fallback):
                veh_params_file = params_path_fallback
                break
    
    # Try absolute paths
    if veh_params_file is None:
        for config_path in possible_config_paths:
            if os.path.exists(config_path):
                veh_params_file = config_path
                break
    
    try:
        traj, bound_r, bound_l, lap_time = trajectory_optimizer(
            input_path=input_path,
            track_name=track_name,
            curv_opt_type=curv_opt_type,
            safety_width=safety_width,
            plot=plot,
            veh_params_file=veh_params_file
        )
        
        # Validate results before converting
        if traj is None or traj.size == 0:
            raise ValueError(f"trajectory_optimizer returned empty trajectory. traj shape: {traj.shape if traj is not None else 'None'}")
        if bound_r is None or bound_r.size == 0:
            raise ValueError(f"trajectory_optimizer returned empty bound_r. bound_r shape: {bound_r.shape if bound_r is not None else 'None'}")
        if bound_l is None or bound_l.size == 0:
            raise ValueError(f"trajectory_optimizer returned empty bound_l. bound_l shape: {bound_l.shape if bound_l is not None else 'None'}")
        
        # Convert numpy arrays to lists for JSON serialization
        result = {
            "trajectory": traj.tolist(),
            "bound_r": bound_r.tolist(),
            "bound_l": bound_l.tolist(),
            "lap_time": float(lap_time)
        }
        
        with open(result_path, 'w') as f:
            json.dump(result, f)
        
        sys.exit(0)
    except Exception as e:
        import traceback
        print(f"Error: {e}", file=sys.stderr)
        print("Traceback:", file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        sys.exit(1)
)";

  script_file.close();

  // Build command to execute Python script
  // Use python3 -u for unbuffered output
  std::stringstream cmd;
  cmd << "python3 -u " << script_path.string() << " "
      << "\"" << input_path << "\" "
      << "\"" << track_name << "\" "
      << "\"" << curv_opt_type << "\" "
      << safety_width << " "
      << (plot ? "true" : "false") << " "
      << "\"" << result_path.string() << "\" 2>&1";

  // Execute Python script and capture output
  std::array<char, 128> buffer;
  std::string output;
  FILE * pipe = popen(cmd.str().c_str(), "r");
  if (!pipe) {
    std::filesystem::remove(script_path);
    throw std::runtime_error("Failed to execute trajectory_optimizer Python script");
  }
  
  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    output += buffer.data();
  }
  
  // Check exit status
  int ret = pclose(pipe);
  if (ret != 0) {
    std::cerr << "Python script output: " << output << std::endl;
    std::filesystem::remove(script_path);
    throw std::runtime_error("trajectory_optimizer Python script failed with exit code " + std::to_string(ret));
  }

  // Read result JSON file
  std::ifstream result_file(result_path);
  if (!result_file.is_open()) {
    std::filesystem::remove(script_path);
    throw std::runtime_error("Failed to open trajectory_optimizer result file");
  }

  // Parse JSON (simple parsing - could use nlohmann/json but keeping it simple)
  std::string json_content((std::istreambuf_iterator<char>(result_file)),
                           std::istreambuf_iterator<char>());
  result_file.close();

  // Parse JSON result
  try {
    json j = json::parse(json_content);
    
    // Parse trajectory
    if (j.contains("trajectory") && j["trajectory"].is_array()) {
      for (const auto & row : j["trajectory"]) {
        if (row.is_array() && row.size() >= 7) {
          Eigen::VectorXd point(7);
          point << row[0].get<double>(), row[1].get<double>(), row[2].get<double>(),
                   row[3].get<double>(), row[4].get<double>(), row[5].get<double>(),
                   row[6].get<double>();
          result.trajectory.push_back(point);
        }
      }
    }
    
    // Parse bound_r
    if (j.contains("bound_r") && j["bound_r"].is_array()) {
      for (const auto & row : j["bound_r"]) {
        if (row.is_array() && row.size() >= 2) {
          result.bound_r.push_back(Eigen::Vector2d(row[0].get<double>(), row[1].get<double>()));
        }
      }
    }
    
    // Parse bound_l
    if (j.contains("bound_l") && j["bound_l"].is_array()) {
      for (const auto & row : j["bound_l"]) {
        if (row.is_array() && row.size() >= 2) {
          result.bound_l.push_back(Eigen::Vector2d(row[0].get<double>(), row[1].get<double>()));
        }
      }
    }
    
    // Parse lap_time
    if (j.contains("lap_time")) {
      result.est_lap_time = j["lap_time"].get<double>();
    }
  } catch (const json::exception & e) {
    std::filesystem::remove(script_path);
    std::filesystem::remove(result_path);
    throw std::runtime_error("Failed to parse JSON result: " + std::string(e.what()));
  }

  // Clean up temporary files
  std::filesystem::remove(script_path);
  std::filesystem::remove(result_path);

  return result;
}

}  // namespace global_planner