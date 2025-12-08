# Global Planner (C++ Version)

This is a **complete C++ replacement** for the Python `global_planner_orig` package. It provides the same functionality as the original Python version, including:

- Map processing from occupancy grids
- Centerline extraction
- Track bounds detection
- Global trajectory optimization (calls Python trajectory_optimizer)
- Waypoint publishing

## Complete Package Replacement

This C++ package **completely replaces** `global_planner_orig`. The required Python module `global_racetrajectory_optimization` is included in this package under `python_modules/`, so you can safely remove `global_planner_orig` after building this package.

## Building

```bash
cd /path/to/workspace
colcon build --packages-select global_planner
```

After building, the Python modules will be installed to:
```
install/global_planner/share/global_planner/python_modules/
```

## Dependencies

- ROS2 (Humble/Iron)
- OpenCV
- Eigen3
- yaml-cpp
- nlohmann/json
- f110_msgs
- visualization_msgs
- nav_msgs
- geometry_msgs
- Python3 with numpy and trajectory_planning_helpers

## Python Dependencies

The C++ package includes the `global_racetrajectory_optimization` Python module in `python_modules/`. You need to:

1. **Install the global_racetrajectory_optimization module**:
   ```bash
   cd /path/to/planner/global_planner
   ./install_global_racetrajectory_optimization.sh
   ```
   
   Or manually:
   ```bash
   cd /path/to/planner/global_planner/python_modules/global_racetrajectory_optimization
   pip install -e .
   # Or for user install:
   pip install --user -e .
   ```

2. **Install required Python packages**:
   ```bash
   pip3 install numpy trajectory_planning_helpers matplotlib scipy scikit-learn
   # For minimum time optimization (optional):
   pip3 install casadi
   ```

3. **Add to PYTHONPATH** (if not using pip install -e):
   ```bash
   export PYTHONPATH=${PYTHONPATH}:/path/to/planner/global_planner/python_modules
   ```

**Note**: The module must be importable as `from global_racetrajectory_optimization.trajectory_optimizer import trajectory_optimizer`. If you encounter import errors, make sure the module is installed or PYTHONPATH is set correctly.

## Usage

### Global Planner Node

```bash
ros2 run global_planner global_planner_node
```

Parameters:
- `rate`: Update rate (default: 10.0)
- `map_name`: Name of the map (default: "map")
- `create_map`: Whether to create a new map (default: false)
- `map_editor`: Map editor mode (default: false)
- `reverse_mapping`: Reverse mapping direction (default: false)
- `safety_width`: Safety width for trajectory (default: 0.3)
- `safety_width_sp`: Safety width for shortest path (default: 0.2)
- `occupancy_grid_threshold`: Threshold for occupancy grid (default: 0.65)
- `filter_kernel_size`: Filter kernel size (default: 3)
- `show_plots`: Show plots (default: false)
- `required_laps`: Required number of laps (default: 1)

### Global Trajectory Publisher

```bash
ros2 run global_planner global_trajectory_publisher --ros-args -p map_path:=/path/to/map/directory
```

This node republishes global waypoints from a JSON file.

## Topics

### Subscribed
- `/map` (nav_msgs/msg/OccupancyGrid): Map data
- `/car_state/pose` (geometry_msgs/msg/PoseStamped): Car pose

### Published
- `/global_waypoints` (f110_msgs/msg/WpntArray): Global waypoints
- `/centerline_waypoints` (f110_msgs/msg/WpntArray): Centerline waypoints
- `/global_waypoints/markers` (visualization_msgs/msg/MarkerArray): Global waypoint markers
- `/centerline_waypoints/markers` (visualization_msgs/msg/MarkerArray): Centerline markers
- `/trackbounds/markers` (visualization_msgs/msg/MarkerArray): Track bounds markers
- `/global_waypoints/shortest_path` (f110_msgs/msg/WpntArray): Shortest path waypoints
- `/global_waypoints/shortest_path/markers` (visualization_msgs/msg/MarkerArray): Shortest path markers
- `/map_infos` (std_msgs/msg/String): Map information
- `/estimated_lap_time` (std_msgs/msg/Float32): Estimated lap time

## Replacing Python Version

This C++ version **completely replaces** the Python `global_planner_orig` package:

1. **Build the C++ package**: `colcon build --packages-select global_planner`
2. **Remove old package**: You can now safely remove `global_planner_orig`
3. **Use C++ nodes**: Use `ros2 run global_planner global_planner_node` instead of the Python node

The interface and behavior are identical:
- Same ROS2 topics and messages
- Same parameters
- Same functionality (calls Python trajectory_optimizer internally from included module)

## Package Structure

```
global_planner/
├── CMakeLists.txt
├── package.xml
├── include/global_planner/
│   ├── global_planner_logic.hpp
│   ├── global_planner_utils.hpp
│   ├── readwrite_global_waypoints.hpp
│   └── trajectory_optimizer_wrapper.hpp
├── src/
│   ├── global_planner_node.cpp
│   ├── global_planner_logic.cpp
│   ├── global_planner_utils.cpp
│   ├── global_trajectory_publisher.cpp
│   ├── readwrite_global_waypoints.cpp
│   └── trajectory_optimizer_wrapper.cpp
└── python_modules/
    └── global_racetrajectory_optimization/  (Required Python module)
        ├── global_racetrajectory_optimization/
        │   ├── trajectory_optimizer.py
        │   ├── helper_funcs_glob/
        │   ├── opt_mintime_traj/
        │   └── ...
        └── ...
```

## Notes

- The trajectory optimizer is called via Python subprocess, so Python dependencies must be available
- The C++ version maintains the same interface and behavior as the Python version
- Some Python-specific features (like matplotlib plotting) are disabled or simplified
- The trajectory optimizer integration calls the Python module from the included `python_modules/` directory
