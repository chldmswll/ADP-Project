cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# nlohmann/json (header-only)
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  # If not found, try to use the header-only version
  include(FetchContent)
  FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.2
  )
  FetchContent_MakeAvailable(json)
endif()

# Include directories
include_directories(include)

# Source files
set(SOURCES
  src/global_planner_node.cpp
  src/global_planner_logic.cpp
  src/global_planner_utils.cpp
  src/global_trajectory_publisher.cpp
  src/readwrite_global_waypoints.cpp
  src/trajectory_optimizer_wrapper.cpp
)

# Header files
set(HEADERS
  include/global_planner/global_planner_logic.hpp
  include/global_planner/global_planner_utils.hpp
  include/global_planner/readwrite_global_waypoints.hpp
)

# Executable: global_planner (matches launch file)
add_executable(global_planner
  src/global_planner_node.cpp
  src/global_planner_logic.cpp
  src/global_planner_utils.cpp
  src/readwrite_global_waypoints.cpp
  src/trajectory_optimizer_wrapper.cpp
)

ament_target_dependencies(global_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  f110_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

target_include_directories(global_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(global_planner
  ${OpenCV_LIBS}
  Eigen3::Eigen
  yaml-cpp
)

if(TARGET nlohmann_json::nlohmann_json)
  target_link_libraries(global_planner nlohmann_json::nlohmann_json)
elseif(TARGET json)
  target_link_libraries(global_planner json)
endif()

# Executable: global_trajectory_publisher
add_executable(global_trajectory_publisher
  src/global_trajectory_publisher.cpp
  src/readwrite_global_waypoints.cpp
)

ament_target_dependencies(global_trajectory_publisher
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  f110_msgs
  ament_index_cpp
)

target_include_directories(global_trajectory_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(global_trajectory_publisher
  yaml-cpp
)

if(TARGET nlohmann_json::nlohmann_json)
  target_link_libraries(global_trajectory_publisher nlohmann_json::nlohmann_json)
elseif(TARGET json)
  target_link_libraries(global_trajectory_publisher json)
endif()

# Install
install(TARGETS
  global_planner
  global_trajectory_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Install Python modules
install(DIRECTORY python_modules/
  DESTINATION share/${PROJECT_NAME}/python_modules
)

# Install params and opt_mintime_traj directories for global_racetrajectory_optimization
install(DIRECTORY python_modules/global_racetrajectory_optimization/global_racetrajectory_optimization/params/
  DESTINATION share/${PROJECT_NAME}/python_modules/global_racetrajectory_optimization/global_racetrajectory_optimization/params
)

install(DIRECTORY python_modules/global_racetrajectory_optimization/global_racetrajectory_optimization/opt_mintime_traj/
  DESTINATION share/${PROJECT_NAME}/python_modules/global_racetrajectory_optimization/global_racetrajectory_optimization/opt_mintime_traj
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
