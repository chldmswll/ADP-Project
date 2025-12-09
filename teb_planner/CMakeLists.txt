cmake_minimum_required(VERSION 3.8)
project(teb_planner)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(frenet_conversion_cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)

add_executable(teb_planner
  src/spline_planner_node.cpp
  src/teb.cpp
  src/fuzzy_controller.cpp
)

target_include_directories(teb_planner PRIVATE include)

ament_target_dependencies(teb_planner
  rclcpp
  rcl_interfaces
  f110_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  frenet_conversion_cpp
  Eigen3
)

target_link_libraries(teb_planner Eigen3::Eigen)

install(TARGETS teb_planner
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

