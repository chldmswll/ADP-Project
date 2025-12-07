cmake_minimum_required(VERSION 3.5)
project(global_planner)

# ROS2 의존성 패키지
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

# Include 디렉토리 설정
include_directories(include)

add_library(utils_lib src/utils.cpp)
target_include_directories(utils_lib PUBLIC
	include
)

ament_target_dependencies(utils_lib
	rclcpp
)

# 라이브러리 생성 (클래스 구현용)
add_library(centerline_extractor_lib src/centerline_extractor.cpp)
add_library(curvature_planner_lib src/curvature_planner.cpp)
add_library(velocity_planner_lib src/velocity_planner.cpp)
add_library(time_optimal_planner_lib src/time_optimal_planner.cpp)

# 소스 파일 추가 (실행 파일)
add_executable(global_planner src/global_planner_node.cpp)
add_executable(global_trajectory_publisher src/global_trajectory_publisher.cpp)

# ROS2 의존성 연결
ament_target_dependencies(global_planner 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs 
  geometry_msgs
  visualization_msgs
  f110_msgs
  )

ament_target_dependencies(global_trajectory_publisher
  rclcpp
  std_msgs
  visualization_msgs
  f110_msgs
  )

# 라이브러리 의존성
ament_target_dependencies(centerline_extractor_lib 
  rclcpp 
  nav_msgs
  f110_msgs
  )

ament_target_dependencies(curvature_planner_lib 
  rclcpp 
  f110_msgs
  )

ament_target_dependencies(velocity_planner_lib 
  rclcpp 
  f110_msgs
  )

ament_target_dependencies(time_optimal_planner_lib 
  rclcpp 
  f110_msgs
  )

ament_target_dependencies(utils_lib
  rclcpp
  f110_msgs
  )

# Utils 라이브러리 링크
target_link_libraries(centerline_extractor_lib utils_lib)
target_link_libraries(curvature_planner_lib utils_lib)
target_link_libraries(velocity_planner_lib utils_lib)
target_link_libraries(time_optimal_planner_lib utils_lib)
target_link_libraries(global_planner utils_lib)

# 플래너 라이브러리들을 global_planner에 링크
target_link_libraries(global_planner 
  centerline_extractor_lib
  curvature_planner_lib
  velocity_planner_lib
  time_optimal_planner_lib
)




# 설치
install(TARGETS
  global_planner
  global_trajectory_publisher
  centerline_extractor_lib
  curvature_planner_lib
  velocity_planner_lib
  time_optimal_planner_lib
  utils_lib
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
