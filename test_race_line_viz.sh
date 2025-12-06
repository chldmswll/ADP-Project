#!/bin/bash

# 레이스라인 시각화 테스트 스크립트
# 사용법: ./test_race_line_viz.sh <맵이름>

if [ -z "$1" ]; then
    echo "사용법: $0 <맵이름>"
    echo "예: $0 hangar_12_v0"
    exit 1
fi

MAP_NAME=$1

echo "=========================================="
echo "레이스라인 시각화 테스트"
echo "=========================================="
echo ""
echo "1. Base System 실행 중..."
echo "   - global_trajectory_publisher가 /global_waypoints 발행"
echo ""

# Base System 실행 (백그라운드)
cd /home/subin/shared_dir/forza_ws/race_stack
ros2 launch stack_master base_system_launch.xml \
  map_name:=$MAP_NAME \
  sim:=true \
  racecar_version:=SIM &

BASE_PID=$!
echo "Base System PID: $BASE_PID"
sleep 3

echo ""
echo "2. ADP-Project Local Planner 실행 중..."
echo "   - teb_local_planner가 /global_waypoints를 구독"
echo "   - /global_waypoints/markers로 변환하여 발행"
echo ""

# ADP-Project Local Planner 실행 (백그라운드)
cd /home/subin/shared_dir/forza_ws/race_stack/ADP-Project
source install/setup.bash
ros2 launch teb_local_planner local_planner.launch.py &

PLANNER_PID=$!
echo "Local Planner PID: $PLANNER_PID"
sleep 2

echo ""
echo "3. Rviz 실행 중..."
echo "   - forza_rviz.rviz 설정 파일 사용"
echo "   - 'Global Waypoints' 디스플레이에서 레이스라인 확인"
echo ""

# Rviz 실행
cd /home/subin/shared_dir/forza_ws/race_stack
rviz2 -d forza_rviz.rviz

echo ""
echo "종료 중..."
kill $BASE_PID $PLANNER_PID 2>/dev/null
echo "완료!"

