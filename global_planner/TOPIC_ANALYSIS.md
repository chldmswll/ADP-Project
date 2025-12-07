# Global Planner 토픽 발행/구독 분석

## 토픽 구조 개요

### 1. global_planner_node (메인 플래너 노드)

#### 구독 토픽
- `/map` (nav_msgs::msg::OccupancyGrid)
  - 맵 데이터를 받아서 경로 생성에 사용
  - 콜백: `map_callback()`
  - 로그: ✅ "Subscribed to /map"

- `/car_state/pose` (geometry_msgs::msg::PoseStamped)
  - 차량 상태 정보
  - 콜백: `car_state_callback()` (현재 비어있음)
  - 로그: ❌ 로그 메시지 없음

#### 발행 토픽 (내부 토픽)
- `/global_planner/waypoints` (f110_msgs::msg::WpntArray)
  - 최적화된 경로 웨이포인트
  - 로그: ✅ "Publishing to /global_planner/waypoints"

- `/global_planner/shortest_path` (f110_msgs::msg::WpntArray)
  - 최단 경로 웨이포인트

- `/global_planner/waypoints/markers` (visualization_msgs::msg::MarkerArray)
  - 웨이포인트 시각화 마커

- `/global_planner/shortest_path/markers` (visualization_msgs::msg::MarkerArray)
  - 최단 경로 시각화 마커

- `/global_planner/trackbounds/markers` (visualization_msgs::msg::MarkerArray)
  - 트랙 경계선 시각화 마커

---

### 2. global_trajectory_publisher (리퍼블리셔 노드)

#### 구독 토픽 (global_planner_node의 내부 토픽)
- `/global_planner/waypoints` (f110_msgs::msg::WpntArray)
  - 콜백: `waypoints_callback()`
  - 로그: ✅ 5초마다 "Republished %zu waypoints to /global_waypoints"

- `/global_planner/shortest_path` (f110_msgs::msg::WpntArray)
  - 콜백: `shortest_path_callback()`
  - 로그: ❌ 로그 메시지 없음

- `/global_planner/waypoints/markers` (visualization_msgs::msg::MarkerArray)
  - 콜백: `waypoints_markers_callback()`
  - 로그: ❌ 로그 메시지 없음

- `/global_planner/shortest_path/markers` (visualization_msgs::msg::MarkerArray)
  - 콜백: `shortest_path_markers_callback()`
  - 로그: ❌ 로그 메시지 없음

- `/global_planner/trackbounds/markers` (visualization_msgs::msg::MarkerArray)
  - 콜백: `trackbounds_markers_callback()`
  - 로그: ❌ 로그 메시지 없음

#### 발행 토픽 (외부 토픽)
- `/global_waypoints` (f110_msgs::msg::WpntArray)
  - 최종 웨이포인트 (외부 노드에서 사용)

- `/global_waypoints/shortest_path` (f110_msgs::msg::WpntArray)
  - 최단 경로 웨이포인트

- `/global_waypoints/markers` (visualization_msgs::msg::MarkerArray)
  - 웨이포인트 시각화 마커

- `/global_waypoints/shortest_path/markers` (visualization_msgs::msg::MarkerArray)
  - 최단 경로 시각화 마커

- `/trackbounds/markers` (visualization_msgs::msg::MarkerArray)
  - 트랙 경계선 시각화 마커

---

## 토픽 연결 체인

```
외부 맵 발행자
    ↓
[/map] → global_planner_node → [/global_planner/waypoints] → global_trajectory_publisher → [/global_waypoints] → 외부 구독자
         (구독)                    (발행)                        (구독)                        (발행)
```

---

## 발견된 문제점 및 수정 사항

### ✅ 수정 완료

1. **로그 메시지 추가**
   - `car_state_sub_` 구독에 로그 메시지 추가
   - 모든 발행 토픽에 로그 메시지 추가
   - `shortest_path_callback()`에 로그 메시지 추가
   - 마커 콜백들에 디버그 로그 추가

2. **car_state_callback 개선**
   - 디버그 로그 메시지 추가 (차량 위치 정보 출력)

3. **일관성 있는 로깅**
   - 모든 구독/발행 토픽에 초기화 로그 추가
   - 콜백 함수들에 적절한 로그 레벨 적용 (INFO/DEBUG)

---

## 토픽 확인 방법

### 1. ROS2 명령어로 확인

```bash
# 모든 토픽 목록 확인
ros2 topic list

# 특정 토픽 정보 확인
ros2 topic info /global_waypoints
ros2 topic info /global_planner/waypoints

# 토픽 메시지 확인 (실시간)
ros2 topic echo /global_waypoints
ros2 topic echo /global_planner/waypoints

# 토픽 발행 빈도 확인
ros2 topic hz /global_waypoints
ros2 topic hz /global_planner/waypoints
```

### 2. 노드 실행 시 로그 확인

노드가 시작될 때 다음과 같은 로그가 출력되어야 합니다:

**global_planner_node:**
```
[INFO] Global planner node starting...
[INFO] Subscribed to /map
[INFO] Subscribed to /car_state/pose
[INFO] Publishing to /global_planner/waypoints
[INFO] Publishing to /global_planner/shortest_path
[INFO] Publishing to /global_planner/waypoints/markers
[INFO] Publishing to /global_planner/shortest_path/markers
[INFO] Publishing to /global_planner/trackbounds/markers
[INFO] Global planner node initialized, waiting for map...
```

**global_trajectory_publisher:**
```
[INFO] Global trajectory publisher starting...
[INFO] Subscribed to /global_planner/waypoints
[INFO] Subscribed to /global_planner/shortest_path
[INFO] Subscribed to /global_planner/waypoints/markers
[INFO] Subscribed to /global_planner/shortest_path/markers
[INFO] Subscribed to /global_planner/trackbounds/markers
[INFO] Publishing to /global_waypoints
[INFO] Publishing to /global_waypoints/shortest_path
[INFO] Publishing to /global_waypoints/markers
[INFO] Publishing to /global_waypoints/shortest_path/markers
[INFO] Publishing to /trackbounds/markers
[INFO] Global trajectory publisher initialized
```

### 3. 토픽 연결 확인

다음 명령어로 토픽이 제대로 연결되어 있는지 확인:

```bash
# global_planner_node의 발행 토픽 확인
ros2 node info /global_planner_node

# global_trajectory_publisher의 구독/발행 토픽 확인
ros2 node info /global_trajectory_publisher
```

---

## 예상 동작 흐름

1. **맵 수신**: `/map` 토픽으로 맵 데이터 수신
2. **경로 생성**: centerline 추출 → 곡률 경로 → 속도 프로파일 → 최적 경로
3. **내부 발행**: `/global_planner/waypoints` 등으로 발행
4. **리퍼블리시**: `global_trajectory_publisher`가 내부 토픽을 구독하여 외부 토픽으로 재발행
5. **외부 사용**: `/global_waypoints` 토픽을 다른 노드에서 구독하여 사용
6. **JSON 저장**: `global_waypoints.json` 파일 자동 생성 (설정 가능)

---

## ✅ 추가된 기능: global_waypoints.json 자동 생성

### 구현 내용

1. **FileWriter 통합**
   - `global_planner_node`에 `FileWriter` 클래스 통합
   - 경로 생성 후 자동으로 JSON 파일 저장

2. **파일 저장 위치**
   - 기본값: 현재 작업 디렉토리
   - ROS2 파라미터로 설정 가능: `map_dir`

3. **저장되는 데이터**
   - `map_info_str`: 맵 정보 문자열
   - `est_lap_time`: 예상 랩 타임
   - `centerline_waypoints`: 센터라인 웨이포인트
   - `global_traj_wpnts_iqp`: 최적화된 경로 (IQP)
   - `global_traj_wpnts_sp`: 최단 경로 (SP)

4. **설정 방법**

```bash
# launch 파일에서 파라미터 설정
ros2 run global_planner global_planner --ros-args -p map_dir:=/path/to/maps -p save_json:=true
```

### 수정된 파일

- ✅ `src/file_writer.cpp`: 메시지 필드명 수정 (`.wpnts`, `x_m`, `y_m`, `vx_mps`, `kappa_radpm` 등)
- ✅ `src/global_planner_node.cpp`: FileWriter 통합 및 JSON 저장 로직 추가
- ✅ `include/global_planner_node.hpp`: FileWriter 헤더 및 멤버 변수 추가
- ✅ `CMakeLists.txt`: file_writer_lib 라이브러리 추가 및 링크

---

## 최종 확인 사항

### 토픽 발행 확인
- ✅ 모든 구독/발행 토픽에 로그 메시지 추가
- ✅ 토픽 연결 구조 정상

### JSON 파일 생성 확인
- ✅ FileWriter가 global_planner_node에 통합됨
- ✅ 메시지 필드명 수정 완료 (`.wpnts` 사용)
- ✅ CMakeLists.txt에 라이브러리 추가 완료

### 테스트 방법

1. **노드 실행**
```bash
ros2 run global_planner global_planner
ros2 run global_planner global_trajectory_publisher
```

2. **토픽 확인**
```bash
ros2 topic list
ros2 topic echo /global_waypoints
```

3. **JSON 파일 확인**
```bash
ls -la global_waypoints.json
cat global_waypoints.json
```

