// 레이스라인 섹터 분할 헤더
// Python global_planner 코드를 참고하여 레이스라인을 섹터로 나누는 기능 구현

#ifndef SECTOR_DIVIDER_HPP
#define SECTOR_DIVIDER_HPP

#include "f110_msgs/msg/wpnt_array.hpp"
#include "f110_msgs/msg/wpnt.hpp"
#include <vector>
#include <string>

using f110_msgs::msg::Wpnt;
using f110_msgs::msg::WpntArray;

// 섹터 타입 정의
enum class SectorType {
    STRAIGHT,      // 직선 구간
    CORNER,        // 코너 구간
    CHICANE,       // 시케인 구간
    UNKNOWN        // 알 수 없음
};

// 섹터 정보 구조체
struct Sector {
    size_t start_index;           // 시작 인덱스
    size_t end_index;              // 끝 인덱스
    float start_distance;          // 시작 누적 거리 (m)
    float end_distance;            // 끝 누적 거리 (m)
    float length;                  // 섹터 길이 (m)
    SectorType type;               // 섹터 타입
    float avg_curvature;           // 평균 곡률
    float max_curvature;           // 최대 곡률
    float min_curvature;           // 최소 곡률
    std::string type_name;         // 타입 이름 (디버깅용)
};

class SectorDivider {
public:
    SectorDivider();
    
    // 레이스라인을 섹터로 분할
    // Python 코드의 웨이포인트 처리 방식을 참고
    std::vector<Sector> divide_into_sectors(const WpntArray& waypoints);
    
    // 고정 거리로 섹터 분할
    std::vector<Sector> divide_by_distance(
        const WpntArray& waypoints, 
        float sector_length = 10.0f);
    
    // 곡률 변화 기준으로 섹터 분할 (직선/코너 구분)
    std::vector<Sector> divide_by_curvature(
        const WpntArray& waypoints,
        float curvature_threshold = 0.1f);
    
    // 혼합 방식: 곡률과 거리를 모두 고려한 섹터 분할
    std::vector<Sector> divide_by_curvature_and_distance(
        const WpntArray& waypoints,
        float curvature_threshold = 0.1f,
        float min_sector_length = 5.0f,
        float max_sector_length = 50.0f);
    
    // 섹터 타입 결정
    SectorType determine_sector_type(
        const std::vector<Wpnt>& sector_waypoints,
        float avg_curvature);
    
    // 섹터 정보 출력 (디버깅용)
    void print_sectors(const std::vector<Sector>& sectors);

private:
    // 누적 거리 계산 (Python 코드의 s_m과 유사)
    float calculate_cumulative_distance(
        const WpntArray& waypoints, 
        size_t index);
    
    // 섹터의 평균/최대/최소 곡률 계산
    void calculate_sector_curvature_stats(
        const std::vector<Wpnt>& sector_waypoints,
        float& avg_curvature,
        float& max_curvature,
        float& min_curvature);
    
    // 곡률 임계값
    float curvature_threshold_;
    
    // 최소/최대 섹터 길이
    float min_sector_length_;
    float max_sector_length_;
};

#endif  // SECTOR_DIVIDER_HPP

