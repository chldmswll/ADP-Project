// 섹터 분할 기능 사용 예제
// 이 파일은 예제이며, 실제 사용 시 global_planner_node.cpp에 통합할 수 있습니다

#include "sector_divider.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include <iostream>

void example_usage() {
    // SectorDivider 객체 생성
    SectorDivider divider;
    
    // 예제: 웨이포인트 배열 생성 (실제로는 global_planner에서 받아옴)
    f110_msgs::msg::WpntArray waypoints;
    // ... waypoints 채우기 ...
    
    // 방법 1: 기본 방식 (곡률과 거리를 모두 고려)
    auto sectors = divider.divide_into_sectors(waypoints);
    
    // 방법 2: 고정 거리로 섹터 분할 (10m마다)
    auto sectors_by_distance = divider.divide_by_distance(waypoints, 10.0f);
    
    // 방법 3: 곡률 변화 기준으로 섹터 분할
    auto sectors_by_curvature = divider.divide_by_curvature(waypoints, 0.1f);
    
    // 방법 4: 혼합 방식 (곡률과 거리 모두 고려)
    auto sectors_mixed = divider.divide_by_curvature_and_distance(
        waypoints,
        0.1f,    // 곡률 임계값
        5.0f,    // 최소 섹터 길이
        50.0f    // 최대 섹터 길이
    );
    
    // 섹터 정보 출력
    divider.print_sectors(sectors);
    
    // 각 섹터에 대한 처리 예제
    for (const auto& sector : sectors) {
        std::cout << "섹터 처리: " << sector.type_name << std::endl;
        // 섹터별 특화 처리 로직 추가 가능
    }
}

