// 레이스라인 섹터 분할 구현
// Python global_planner 코드를 참고하여 레이스라인을 섹터로 나누는 기능 구현

#include "sector_divider.hpp"
#include "utils.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

SectorDivider::SectorDivider() 
    : curvature_threshold_(0.1f),
      min_sector_length_(5.0f),
      max_sector_length_(50.0f) {
}

std::vector<Sector> SectorDivider::divide_into_sectors(const WpntArray& waypoints) {
    // 기본적으로 곡률과 거리를 모두 고려한 혼합 방식 사용
    return divide_by_curvature_and_distance(waypoints);
}

std::vector<Sector> SectorDivider::divide_by_distance(
    const WpntArray& waypoints, 
    float sector_length) {
    
    std::vector<Sector> sectors;
    
    if (waypoints.waypoints.empty()) {
        return sectors;
    }
    
    size_t start_idx = 0;
    float current_distance = 0.0f;
    float sector_start_distance = 0.0f;
    
    for (size_t i = 1; i < waypoints.waypoints.size(); ++i) {
        const auto& prev = waypoints.waypoints[i - 1];
        const auto& curr = waypoints.waypoints[i];
        
        float segment_distance = calculate_distance(prev, curr);
        current_distance += segment_distance;
        
        // 섹터 길이에 도달하면 새 섹터 시작
        if (current_distance >= sector_length) {
            Sector sector;
            sector.start_index = start_idx;
            sector.end_index = i;
            sector.start_distance = sector_start_distance;
            sector.end_distance = sector_start_distance + current_distance;
            sector.length = current_distance;
            
            // 섹터 내 웨이포인트 추출
            std::vector<Wpnt> sector_waypoints;
            for (size_t j = start_idx; j <= i; ++j) {
                sector_waypoints.push_back(waypoints.waypoints[j]);
            }
            
            // 곡률 통계 계산
            calculate_sector_curvature_stats(
                sector_waypoints,
                sector.avg_curvature,
                sector.max_curvature,
                sector.min_curvature);
            
            // 섹터 타입 결정
            sector.type = determine_sector_type(
                sector_waypoints,
                sector.avg_curvature);
            
            // 타입 이름 설정
            switch (sector.type) {
                case SectorType::STRAIGHT:
                    sector.type_name = "직선";
                    break;
                case SectorType::CORNER:
                    sector.type_name = "코너";
                    break;
                case SectorType::CHICANE:
                    sector.type_name = "시케인";
                    break;
                default:
                    sector.type_name = "알 수 없음";
                    break;
            }
            
            sectors.push_back(sector);
            
            // 다음 섹터 시작
            start_idx = i;
            sector_start_distance += current_distance;
            current_distance = 0.0f;
        }
    }
    
    // 마지막 섹터 처리
    if (start_idx < waypoints.waypoints.size() - 1) {
        Sector sector;
        sector.start_index = start_idx;
        sector.end_index = waypoints.waypoints.size() - 1;
        sector.start_distance = sector_start_distance;
        sector.end_distance = sector_start_distance + current_distance;
        sector.length = current_distance;
        
        std::vector<Wpnt> sector_waypoints;
        for (size_t j = start_idx; j < waypoints.waypoints.size(); ++j) {
            sector_waypoints.push_back(waypoints.waypoints[j]);
        }
        
        calculate_sector_curvature_stats(
            sector_waypoints,
            sector.avg_curvature,
            sector.max_curvature,
            sector.min_curvature);
        
        sector.type = determine_sector_type(
            sector_waypoints,
            sector.avg_curvature);
        
        // 타입 이름 설정
        switch (sector.type) {
            case SectorType::STRAIGHT:
                sector.type_name = "직선";
                break;
            case SectorType::CORNER:
                sector.type_name = "코너";
                break;
            case SectorType::CHICANE:
                sector.type_name = "시케인";
                break;
            default:
                sector.type_name = "알 수 없음";
                break;
        }
        
        sectors.push_back(sector);
    }
    
    return sectors;
}

std::vector<Sector> SectorDivider::divide_by_curvature(
    const WpntArray& waypoints,
    float curvature_threshold) {
    
    std::vector<Sector> sectors;
    
    if (waypoints.waypoints.size() < 3) {
        return sectors;
    }
    
    size_t start_idx = 0;
    bool in_corner = false;
    float cumulative_distance = 0.0f;
    float sector_start_distance = 0.0f;
    
    // 첫 번째 웨이포인트의 곡률 확인
    float first_curvature = std::abs(waypoints.waypoints[0].curvature);
    in_corner = (first_curvature > curvature_threshold);
    
    for (size_t i = 1; i < waypoints.waypoints.size() - 1; ++i) {
        const auto& prev = waypoints.waypoints[i - 1];
        const auto& curr = waypoints.waypoints[i];
        const auto& next = waypoints.waypoints[i + 1];
        
        float segment_distance = calculate_distance(prev, curr);
        cumulative_distance += segment_distance;
        
        float abs_curvature = std::abs(curr.curvature);
        bool is_corner = (abs_curvature > curvature_threshold);
        
        // 섹터 타입이 변경되면 새 섹터 시작
        if (is_corner != in_corner) {
            // 현재까지의 섹터 저장
            if (i - start_idx > 1) {
                Sector sector;
                sector.start_index = start_idx;
                sector.end_index = i - 1;
                sector.start_distance = sector_start_distance;
                sector.end_distance = cumulative_distance - segment_distance;
                sector.length = sector.end_distance - sector.start_distance;
                
                std::vector<Wpnt> sector_waypoints;
                for (size_t j = start_idx; j < i; ++j) {
                    sector_waypoints.push_back(waypoints.waypoints[j]);
                }
                
                calculate_sector_curvature_stats(
                    sector_waypoints,
                    sector.avg_curvature,
                    sector.max_curvature,
                    sector.min_curvature);
                
                sector.type = determine_sector_type(
                    sector_waypoints,
                    sector.avg_curvature);
                
                // 타입 이름 설정
                switch (sector.type) {
                    case SectorType::STRAIGHT:
                        sector.type_name = "직선";
                        break;
                    case SectorType::CORNER:
                        sector.type_name = "코너";
                        break;
                    case SectorType::CHICANE:
                        sector.type_name = "시케인";
                        break;
                    default:
                        sector.type_name = "알 수 없음";
                        break;
                }
                
                sectors.push_back(sector);
            }
            
            // 새 섹터 시작
            start_idx = i - 1;
            sector_start_distance = cumulative_distance - segment_distance;
            in_corner = is_corner;
        }
    }
    
    // 마지막 섹터 처리
    if (start_idx < waypoints.waypoints.size() - 1) {
        Sector sector;
        sector.start_index = start_idx;
        sector.end_index = waypoints.waypoints.size() - 1;
        sector.start_distance = sector_start_distance;
        
        // 마지막 섹터의 누적 거리 계산
        float final_distance = sector_start_distance;
        for (size_t j = start_idx; j < waypoints.waypoints.size() - 1; ++j) {
            final_distance += calculate_distance(
                waypoints.waypoints[j],
                waypoints.waypoints[j + 1]);
        }
        
        sector.end_distance = final_distance;
        sector.length = sector.end_distance - sector.start_distance;
        
        std::vector<Wpnt> sector_waypoints;
        for (size_t j = start_idx; j < waypoints.waypoints.size(); ++j) {
            sector_waypoints.push_back(waypoints.waypoints[j]);
        }
        
        calculate_sector_curvature_stats(
            sector_waypoints,
            sector.avg_curvature,
            sector.max_curvature,
            sector.min_curvature);
        
        sector.type = determine_sector_type(
            sector_waypoints,
            sector.avg_curvature);
        
        // 타입 이름 설정
        switch (sector.type) {
            case SectorType::STRAIGHT:
                sector.type_name = "직선";
                break;
            case SectorType::CORNER:
                sector.type_name = "코너";
                break;
            case SectorType::CHICANE:
                sector.type_name = "시케인";
                break;
            default:
                sector.type_name = "알 수 없음";
                break;
        }
        
        sectors.push_back(sector);
    }
    
    return sectors;
}

std::vector<Sector> SectorDivider::divide_by_curvature_and_distance(
    const WpntArray& waypoints,
    float curvature_threshold,
    float min_sector_length,
    float max_sector_length) {
    
    std::vector<Sector> sectors;
    
    if (waypoints.waypoints.size() < 3) {
        return sectors;
    }
    
    size_t start_idx = 0;
    float cumulative_distance = 0.0f;
    float sector_start_distance = 0.0f;
    float sector_length = 0.0f;
    
    // 첫 번째 섹터의 곡률 상태 확인
    float first_curvature = std::abs(waypoints.waypoints[0].curvature);
    bool in_corner = (first_curvature > curvature_threshold);
    
    for (size_t i = 1; i < waypoints.waypoints.size(); ++i) {
        const auto& prev = waypoints.waypoints[i - 1];
        const auto& curr = waypoints.waypoints[i];
        
        float segment_distance = calculate_distance(prev, curr);
        cumulative_distance += segment_distance;
        sector_length += segment_distance;
        
        float abs_curvature = std::abs(curr.curvature);
        bool is_corner = (abs_curvature > curvature_threshold);
        
        bool should_split = false;
        
        // 섹터 타입이 변경되고 최소 길이 이상이면 분할
        if (is_corner != in_corner && sector_length >= min_sector_length) {
            should_split = true;
        }
        // 최대 길이를 초과하면 분할
        else if (sector_length >= max_sector_length) {
            should_split = true;
        }
        
        if (should_split) {
            // 현재까지의 섹터 저장
            Sector sector;
            sector.start_index = start_idx;
            sector.end_index = i - 1;
            sector.start_distance = sector_start_distance;
            sector.end_distance = cumulative_distance - segment_distance;
            sector.length = sector.end_distance - sector.start_distance;
            
            std::vector<Wpnt> sector_waypoints;
            for (size_t j = start_idx; j < i; ++j) {
                sector_waypoints.push_back(waypoints.waypoints[j]);
            }
            
            calculate_sector_curvature_stats(
                sector_waypoints,
                sector.avg_curvature,
                sector.max_curvature,
                sector.min_curvature);
            
            sector.type = determine_sector_type(
                sector_waypoints,
                sector.avg_curvature);
            
            // 타입 이름 설정
            switch (sector.type) {
                case SectorType::STRAIGHT:
                    sector.type_name = "직선";
                    break;
                case SectorType::CORNER:
                    sector.type_name = "코너";
                    break;
                case SectorType::CHICANE:
                    sector.type_name = "시케인";
                    break;
                default:
                    sector.type_name = "알 수 없음";
                    break;
            }
            
            sectors.push_back(sector);
            
            // 새 섹터 시작
            start_idx = i - 1;
            sector_start_distance = cumulative_distance - segment_distance;
            sector_length = segment_distance;
            in_corner = is_corner;
        }
    }
    
    // 마지막 섹터 처리
    if (start_idx < waypoints.waypoints.size() - 1) {
        Sector sector;
        sector.start_index = start_idx;
        sector.end_index = waypoints.waypoints.size() - 1;
        sector.start_distance = sector_start_distance;
        sector.end_distance = cumulative_distance;
        sector.length = sector.end_distance - sector.start_distance;
        
        std::vector<Wpnt> sector_waypoints;
        for (size_t j = start_idx; j < waypoints.waypoints.size(); ++j) {
            sector_waypoints.push_back(waypoints.waypoints[j]);
        }
        
        calculate_sector_curvature_stats(
            sector_waypoints,
            sector.avg_curvature,
            sector.max_curvature,
            sector.min_curvature);
        
        sector.type = determine_sector_type(
            sector_waypoints,
            sector.avg_curvature);
        
        // 타입 이름 설정
        switch (sector.type) {
            case SectorType::STRAIGHT:
                sector.type_name = "직선";
                break;
            case SectorType::CORNER:
                sector.type_name = "코너";
                break;
            case SectorType::CHICANE:
                sector.type_name = "시케인";
                break;
            default:
                sector.type_name = "알 수 없음";
                break;
        }
        
        sectors.push_back(sector);
    }
    
    return sectors;
}

SectorType SectorDivider::determine_sector_type(
    const std::vector<Wpnt>& sector_waypoints,
    float avg_curvature) {
    
    if (sector_waypoints.empty()) {
        return SectorType::UNKNOWN;
    }
    
    float abs_avg_curvature = std::abs(avg_curvature);
    
    // 직선 구간: 곡률이 매우 작음
    if (abs_avg_curvature < 0.05f) {
        return SectorType::STRAIGHT;
    }
    
    // 곡률 변화가 큰 경우 시케인으로 판단
    if (sector_waypoints.size() >= 3) {
        float curvature_variance = 0.0f;
        for (const auto& wpnt : sector_waypoints) {
            float diff = std::abs(wpnt.curvature) - abs_avg_curvature;
            curvature_variance += diff * diff;
        }
        curvature_variance /= sector_waypoints.size();
        
        // 곡률 변화가 크면 시케인
        if (curvature_variance > 0.01f) {
            return SectorType::CHICANE;
        }
    }
    
    // 코너 구간
    if (abs_avg_curvature > 0.1f) {
        return SectorType::CORNER;
    }
    
    return SectorType::UNKNOWN;
}

float SectorDivider::calculate_cumulative_distance(
    const WpntArray& waypoints,
    size_t index) {
    
    if (index == 0 || index >= waypoints.waypoints.size()) {
        return 0.0f;
    }
    
    float distance = 0.0f;
    for (size_t i = 1; i <= index; ++i) {
        distance += calculate_distance(
            waypoints.waypoints[i - 1],
            waypoints.waypoints[i]);
    }
    
    return distance;
}

void SectorDivider::calculate_sector_curvature_stats(
    const std::vector<Wpnt>& sector_waypoints,
    float& avg_curvature,
    float& max_curvature,
    float& min_curvature) {
    
    if (sector_waypoints.empty()) {
        avg_curvature = 0.0f;
        max_curvature = 0.0f;
        min_curvature = 0.0f;
        return;
    }
    
    float sum_curvature = 0.0f;
    max_curvature = std::abs(sector_waypoints[0].curvature);
    min_curvature = std::abs(sector_waypoints[0].curvature);
    
    for (const auto& wpnt : sector_waypoints) {
        float abs_curvature = std::abs(wpnt.curvature);
        sum_curvature += abs_curvature;
        
        if (abs_curvature > max_curvature) {
            max_curvature = abs_curvature;
        }
        if (abs_curvature < min_curvature) {
            min_curvature = abs_curvature;
        }
    }
    
    avg_curvature = sum_curvature / sector_waypoints.size();
}

void SectorDivider::print_sectors(const std::vector<Sector>& sectors) {
    std::cout << "=== 레이스라인 섹터 정보 ===" << std::endl;
    std::cout << "총 섹터 수: " << sectors.size() << std::endl;
    std::cout << std::endl;
    
    for (size_t i = 0; i < sectors.size(); ++i) {
        const auto& sector = sectors[i];
        
        std::string type_name;
        switch (sector.type) {
            case SectorType::STRAIGHT:
                type_name = "직선";
                break;
            case SectorType::CORNER:
                type_name = "코너";
                break;
            case SectorType::CHICANE:
                type_name = "시케인";
                break;
            default:
                type_name = "알 수 없음";
                break;
        }
        
        std::cout << "섹터 " << i + 1 << ":" << std::endl;
        std::cout << "  타입: " << type_name << std::endl;
        std::cout << "  인덱스: " << sector.start_index << " ~ " << sector.end_index << std::endl;
        std::cout << "  거리: " << sector.start_distance << "m ~ " 
                  << sector.end_distance << "m" << std::endl;
        std::cout << "  길이: " << sector.length << "m" << std::endl;
        std::cout << "  평균 곡률: " << sector.avg_curvature << " rad/m" << std::endl;
        std::cout << "  최대 곡률: " << sector.max_curvature << " rad/m" << std::endl;
        std::cout << "  최소 곡률: " << sector.min_curvature << " rad/m" << std::endl;
        std::cout << std::endl;
    }
}

