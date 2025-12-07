//트랙 경계선으로 중심선 추출

#include "centerline_extractor.hpp"
#include "utils.hpp"
#include <cmath>
#include <algorithm>

CenterlineExtractor::CenterlineExtractor() {
    // 초기화
}

void CenterlineExtractor::extract_centerline(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    centerline_.clear();
    left_boundary_.clear();
    right_boundary_.clear();
    
    if (!map || map->data.empty()) {
        return;
    }
    
    const int width = map->info.width;
    const int height = map->info.height;
    const double resolution = map->info.resolution;
    const double origin_x = map->info.origin.position.x;
    const double origin_y = map->info.origin.position.y;
    
    // 개선된 중심선 추출: 각 열과 행을 모두 스캔하여 더 정확한 중심선 생성
    std::vector<Wpnt> temp_centerline;
    
    // 1. 각 열(column)에 대해 좌측과 우측 경계선을 찾아 중심선 추출
    for (int x = 0; x < width; x++) {
        int left_boundary = -1;   // 좌측 경계선 (위에서 아래로 스캔)
        int right_boundary = -1;  // 우측 경계선 (아래에서 위로 스캔)
        
        // 위에서 아래로 스캔하여 좌측 경계선 찾기
        for (int y = 0; y < height - 1; y++) {
            int idx = y * width + x;
            int next_idx = (y + 1) * width + x;
            
            // free(0)에서 occupied(100)로 변하는 경계 찾기
            if (map->data[idx] == 0 && map->data[next_idx] >= 50) {
                left_boundary = y;
                break;
            }
            // 또는 occupied에서 free로 변하는 경계 (트랙 내부 경계)
            if (map->data[idx] >= 50 && map->data[next_idx] == 0) {
                left_boundary = y;
                break;
            }
        }
        
        // 아래에서 위로 스캔하여 우측 경계선 찾기
        for (int y = height - 1; y > 0; y--) {
            int idx = y * width + x;
            int prev_idx = (y - 1) * width + x;
            
            // free(0)에서 occupied(100)로 변하는 경계 찾기
            if (map->data[idx] == 0 && map->data[prev_idx] >= 50) {
                right_boundary = y;
                break;
            }
            // 또는 occupied에서 free로 변하는 경계
            if (map->data[idx] >= 50 && map->data[prev_idx] == 0) {
                right_boundary = y;
                break;
            }
        }
        
        // 두 경계선이 모두 찾아졌으면 중심선 포인트 생성
        if (left_boundary >= 0 && right_boundary >= 0 && left_boundary < right_boundary) {
            // 중심선 y 좌표 계산 (두 경계선의 중점)
            double center_y = (left_boundary + right_boundary) / 2.0;
            
            // 맵 좌표를 월드 좌표로 변환
            double world_x = origin_x + (x + 0.5) * resolution;
            double world_y = origin_y + (center_y + 0.5) * resolution;
            
            Wpnt point;
            point.x_m = world_x;
            point.y_m = world_y;
            point.psi_rad = 0.0;  // 나중에 계산
            point.kappa_radpm = 0.0;  // 나중에 계산
            point.vx_mps = 0.0;
            point.id = static_cast<int32_t>(temp_centerline.size());
            
            temp_centerline.push_back(point);
            
            // 경계선 좌표 저장
            double left_y = origin_y + (left_boundary + 0.5) * resolution;
            double right_y = origin_y + (right_boundary + 0.5) * resolution;
            left_boundary_.push_back({world_x, left_y});
            right_boundary_.push_back({world_x, right_y});
        }
    }
    
    // 2. 중심선 포인트들을 거리와 방향 기준으로 정렬 및 필터링
    if (temp_centerline.size() < 2) {
        centerline_ = temp_centerline;
        return;
    }
    
    // x 좌표 기준으로 정렬 (트랙이 대부분 수평 방향이므로)
    std::sort(temp_centerline.begin(), temp_centerline.end(), 
        [](const Wpnt& a, const Wpnt& b) {
            return a.x_m < b.x_m;
        });
    
    // 중복 제거 및 너무 가까운 포인트 제거
    centerline_.clear();
    const double min_distance = resolution * 0.3;  // 최소 거리
    
    for (size_t i = 0; i < temp_centerline.size(); ++i) {
        if (centerline_.empty()) {
            centerline_.push_back(temp_centerline[i]);
            continue;
        }
        
        const Wpnt& last = centerline_.back();
        double dist = std::sqrt(
            std::pow(temp_centerline[i].x_m - last.x_m, 2) +
            std::pow(temp_centerline[i].y_m - last.y_m, 2)
        );
        
        if (dist >= min_distance) {
            centerline_.push_back(temp_centerline[i]);
        }
    }
    
    // 3. 중심선 스무딩 (노이즈 제거)
    if (centerline_.size() > 2) {
        std::vector<Wpnt> smoothed;
        smoothed.reserve(centerline_.size());
        
        for (size_t i = 0; i < centerline_.size(); ++i) {
            Wpnt smoothed_point = centerline_[i];
            
            // 이동 평균 필터 (양쪽 2개씩 고려)
            int window = 2;
            double sum_x = 0.0, sum_y = 0.0;
            int count = 0;
            
            for (int j = -window; j <= window; ++j) {
                int idx = static_cast<int>(i) + j;
                if (idx >= 0 && idx < static_cast<int>(centerline_.size())) {
                    sum_x += centerline_[idx].x_m;
                    sum_y += centerline_[idx].y_m;
                    count++;
                }
            }
            
            if (count > 0) {
                smoothed_point.x_m = sum_x / count;
                smoothed_point.y_m = sum_y / count;
            }
            
            smoothed.push_back(smoothed_point);
        }
        
        centerline_ = smoothed;
    }
    
    // yaw 각도 계산 (이전 포인트와 현재 포인트 사이의 방향)
    for (size_t i = 1; i < centerline_.size(); i++) {
        double dx = centerline_[i].x_m - centerline_[i-1].x_m;
        double dy = centerline_[i].y_m - centerline_[i-1].y_m;
        centerline_[i-1].psi_rad = std::atan2(dy, dx);
    }
    
    // 마지막 포인트의 yaw는 이전 포인트와 동일하게 설정
    if (centerline_.size() > 1) {
        centerline_.back().psi_rad = centerline_[centerline_.size() - 2].psi_rad;
    }
    
    // 곡률 계산
    for (size_t i = 1; i < centerline_.size() - 1; i++) {
        centerline_[i].kappa_radpm = calculate_curvature(
            centerline_[i-1], 
            centerline_[i], 
            centerline_[i+1]
        );
    }
    
    // 첫 번째와 마지막 포인트의 곡률은 인접 포인트와 동일하게 설정
    if (centerline_.size() > 2) {
        centerline_[0].kappa_radpm = centerline_[1].kappa_radpm;
        centerline_.back().kappa_radpm = centerline_[centerline_.size() - 2].kappa_radpm;
    }
}

const std::vector<Wpnt>& CenterlineExtractor::get_centerline() const {
    return centerline_;
}

const std::vector<std::pair<double, double>>& CenterlineExtractor::get_left_boundary() const {
    return left_boundary_;
}

const std::vector<std::pair<double, double>>& CenterlineExtractor::get_right_boundary() const {
    return right_boundary_;
}


/*구현 내용
1. 경계선 탐지: 각 열(x)에서 좌측/우측 경계선을 탐지
좌측: 위에서 아래로 스캔하며 free(0) ↔ occupied(≥50) 경계 탐지
우측: 아래에서 위로 스캔하며 경계 탐지
2. 중심선 계산: 두 경계선의 중점을 중심선으로 사용
3. 좌표 변환: 그리드 좌표를 월드 좌표로 변환
4. yaw 계산: 이전 포인트와 현재 포인트 사이의 방향각 계산
5. 곡률 계산: 기존 calculate_curvature 함수로 곡률 계산
이제 OccupancyGrid에서 트랙 경계선을 감지해 중심선을 추출합니다. 중심선 포인트는 Wpnt 형식으로 저장되며, x, y, yaw, curvature 정보가 포함됩니다.*/