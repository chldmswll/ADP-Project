//파일 저장 헤더
// Python readwrite_global_waypoints.py를 참고하여 global_waypoints.json 생성 기능 구현

#ifndef FILE_WRITER_HPP
#define FILE_WRITER_HPP

#include "f110_msgs/msg/wpnt_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>

using f110_msgs::msg::WpntArray;
using std_msgs::msg::String;
using std_msgs::msg::Float32;

class FileWriter {
public:
    FileWriter();
    
    // Python write_global_waypoints 함수와 동일한 구조로 JSON 파일 생성
    // Python 코드: write_global_waypoints(map_dir, map_info_str, est_lap_time, 
    //                                    centerline_markers, centerline_waypoints,
    //                                    global_traj_markers_iqp, global_traj_wpnts_iqp,
    //                                    global_traj_markers_sp, global_traj_wpnts_sp,
    //                                    trackbounds_markers)
    bool write_global_waypoints(
        const std::string& map_dir,
        const String& map_info_str,
        const Float32& est_lap_time,
        const WpntArray& centerline_waypoints,
        const WpntArray& global_traj_wpnts_iqp,
        const WpntArray& global_traj_wpnts_sp);
    
    // 기존 함수들 (하위 호환성 유지)
    bool save_global_waypoints(const WpntArray& waypoints, const std::string& filepath);
    
    // speed_scaling.yaml 생성 및 저장
    bool save_speed_scaling(const WpntArray& waypoints, const std::string& filepath, 
                           int n_sectors = 9, float global_limit = 0.8f);
    
private:
    // WpntArray를 JSON 형식으로 변환
    std::string wpnt_array_to_json(const WpntArray& wpnt_array, const std::string& array_name);
    
    // String 메시지를 JSON 형식으로 변환
    std::string string_to_json(const String& str, const std::string& field_name);
    
    // Float32 메시지를 JSON 형식으로 변환
    std::string float32_to_json(const Float32& f32, const std::string& field_name);
    
    // Wpnt를 JSON 형식으로 변환
    std::string wpnt_to_json(const f110_msgs::msg::Wpnt& wpnt);
    
    // 섹터 자동 분할 (속도 변화 기반)
    void calculate_sectors(const WpntArray& waypoints, int n_sectors, 
                          std::vector<int>& sector_starts, std::vector<int>& sector_ends);
    
    // 각 섹터의 평균 속도 기반 scaling 계산
    float calculate_sector_scaling(const WpntArray& waypoints, int start_idx, int end_idx);
    
    // JSON 이스케이프 처리
    std::string escape_json_string(const std::string& str);
};

#endif  // FILE_WRITER_HPP

