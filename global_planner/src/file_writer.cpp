//파일 저장 구현
// Python readwrite_global_waypoints.py를 참고하여 global_waypoints.json 생성 기능 구현

#include "file_writer.hpp"
#include <fstream>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <iostream>

FileWriter::FileWriter() {
    // 초기화
}

bool FileWriter::write_global_waypoints(
    const std::string& map_dir,
    const String& map_info_str,
    const Float32& est_lap_time,
    const WpntArray& centerline_waypoints,
    const WpntArray& global_traj_wpnts_iqp,
    const WpntArray& global_traj_wpnts_sp) {
    
    // Python 코드: path = os.path.join(map_dir, 'global_waypoints.json')
    std::string path = map_dir;
    if (!path.empty() && path.back() != '/') {
        path += "/";
    }
    path += "global_waypoints.json";
    
    std::cout << "[INFO] WRITE_GLOBAL_WAYPOINTS: Writing global waypoints to " << path << std::endl;
    
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ERROR] Failed to open file: " << path << std::endl;
        return false;
    }
    
    // Python 코드의 딕셔너리 구조를 JSON으로 변환
    file << "{\n";
    
    // map_info_str: {'data': '...'}
    file << "  \"map_info_str\": " << string_to_json(map_info_str, "") << ",\n";
    
    // est_lap_time: {'data': ...}
    file << "  \"est_lap_time\": " << float32_to_json(est_lap_time, "") << ",\n";
    
    // centerline_waypoints: message_to_ordereddict 결과
    file << "  \"centerline_waypoints\": " << wpnt_array_to_json(centerline_waypoints, "") << ",\n";
    
    // global_traj_wpnts_iqp: message_to_ordereddict 결과
    file << "  \"global_traj_wpnts_iqp\": " << wpnt_array_to_json(global_traj_wpnts_iqp, "") << ",\n";
    
    // global_traj_wpnts_sp: message_to_ordereddict 결과
    file << "  \"global_traj_wpnts_sp\": " << wpnt_array_to_json(global_traj_wpnts_sp, "");
    
    file << "\n}\n";
    
    file.close();
    std::cout << "[INFO] Successfully wrote global waypoints to " << path << std::endl;
    return true;
}

std::string FileWriter::string_to_json(const String& str, const std::string& field_name) {
    std::ostringstream oss;
    oss << "{\"data\": \"" << escape_json_string(str.data) << "\"}";
    return oss.str();
}

std::string FileWriter::float32_to_json(const Float32& f32, const std::string& field_name) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    oss << "{\"data\": " << f32.data << "}";
    return oss.str();
}

std::string FileWriter::wpnt_array_to_json(const WpntArray& wpnt_array, const std::string& array_name) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "    \"waypoints\": [\n";
    
    // wpnts 필드 사용 (실제 f110_msgs 구조에 맞춤)
    for (size_t i = 0; i < wpnt_array.wpnts.size(); ++i) {
        oss << "      " << wpnt_to_json(wpnt_array.wpnts[i]);
        if (i < wpnt_array.wpnts.size() - 1) {
            oss << ",";
        }
        oss << "\n";
    }
    
    oss << "    ]\n";
    oss << "  }";
    return oss.str();
}

std::string FileWriter::wpnt_to_json(const f110_msgs::msg::Wpnt& wpnt) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6);
    
    // f110_msgs의 실제 Wpnt 구조에 맞춤: x_m, y_m, psi_rad, kappa_radpm, vx_mps 등
    oss << "{\n";
    
    // s_m: 누적 거리는 계산 필요 (일단 0으로 설정, 나중에 확장 가능)
    oss << "        \"s_m\": 0.0,\n";
    
    // x_m, y_m: 실제 필드 사용
    oss << "        \"x_m\": " << wpnt.x_m << ",\n";
    oss << "        \"y_m\": " << wpnt.y_m << ",\n";
    
    // d_right, d_left: 현재 메시지에 없음 (일단 0으로 설정)
    oss << "        \"d_right\": 0.0,\n";
    oss << "        \"d_left\": 0.0,\n";
    
    // psi_rad: 실제 필드 사용
    oss << "        \"psi_rad\": " << wpnt.psi_rad << ",\n";
    
    // kappa_radpm: 실제 필드 사용
    oss << "        \"kappa_radpm\": " << wpnt.kappa_radpm << ",\n";
    
    // vx_mps: 실제 필드 사용
    oss << "        \"vx_mps\": " << wpnt.vx_mps << ",\n";
    
    // ax_mps2: 현재 메시지에 없음 (일단 0으로 설정)
    oss << "        \"ax_mps2\": 0.0";
    
    oss << "\n      }";
    
    return oss.str();
}

std::string FileWriter::escape_json_string(const std::string& str) {
    std::ostringstream oss;
    for (char c : str) {
        switch (c) {
            case '"': oss << "\\\""; break;
            case '\\': oss << "\\\\"; break;
            case '\b': oss << "\\b"; break;
            case '\f': oss << "\\f"; break;
            case '\n': oss << "\\n"; break;
            case '\r': oss << "\\r"; break;
            case '\t': oss << "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    oss << "\\u" << std::hex << std::setw(4) << std::setfill('0') 
                        << static_cast<int>(c);
                } else {
                    oss << c;
                }
                break;
        }
    }
    return oss.str();
}

bool FileWriter::save_global_waypoints(const WpntArray& waypoints, const std::string& filepath) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }
    
    // JSON 형식으로 저장
    file << "[\n";
    for (size_t i = 0; i < waypoints.wpnts.size(); ++i) {
        const auto& wpnt = waypoints.wpnts[i];
        
        // quaternion 계산 (psi_rad에서)
        float psi = wpnt.psi_rad;
        float qz = std::sin(psi / 2.0f);
        float qw = std::cos(psi / 2.0f);
        
        file << "  {\n";
        file << "    \"header\": {\n";
        file << "      \"stamp\": {\"sec\": 0, \"nanosec\": 0},\n";
        file << "      \"frame_id\": \"map\"\n";
        file << "    },\n";
        file << "    \"ns\": \"\",\n";
        file << "    \"id\": " << (1500 + i) << ",\n";
        file << "    \"type\": 2,\n";
        file << "    \"action\": 0,\n";
        file << "    \"pose\": {\n";
        file << "      \"position\": {\"x\": " << std::fixed << std::setprecision(6) << wpnt.x_m 
             << ", \"y\": " << wpnt.y_m << ", \"z\": 0.0},\n";
        file << "      \"orientation\": {\"x\": 0.0, \"y\": 0.0, \"z\": " << qz 
             << ", \"w\": " << qw << "}\n";
        file << "    },\n";
        file << "    \"scale\": {\"x\": 0.05, \"y\": 0.05, \"z\": 0.05},\n";
        file << "    \"color\": {\"r\": 0.5, \"g\": 1.0, \"b\": 0.0, \"a\": 1.0},\n";
        file << "    \"lifetime\": {\"sec\": 0, \"nanosec\": 0},\n";
        file << "    \"frame_locked\": false,\n";
        file << "    \"points\": [],\n";
        file << "    \"colors\": [],\n";
        file << "    \"texture_resource\": \"\",\n";
        file << "    \"texture\": {\"header\": {\"stamp\": {\"sec\": 0, \"nanosec\": 0}, \"frame_id\": \"\"}, \"format\": \"\", \"data\": []},\n";
        file << "    \"uv_coordinates\": [],\n";
        file << "    \"text\": \"\",\n";
        file << "    \"mesh_resource\": \"\",\n";
        file << "    \"mesh_file\": {\"filename\": \"\", \"data\": []},\n";
        file << "    \"mesh_use_embedded_materials\": false";
        
        if (i < waypoints.wpnts.size() - 1) {
            file << "},\n";
        } else {
            file << "}\n";
        }
    }
    file << "]\n";
    
    file.close();
    return true;
}

bool FileWriter::save_speed_scaling(const WpntArray& waypoints, const std::string& filepath,
                                   int n_sectors, float global_limit) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }
    
    // 섹터 계산
    std::vector<int> sector_starts, sector_ends;
    calculate_sectors(waypoints, n_sectors, sector_starts, sector_ends);
    
    // YAML 형식으로 저장
    file << "sector_tuner:\n";
    file << "  ros__parameters:\n";
    file << "    global_limit: " << global_limit << "\n";
    file << "    n_sectors: " << n_sectors << "\n";
    
    for (int i = 0; i < n_sectors; ++i) {
        float scaling = calculate_sector_scaling(waypoints, sector_starts[i], sector_ends[i]);
        
        file << "    Sector" << i << ":\n";
        file << "      start: " << sector_starts[i] << "\n";
        file << "      end: " << sector_ends[i] << "\n";
        file << "      scaling: " << std::fixed << std::setprecision(2) << scaling << "\n";
        file << "      only_FTG: false\n";
        file << "      no_FTG: false\n";
    }
    
    file.close();
    return true;
}

void FileWriter::calculate_sectors(const WpntArray& waypoints, int n_sectors,
                                   std::vector<int>& sector_starts, std::vector<int>& sector_ends) {
    if (waypoints.wpnts.empty()) {
        return;
    }
    
    int total_points = waypoints.wpnts.size();
    int points_per_sector = total_points / n_sectors;
    
    sector_starts.clear();
    sector_ends.clear();
    sector_starts.reserve(n_sectors);
    sector_ends.reserve(n_sectors);
    
    for (int i = 0; i < n_sectors; ++i) {
        int start = i * points_per_sector;
        int end = (i == n_sectors - 1) ? total_points - 1 : (i + 1) * points_per_sector - 1;
        
        sector_starts.push_back(start);
        sector_ends.push_back(end);
    }
}

float FileWriter::calculate_sector_scaling(const WpntArray& waypoints, int start_idx, int end_idx) {
    if (start_idx < 0 || end_idx >= static_cast<int>(waypoints.wpnts.size()) || start_idx > end_idx) {
        return 0.8f; // 기본값
    }
    
    // 섹터 내 평균 속도 계산
    float sum_velocity = 0.0f;
    int count = 0;
    
    for (int i = start_idx; i <= end_idx; ++i) {
        sum_velocity += waypoints.wpnts[i].vx_mps;
        count++;
    }
    
    if (count == 0) {
        return 0.8f;
    }
    
    float avg_velocity = sum_velocity / count;
    
    // 속도에 기반한 scaling 계산 (0.5 ~ 1.0 범위)
    // 최대 속도 20 m/s 기준으로 정규화
    float max_velocity = 20.0f;
    float scaling = std::max(0.5f, std::min(1.0f, avg_velocity / max_velocity));
    
    // 곡률도 고려 (곡률이 크면 scaling 감소)
    float sum_curvature = 0.0f;
    for (int i = start_idx; i <= end_idx; ++i) {
        sum_curvature += std::abs(waypoints.wpnts[i].kappa_radpm);
    }
    float avg_curvature = sum_curvature / count;
    
    // 곡률이 클수록 scaling 감소
    if (avg_curvature > 0.1f) {
        scaling *= (1.0f - std::min(0.3f, avg_curvature * 2.0f));
    }
    
    return std::max(0.5f, std::min(1.0f, scaling));
}

