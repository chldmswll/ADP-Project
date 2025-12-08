#include "global_planner/global_planner_logic.hpp"
#include "global_planner/readwrite_global_waypoints.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner()
  : Node("global_planner_node")
  {
    // Declare parameters (allow undeclared to accept from launch file)
    this->declare_parameter("rate", rclcpp::ParameterValue(10.0));
    this->declare_parameter<std::string>("map_name", "map");
    this->declare_parameter<bool>("create_map", false);
    this->declare_parameter<bool>("map_editor", false);
    this->declare_parameter<bool>("reverse_mapping", false);
    this->declare_parameter<double>("safety_width", 0.3);
    this->declare_parameter<double>("safety_width_sp", 0.2);
    this->declare_parameter<double>("occupancy_grid_threshold", 0.65);
    this->declare_parameter<int>("filter_kernel_size", 3);
    this->declare_parameter<bool>("show_plots", false);
    this->declare_parameter<int>("required_laps", 1);

    // Get rate parameter - handle both int and double from YAML
    double rate = 10.0;
    if (this->has_parameter("rate")) {
      auto rate_param = this->get_parameter("rate");
      if (rate_param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        rate = rate_param.as_double();
      } else if (rate_param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
        rate = static_cast<double>(rate_param.as_int());
      }
    }
    std::string map_name = this->get_parameter("map_name").as_string();
    bool create_map = this->get_parameter("create_map").as_bool();
    bool map_editor = this->get_parameter("map_editor").as_bool();
    bool reverse_mapping = this->get_parameter("reverse_mapping").as_bool();
    double safety_width = this->get_parameter("safety_width").as_double();
    double safety_width_sp = this->get_parameter("safety_width_sp").as_double();
    double occupancy_grid_threshold = this->get_parameter("occupancy_grid_threshold").as_double();
    int filter_kernel_size = this->get_parameter("filter_kernel_size").as_int();
    bool show_plots = this->get_parameter("show_plots").as_bool();
    int required_laps = this->get_parameter("required_laps").as_int();

    // Get map source path
    std::string map_dir = "/home/misys/forza_ws/race_stack/stack_master/maps/" + map_name;
    try {
      std::string share_dir = ament_index_cpp::get_package_share_directory("stack_master");
      std::filesystem::path path(share_dir);
      // share_dir is typically: install/stack_master/share/stack_master
      // We want to go to: src/race_stack/stack_master/maps/map_name
      // So we go: install -> .. -> src/race_stack/stack_master/maps
      path = path.parent_path().parent_path() / "src/race_stack/stack_master/maps" / map_name;
      if (std::filesystem::exists(path)) {
        map_dir = path.string();
      } else {
        // Try alternative: assume maps are in share directory
        path = std::filesystem::path(share_dir).parent_path().parent_path().parent_path() / "stack_master/maps" / map_name;
        if (std::filesystem::exists(path)) {
          map_dir = path.string();
        }
      }
    } catch (...) {
      RCLCPP_WARN(this->get_logger(), "Could not get package share directory, using default path");
    }

    // Create logic instance
    logic_ = std::make_unique<global_planner::GlobalPlannerLogic>(
      safety_width,
      safety_width_sp,
      occupancy_grid_threshold,
      map_editor,
      create_map,
      map_name,
      map_dir,
      "",  // finish_script_path
      "",  // input_path
      show_plots,
      filter_kernel_size,
      required_laps,
      reverse_mapping,
      [this](const std::string & msg) { RCLCPP_INFO(this->get_logger(), "%s", msg.c_str()); },
      [this](const std::string & msg) { RCLCPP_WARN(this->get_logger(), "%s", msg.c_str()); },
      [this](const std::string & msg) { RCLCPP_ERROR(this->get_logger(), "%s", msg.c_str()); }
    );

    // Subscribers
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&GlobalPlanner::map_cb, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/car_state/pose", 10, std::bind(&GlobalPlanner::pose_cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Waiting for map and car pose...");

    // Publishers
    global_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints", 10);
    centerline_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/centerline_waypoints", 10);
    global_waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/global_waypoints/markers", 10);
    centerline_waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/centerline_waypoints/markers", 10);
    track_bounds_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/trackbounds/markers", 10);
    shortest_path_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>(
      "/global_waypoints/shortest_path", 10);
    shortest_path_waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/global_waypoints/shortest_path/markers", 10);
    map_infos_pub_ = this->create_publisher<std_msgs::msg::String>("/map_infos", 10);
    est_lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/estimated_lap_time", 10);

    // Main loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
      std::bind(&GlobalPlanner::global_plan_callback, this));

    map_name_ = map_name;
    map_editor_ = map_editor;
    create_map_ = create_map;
  }

private:
  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    logic_->update_map(*msg);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    logic_->update_pose(*msg);
  }

  void global_plan_callback()
  {
    try {
      auto [success, map_name] = logic_->global_plan_logic();
      if (success) {
        RCLCPP_WARN(this->get_logger(), "Global planner succeeded: map_name=%s", map_name.c_str());
        read_and_publish(map_editor_, create_map_);
        RCLCPP_INFO(this->get_logger(), "Killing global planner...");
        rclcpp::shutdown();
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      RCLCPP_WARN(this->get_logger(), "Killing global planner...");
      rclcpp::shutdown();
    }
  }

  void read_and_publish(bool map_editor_mode, bool create_map)
  {
    if (map_editor_mode && create_map) {
      RCLCPP_WARN(this->get_logger(),
        "In map_editor_mapping mode. Waypoints are not calculated, and thus not published.");
      return;
    }

    try {
      std::string map_dir = "/home/misys/forza_ws/race_stack/stack_master/maps/" + map_name_;
      try {
        std::string share_dir = ament_index_cpp::get_package_share_directory("stack_master");
        std::filesystem::path path(share_dir);
        // share_dir is typically: install/stack_master/share/stack_master
        // We want to go to: src/race_stack/stack_master/maps/map_name
        path = path.parent_path().parent_path() / "src/race_stack/stack_master/maps" / map_name_;
        if (std::filesystem::exists(path)) {
          map_dir = path.string();
        } else {
          // Try alternative: assume maps are in share directory
          path = std::filesystem::path(share_dir).parent_path().parent_path().parent_path() / "stack_master/maps" / map_name_;
          if (std::filesystem::exists(path)) {
            map_dir = path.string();
          }
        }
      } catch (...) {
        // Use default path
      }

      auto [map_infos, est_lap_time, centerline_markers, centerline_wpnts,
            glb_markers, glb_wpnts, glb_sp_markers, glb_sp_wpnts, track_bounds] =
        global_planner::read_global_waypoints(map_dir);

      global_waypoints_pub_->publish(glb_wpnts);
      centerline_waypoints_pub_->publish(centerline_wpnts);
      centerline_waypoints_markers_pub_->publish(centerline_markers);
      global_waypoints_markers_pub_->publish(glb_markers);
      track_bounds_pub_->publish(track_bounds);
      shortest_path_waypoints_pub_->publish(glb_sp_wpnts);
      shortest_path_waypoints_markers_pub_->publish(glb_sp_markers);
      map_infos_pub_->publish(map_infos);
      est_lap_time_pub_->publish(est_lap_time);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "%s. Not republishing waypoints.", e.what());
    }
  }

  std::unique_ptr<global_planner::GlobalPlannerLogic> logic_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr global_waypoints_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr centerline_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr global_waypoints_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_waypoints_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_bounds_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr shortest_path_waypoints_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr shortest_path_waypoints_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_infos_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr est_lap_time_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string map_name_;
  bool map_editor_;
  bool create_map_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
