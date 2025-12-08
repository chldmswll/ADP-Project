#include "global_planner/readwrite_global_waypoints.hpp"
#include <rclcpp/rclcpp.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>

class GlobalRepublisher : public rclcpp::Node
{
public:
  GlobalRepublisher()
  : Node("global_republisher_node")
  {
    this->declare_parameter<std::string>("map_path", "");

    // Subscribers
    glb_wpnts_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
      "/global_waypoints", 10,
      std::bind(&GlobalRepublisher::glb_wpnts_cb, this, std::placeholders::_1));
    glb_sp_wpnts_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
      "/global_waypoints/shortest_path", 10,
      std::bind(&GlobalRepublisher::glb_sp_wpnts_cb, this, std::placeholders::_1));
    centerline_wpnt_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
      "/centerline_waypoints", 10,
      std::bind(&GlobalRepublisher::centerline_wpnt_cb, this, std::placeholders::_1));
    map_info_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/map_infos", 10,
      std::bind(&GlobalRepublisher::map_info_cb, this, std::placeholders::_1));

    // Publishers
    glb_wpnts_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("global_waypoints", 10);
    glb_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "global_waypoints/markers", 10);
    vis_track_bnds_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "trackbounds/markers", 10);
    glb_sp_wpnts_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>(
      "global_waypoints/shortest_path", 10);
    glb_sp_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "global_waypoints/shortest_path/markers", 10);
    centerline_wpnts_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>(
      "/centerline_waypoints", 10);
    centerline_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/centerline_waypoints/markers", 10);
    map_info_pub_ = this->create_publisher<std_msgs::msg::String>("map_infos", 10);

    // Read info from JSON file if provided
    std::string map_path = this->get_parameter("map_path").as_string();
    if (!map_path.empty()) {
      RCLCPP_INFO(this->get_logger(), "Reading parameters from %s", map_path.c_str());
      try {
        auto [map_infos, est_lap_time, centerline_markers, centerline_wpnts,
              glb_markers, glb_wpnts, glb_sp_markers, glb_sp_wpnts, track_bounds] =
          global_planner::read_global_waypoints(map_path);

        map_infos_ = std::make_shared<std_msgs::msg::String>(map_infos);
        est_lap_time_ = std::make_shared<std_msgs::msg::Float32>(est_lap_time);
        centerline_markers_ = std::make_shared<visualization_msgs::msg::MarkerArray>(centerline_markers);
        centerline_wpnts_ = std::make_shared<f110_msgs::msg::WpntArray>(centerline_wpnts);
        glb_markers_ = std::make_shared<visualization_msgs::msg::MarkerArray>(glb_markers);
        glb_wpnts_ = std::make_shared<f110_msgs::msg::WpntArray>(glb_wpnts);
        glb_sp_markers_ = std::make_shared<visualization_msgs::msg::MarkerArray>(glb_sp_markers);
        glb_sp_wpnts_ = std::make_shared<f110_msgs::msg::WpntArray>(glb_sp_wpnts);
        track_bounds_ = std::make_shared<visualization_msgs::msg::MarkerArray>(track_bounds);

        // Publish markers once
        pub_marker_once();
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "%s param not found. Not publishing", map_path.c_str());
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "global_trajectory_publisher did not find any map_path param");
    }

    // Publish at 2 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GlobalRepublisher::global_republisher, this));
  }

private:
  void glb_wpnts_cb(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    glb_wpnts_ = msg;
    if (!msg->wpnts.empty()) {
      double track_length = msg->wpnts.back().s_m;
      this->declare_parameter("track_length", track_length);
      this->set_parameter(rclcpp::Parameter("track_length", track_length));
    }
  }

  void glb_sp_wpnts_cb(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    glb_sp_wpnts_ = msg;
  }

  void centerline_wpnt_cb(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    centerline_wpnts_ = msg;
  }

  void map_info_cb(const std_msgs::msg::String::SharedPtr msg)
  {
    map_infos_ = msg;
  }

  void global_republisher()
  {
    if (glb_wpnts_) {
      glb_wpnts_pub_->publish(*glb_wpnts_);
    }

    if (glb_sp_wpnts_) {
      glb_sp_wpnts_pub_->publish(*glb_sp_wpnts_);
    }

    if (centerline_wpnts_) {
      centerline_wpnts_pub_->publish(*centerline_wpnts_);
    }
  }

  void pub_marker_once()
  {
    if (glb_markers_) {
      glb_markers_pub_->publish(*glb_markers_);
    }

    if (glb_sp_markers_) {
      glb_sp_markers_pub_->publish(*glb_sp_markers_);
    }

    if (centerline_markers_) {
      centerline_markers_pub_->publish(*centerline_markers_);
    }

    if (track_bounds_) {
      vis_track_bnds_pub_->publish(*track_bounds_);
    }
  }

  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr glb_wpnts_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr glb_sp_wpnts_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr centerline_wpnt_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_info_sub_;

  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr glb_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr glb_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_track_bnds_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr glb_sp_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr glb_sp_markers_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr centerline_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_info_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  f110_msgs::msg::WpntArray::SharedPtr glb_wpnts_;
  f110_msgs::msg::WpntArray::SharedPtr glb_sp_wpnts_;
  f110_msgs::msg::WpntArray::SharedPtr centerline_wpnts_;
  visualization_msgs::msg::MarkerArray::SharedPtr glb_markers_;
  visualization_msgs::msg::MarkerArray::SharedPtr glb_sp_markers_;
  visualization_msgs::msg::MarkerArray::SharedPtr centerline_markers_;
  visualization_msgs::msg::MarkerArray::SharedPtr track_bounds_;
  std_msgs::msg::String::SharedPtr map_infos_;
  std_msgs::msg::Float32::SharedPtr est_lap_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GlobalRepublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
