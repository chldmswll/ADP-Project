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

    // Publishers (use absolute paths to match subscribers)
    glb_wpnts_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints", 10);
    glb_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/global_waypoints/markers", 10);
    vis_track_bnds_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/trackbounds/markers", 10);
    glb_sp_wpnts_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>(
      "/global_waypoints/shortest_path", 10);
    glb_sp_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/global_waypoints/shortest_path/markers", 10);
    centerline_wpnts_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>(
      "/centerline_waypoints", 10);
    centerline_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/centerline_waypoints/markers", 10);
    map_info_pub_ = this->create_publisher<std_msgs::msg::String>("/map_infos", 10);

    // Read info from JSON file if provided
    std::string map_path = this->get_parameter("map_path").as_string();
    RCLCPP_INFO(this->get_logger(), "global_trajectory_publisher started. map_path='%s'", map_path.c_str());
    if (!map_path.empty()) {
      RCLCPP_INFO(this->get_logger(), "Reading waypoints from %s", map_path.c_str());
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

        RCLCPP_INFO(this->get_logger(), 
                    "Successfully loaded waypoints: global=%zu, centerline=%zu, shortest_path=%zu",
                    glb_wpnts.wpnts.size(), centerline_wpnts.wpnts.size(), glb_sp_wpnts.wpnts.size());

        // Publish markers once immediately
        pub_marker_once();
        
        // Also publish waypoints immediately (not just wait for timer)
        if (!glb_wpnts.wpnts.empty()) {
          glb_wpnts_pub_->publish(glb_wpnts);
          RCLCPP_INFO(this->get_logger(), "Published %zu global waypoints immediately", glb_wpnts.wpnts.size());
        } else {
          RCLCPP_WARN(this->get_logger(), "Global waypoints are empty, not publishing");
        }
        if (!centerline_wpnts.wpnts.empty()) {
          centerline_wpnts_pub_->publish(centerline_wpnts);
        }
        if (!glb_sp_wpnts.wpnts.empty()) {
          glb_sp_wpnts_pub_->publish(glb_sp_wpnts);
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Failed to read waypoints from %s: %s. Will try to subscribe from topics instead.", 
                    map_path.c_str(), e.what());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "global_trajectory_publisher did not find any map_path param. Will subscribe from topics instead.");
    }
    
    RCLCPP_INFO(this->get_logger(), "global_trajectory_publisher initialization complete. Timer started at 2Hz.");

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
      // Only declare if not already declared, otherwise just set
      if (!this->has_parameter("track_length")) {
        this->declare_parameter("track_length", track_length);
      } else {
        this->set_parameter(rclcpp::Parameter("track_length", track_length));
      }
      RCLCPP_DEBUG(this->get_logger(), "Received %zu waypoints from /global_waypoints topic", msg->wpnts.size());
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
    // Republish waypoints from file or topic if available
    if (glb_wpnts_ && !glb_wpnts_->wpnts.empty()) {
      glb_wpnts_pub_->publish(*glb_wpnts_);
      RCLCPP_DEBUG(this->get_logger(), "Republished %zu global waypoints", glb_wpnts_->wpnts.size());
    }

    if (glb_sp_wpnts_ && !glb_sp_wpnts_->wpnts.empty()) {
      glb_sp_wpnts_pub_->publish(*glb_sp_wpnts_);
    }

    if (centerline_wpnts_ && !centerline_wpnts_->wpnts.empty()) {
      centerline_wpnts_pub_->publish(*centerline_wpnts_);
    }

    // Also republish markers periodically
    pub_marker_once();
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

    if (map_infos_) {
      map_info_pub_->publish(*map_infos_);
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
