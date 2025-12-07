// global_trajectory_publisher: global_planner의 내부 토픽을 구독하여 외부 토픽으로 republish

#include "rclcpp/rclcpp.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

class GlobalTrajectoryPublisher : public rclcpp::Node {
public:
    GlobalTrajectoryPublisher() : Node("global_trajectory_publisher") {
        RCLCPP_INFO(this->get_logger(), "Global trajectory publisher starting...");
        
        // global_planner의 내부 토픽을 구독
        waypoints_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_planner/waypoints", 10,
            std::bind(&GlobalTrajectoryPublisher::waypoints_callback, this, std::placeholders::_1));
        
        shortest_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_planner/shortest_path", 10,
            std::bind(&GlobalTrajectoryPublisher::shortest_path_callback, this, std::placeholders::_1));
        
        waypoints_markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/global_planner/waypoints/markers", 10,
            std::bind(&GlobalTrajectoryPublisher::waypoints_markers_callback, this, std::placeholders::_1));
        
        shortest_path_markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/global_planner/shortest_path/markers", 10,
            std::bind(&GlobalTrajectoryPublisher::shortest_path_markers_callback, this, std::placeholders::_1));
        
        centerline_waypoints_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_planner/centerline_waypoints", 10,
            std::bind(&GlobalTrajectoryPublisher::centerline_waypoints_callback, this, std::placeholders::_1));
        
        centerline_waypoints_markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/global_planner/centerline_waypoints/markers", 10,
            std::bind(&GlobalTrajectoryPublisher::centerline_waypoints_markers_callback, this, std::placeholders::_1));
        
        trackbounds_markers_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "/global_planner/trackbounds/markers", 10,
            std::bind(&GlobalTrajectoryPublisher::trackbounds_markers_callback, this, std::placeholders::_1));
        
        map_infos_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/global_planner/map_infos", 10,
            std::bind(&GlobalTrajectoryPublisher::map_infos_callback, this, std::placeholders::_1));
        
        est_lap_time_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/global_planner/estimated_lap_time", 10,
            std::bind(&GlobalTrajectoryPublisher::est_lap_time_callback, this, std::placeholders::_1));
        
        // 외부 토픽으로 발행
        waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints", 10);
        shortest_path_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints/shortest_path", 10);
        centerline_waypoints_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>("/centerline_waypoints", 10);
        waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_waypoints/markers", 10);
        shortest_path_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/global_waypoints/shortest_path/markers", 10);
        centerline_waypoints_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/centerline_waypoints/markers", 10);
        trackbounds_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trackbounds/markers", 10);
        map_infos_pub_ = this->create_publisher<std_msgs::msg::String>("/map_infos", 10);
        est_lap_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("/estimated_lap_time", 10);
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/waypoints");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/shortest_path");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/centerline_waypoints");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/waypoints/markers");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/shortest_path/markers");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/centerline_waypoints/markers");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/trackbounds/markers");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/map_infos");
        RCLCPP_INFO(this->get_logger(), "Subscribed to /global_planner/estimated_lap_time");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_waypoints");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_waypoints/shortest_path");
        RCLCPP_INFO(this->get_logger(), "Publishing to /centerline_waypoints");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_waypoints/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /global_waypoints/shortest_path/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /centerline_waypoints/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /trackbounds/markers");
        RCLCPP_INFO(this->get_logger(), "Publishing to /map_infos");
        RCLCPP_INFO(this->get_logger(), "Publishing to /estimated_lap_time");
        RCLCPP_INFO(this->get_logger(), "Global trajectory publisher initialized");
    }

private:
    void waypoints_callback(const f110_msgs::msg::WpntArray::SharedPtr msg) {
        waypoints_pub_->publish(*msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Republished %zu waypoints to /global_waypoints", msg->wpnts.size());
    }
    
    void shortest_path_callback(const f110_msgs::msg::WpntArray::SharedPtr msg) {
        shortest_path_pub_->publish(*msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Republished %zu waypoints to /global_waypoints/shortest_path", msg->wpnts.size());
    }
    
    void waypoints_markers_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        waypoints_markers_pub_->publish(*msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Republished %zu markers to /global_waypoints/markers", msg->markers.size());
    }
    
    void shortest_path_markers_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        shortest_path_markers_pub_->publish(*msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Republished %zu markers to /global_waypoints/shortest_path/markers", msg->markers.size());
    }
    
    void centerline_waypoints_callback(const f110_msgs::msg::WpntArray::SharedPtr msg) {
        centerline_waypoints_pub_->publish(*msg);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Republished %zu centerline waypoints to /centerline_waypoints", msg->wpnts.size());
    }
    
    void centerline_waypoints_markers_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        centerline_waypoints_markers_pub_->publish(*msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Republished %zu centerline markers to /centerline_waypoints/markers", msg->markers.size());
    }
    
    void trackbounds_markers_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        trackbounds_markers_pub_->publish(*msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Republished %zu markers to /trackbounds/markers", msg->markers.size());
    }
    
    void map_infos_callback(const std_msgs::msg::String::SharedPtr msg) {
        map_infos_pub_->publish(*msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Republished map_infos to /map_infos");
    }
    
    void est_lap_time_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        est_lap_time_pub_->publish(*msg);
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Republished estimated_lap_time: %.2f to /estimated_lap_time", msg->data);
    }

    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr shortest_path_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr centerline_waypoints_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_markers_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr shortest_path_markers_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_waypoints_markers_sub_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr trackbounds_markers_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_infos_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr est_lap_time_sub_;
    
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr shortest_path_pub_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr centerline_waypoints_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr shortest_path_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_waypoints_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trackbounds_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_infos_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr est_lap_time_pub_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}

