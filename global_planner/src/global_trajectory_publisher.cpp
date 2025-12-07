// global_trajectory_publisher: global_planner의 내부 토픽을 구독하여 /global_waypoints로 발행

#include "rclcpp/rclcpp.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

class GlobalTrajectoryPublisher : public rclcpp::Node {
public:
    GlobalTrajectoryPublisher() : Node("global_trajectory_publisher") {
        // global_planner의 내부 토픽을 구독
        subscription_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_planner/waypoints", 10,
            std::bind(&GlobalTrajectoryPublisher::waypoints_callback, this, std::placeholders::_1));
        
        // /global_waypoints로 발행
        publisher_ = this->create_publisher<f110_msgs::msg::WpntArray>(
            "/global_waypoints", 10);
    }

private:
    void waypoints_callback(const f110_msgs::msg::WpntArray::SharedPtr msg) {
        // 받은 메시지를 /global_waypoints로 발행
        publisher_->publish(*msg);
    }

    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr subscription_;
    rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}

