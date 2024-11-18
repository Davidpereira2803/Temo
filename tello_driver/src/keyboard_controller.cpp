#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class KeyboardController : public rclcpp::Node
{
public:
    KeyboardController() : Node("simple_movement")
    {
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/tello/cmd_vel", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 10 Hz
            std::bind(&KeyboardController::publish_forward_movement, this));
        RCLCPP_INFO(this->get_logger(), "Starting simple continuous forward movement test...");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publish_forward_movement()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = 1.0;  // Forward movement
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Publishing forward movement command");
        velocity_publisher_->publish(twist);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
