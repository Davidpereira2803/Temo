#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <thread>
#include <termios.h>
#include <unistd.h>

class KeyboardController : public rclcpp::Node
{
public:
    KeyboardController() : Node("keyboard_controller")
    {
        // Create a publisher for velocity commands
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/tello/cmd_vel", 10);
    }

    void run()
    {
        std::cout << "Press and hold 'w' to move forward, 's' to move backward, 'a' to move left, 'd' to move right, 'q' to quit.\n";
        rclcpp::Rate rate(10); // 10 Hz loop rate
        geometry_msgs::msg::Twist twist;

        while (rclcpp::ok())
        {
            char c = get_char();

            if (c == 'w')
            {
                twist.linear.x = 1.0; // Adjust speed as needed
                RCLCPP_INFO(this->get_logger(), "Moving forward");
            }
            else if (c == 's')
            {
                twist.linear.x = -1.0;
                RCLCPP_INFO(this->get_logger(), "Moving backward");
            }
            else if (c == 'a')
            {
                twist.linear.y = 1.0;
                RCLCPP_INFO(this->get_logger(), "Moving left");
            }
            else if (c == 'd')
            {
                twist.linear.y = -1.0;
                RCLCPP_INFO(this->get_logger(), "Moving right");
            }
            else if (c == 'q')
            {
                RCLCPP_INFO(this->get_logger(), "Stopping and exiting...");
                break;
            }
            else
            {
                // Stop movement if no valid key is pressed
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
            }

            // Publish the velocity command continuously
            velocity_publisher_->publish(twist);
            rate.sleep();
        }
    }


private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    char get_char()
    {
        struct termios oldt, newt;
        char c;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        c = getchar();
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        return c;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardController>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
