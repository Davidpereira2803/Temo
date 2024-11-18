// ~/ros2_ws/src/tello_ros/tello_driver/src/keyboard_controller.cpp
#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include <iostream>
#include <thread>
#include <termios.h>
#include <unistd.h>

class KeyboardController : public rclcpp::Node
{
public:
    KeyboardController() : Node("keyboard_controller")
    {
        client_ = this->create_client<tello_msgs::srv::TelloAction>("/tello_action");
        // Wait until the service is available
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for /tello_action service...");
        }
    }

    void send_command(const std::string &command)
    {
        auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
        request->cmd = command;

        auto result = client_->async_send_request(request);
        // Wait for the result
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = result.get();
            if (response->rc == response->OK)
            {
                RCLCPP_INFO(this->get_logger(), "Command executed successfully: %s", command.c_str());
            }
            else if (response->rc == response->ERROR_NOT_CONNECTED)
            {
                RCLCPP_WARN(this->get_logger(), "Error: Drone not connected.");
            }
            else if (response->rc == response->ERROR_BUSY)
            {
                RCLCPP_WARN(this->get_logger(), "Error: Drone is busy.");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unexpected response code: %d", response->rc);
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send command: %s", command.c_str());
        }
    }

    void run()
    {
        std::cout << "Press 1 to take off, 2 to land, 5 to flip forward. Press 'q' to quit.\n 3 for up and 4 for down";
        char c;
        while (true)
        {
            c = get_char();
            if (c == '1')
            {
                RCLCPP_INFO(this->get_logger(), "Takeoff command");
                send_command("takeoff");
            }
            else if (c == '2')
            {
                RCLCPP_INFO(this->get_logger(), "Land command");
                send_command("land");
            }
            else if (c == '5')555
            {
                RCLCPP_INFO(this->get_logger(), "Flip forward command");
                send_command("flip f");
                //send_command("flip b");
                //send_command("flip l");
                //send_command("flip r");
            }
            else if (c == '3')
            {
                RCLCPP_INFO(this->get_logger(), "up 50 command");
                send_command("up 50");
            }
            else if (c == '4')
            {
                RCLCPP_INFO(this->get_logger(), "down 50 command");
                send_command("down 50");
            }
            else if (c == 'e')
            {
                RCLCPP_INFO(this->get_logger(), "cw 90 command");
                send_command("cw 90");
            }
            else if (c == 'q')
            {
                RCLCPP_INFO(this->get_logger(), "ccw 90 command");
                send_command("ccw 90");
            }
            else if (c == 'd')
            {
                RCLCPP_INFO(this->get_logger(), "right 100 command");
                send_command("right 100");
            }
            else if (c == 'a')
            {
                RCLCPP_INFO(this->get_logger(), "left 100 command");
                send_command("left 100");
            }
            else if (c == 's')
            {
                RCLCPP_INFO(this->get_logger(), "back 100 command");
                send_command("back 100");
            }
            else if (c == 'w')
            {
                RCLCPP_INFO(this->get_logger(), "forward 100 command");
                send_command("forward 100");
            }
            else if (c == 'b')
            {
                RCLCPP_INFO(this->get_logger(), "Battery command");
                send_command("battery?");
            }
            else if (c == '0')
            {
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                break;
            }
        }
    }

private:
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr client_;

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
