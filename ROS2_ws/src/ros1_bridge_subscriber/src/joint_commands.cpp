#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>

using std::placeholders::_1;

class JointValuesPublisher : public rclcpp::Node
{
public:
    JointValuesPublisher()
        : Node("joint_values_publisher")
    {
        joint_values_publisher_ = this->create_publisher<std_msgs::msg::String>("/chatter", 10);

        std::cout << "Do you want to send default values or custom values? (Enter 'default' or 'custom'): ";
        std::getline(std::cin, input_choice_);

        if (input_choice_ == "default")
        {
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&JointValuesPublisher::publishDefaultValues, this));
        }
        else if (input_choice_ == "custom")
        {
            timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&JointValuesPublisher::publishCustomValues, this));
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid input. Please enter 'default' or 'custom'.");
        }
    }

private:
    void publishDefaultValues()
    {
        auto message = std_msgs::msg::String();
        message.data = "0.04279806398200939, 0.13935736088944836, 1.7323117205860223, -6.135923395362245e-05, 1.1901640279953214, 0.0"; // Default joint values
        joint_values_publisher_->publish(message);
    }

    void publishCustomValues()
    {
        std::cout << "Enter joint values separated by commas (e.g., 1.0,2.0,3.0,4.0,5.0,6.0): ";
        std::string input;
        std::getline(std::cin, input);

        auto message = std_msgs::msg::String();
        message.data = input;
        joint_values_publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joint_values_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string input_choice_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointValuesPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

