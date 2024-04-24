#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received joint state:");
    for (size_t i = 0; i < msg->name.size(); ++i) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Joint %zu: Position=%.4f", i+1, msg->position[i]);
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ros1_bridge_subscriber");

    auto subscriber = node->create_subscription<sensor_msgs::msg::JointState>(
        "/vp6242/joint_states", 10, joint_state_callback);

    RCLCPP_INFO(node->get_logger(), "ROS 2 subscriber node initialized");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
