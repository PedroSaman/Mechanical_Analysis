#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

class JointTrajectoryPublisher : public rclcpp::Node 
{
public:
    JointTrajectoryPublisher() : Node("joint_trajectory_publisher")
    {
        publisher_ = this->create_publisher<control_msgs::action::FollowJointTrajectory_Goal>(
            "/denso_bcap_controller/follow_joint_trajectory/goal", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&JointTrajectoryPublisher::publish_joint_trajectory, this));
    }

private:
    void publish_joint_trajectory()
    {
        auto msg = std::make_shared<control_msgs::action::FollowJointTrajectory_Goal>();
        msg->trajectory = trajectory_msgs::msg::JointTrajectory();
        msg->trajectory.header.stamp = this->now();
        msg->trajectory.header.frame_id = "base_link";
        msg->trajectory.joint_names.push_back("joint1");

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.push_back(0.1);  // Initial joint position
        msg->trajectory.points.push_back(point);

        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Published joint trajectory command");
    }

    rclcpp::Publisher<control_msgs::action::FollowJointTrajectory_Goal>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto joint_trajectory_publisher = std::make_shared<JointTrajectoryPublisher>();
    rclcpp::spin(joint_trajectory_publisher);
    rclcpp::shutdown();
    return 0;
}

