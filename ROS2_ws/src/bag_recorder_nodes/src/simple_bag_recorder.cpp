#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <time.h>

using std::placeholders::_1;

class SimpleBagRecorder : public rclcpp::Node
{
public:
  SimpleBagRecorder()
  : Node("simple_bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    time_t     now = time(0);
    struct tm  tstruct;
    char       file_path[80];
    tstruct = *localtime(&now);
    strftime(file_path, sizeof(file_path), "rosbag_files/rosbag2_%Y_%m_%d_%H_%M_%S", &tstruct);
    
    writer_->open(file_path);

    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&SimpleBagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();
    writer_->write(msg, "/joint_states", "sensor_msgs/msg/JointState", time_stamp);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleBagRecorder>());
  rclcpp::shutdown();
  return 0;
}