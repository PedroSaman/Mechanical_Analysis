#include <memory>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "pose_commander",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("pose_commander");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "cobotta_arm");

  // Set a target Pose (J6)
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0;
    msg.orientation.y = 0.999929;
    msg.orientation.z = 0;
    msg.orientation.w = -0.0119291;
    msg.position.x = 0.208928;
    msg.position.y = -0.185724;
    msg.position.z = 0.242824;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Set a target Pose (J6)
  auto const target_pose2 = []{
    geometry_msgs::msg::Pose msg2;
    msg2.orientation.x = 0;
    msg2.orientation.y = 0.999929;
    msg2.orientation.z = 0;
    msg2.orientation.w = -0.012012;
    msg2.position.x = 0.208928;
    msg2.position.y = -0.185724;
    msg2.position.z = 0;
    return msg2;
  }();
  move_group_interface.setPoseTarget(target_pose2);

  // Create a plan to that target pose
  auto const [success2, plan2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg2;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg2));
    return std::make_pair(ok, msg2);
  }();

  // Execute the plan
  if(success2) {
    move_group_interface.execute(plan2);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}