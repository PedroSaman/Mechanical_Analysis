#include <memory>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

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
  move_group_interface.setMaxVelocityScalingFactor(1);

  // Create table object for the robot to avoid
  auto const table_object = [frame_id =
                                 move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject table_object;
    table_object.header.frame_id = frame_id;
    table_object.id = "table";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.5;
    primitive.dimensions[primitive.BOX_Y] = 0.5;
    primitive.dimensions[primitive.BOX_Z] = 0.2;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.x = 0;
    table_pose.orientation.y = 0;
    table_pose.orientation.z = 0;
    table_pose.position.x = 0.38;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.1;

    table_object.primitives.push_back(primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;

    return table_object;
  }();

  // Add table object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(table_object);

  // Create block object for the robot to avoid
  auto const block_object = [frame_id =
                                 move_group_interface.getPlanningFrame()] {
    moveit_msgs::msg::CollisionObject block_object;
    block_object.header.frame_id = frame_id;
    block_object.id = "block1";
    shape_msgs::msg::SolidPrimitive primitive;

    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.01;
    primitive.dimensions[primitive.BOX_Y] = 0.01;
    primitive.dimensions[primitive.BOX_Z] = 0.01;

    // Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose block_pose;
    block_pose.orientation.x = 0;
    block_pose.orientation.y = 0;
    block_pose.orientation.z = 0;
    block_pose.position.x = 0.19;
    block_pose.position.y = 0.18;
    block_pose.position.z = 0.205;

    block_object.primitives.push_back(primitive);
    block_object.primitive_poses.push_back(block_pose);
    block_object.operation = block_object.ADD;

    return block_object;
  }();

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 103;
  color.g = 0;
  color.b = 103;
  color.a = 0.61;

  // Add table object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface2;
  planning_scene_interface2.applyCollisionObject(block_object, color);

  // Set a target Pose (J6)
  auto const target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0;
    msg.orientation.y = 1;
    msg.orientation.z = 0;
    msg.orientation.w = 0;
    msg.position.x = 0.19;
    msg.position.y = 0.18;
    msg.position.z = 0.35;
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

  
  

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}