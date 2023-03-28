#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/color_rgba.hpp>
#include "environment_interface/srv/block_create.hpp"
#include "environment_interface/srv/block_remove.hpp"
#include "environment_interface/msg/block.hpp"
#include "environment_information.h"
#include <memory>

void remove_block(const std::shared_ptr<environment_interface::srv::BlockRemove::Request> request,
          std::shared_ptr<environment_interface::srv::BlockRemove::Response>       response)
{
  moveit_msgs::msg::CollisionObject block_object;
  block_object.id = request->block.name;
  block_object.operation = block_object.REMOVE;
  
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(block_object);
  
  response->output = request->block.number;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming remove block request \n\n\nBlock name: %s" , request->block.name.c_str()); 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: removed block");
}

void add_block(const std::shared_ptr<environment_interface::srv::BlockCreate::Request> request,
          std::shared_ptr<environment_interface::srv::BlockCreate::Response>       response)
{
  moveit_msgs::msg::CollisionObject block_object;
  block_object.header.frame_id = request->block.frame_id;
  block_object.id = request->block.name;
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = block_size*request->block.x_size - minimum_resolution;
  primitive.dimensions[primitive.BOX_Y] = block_size*request->block.y_size - minimum_resolution;
  primitive.dimensions[primitive.BOX_Z] = block_size - minimum_resolution;

  // Define the pose of the box (relative to the frame_id)

  geometry_msgs::msg::Pose block_pose;
  block_pose.orientation.x = 0;
  block_pose.orientation.y = 0;
  block_pose.orientation.z = 0;
  block_pose.position.x = -dispenser_x_size/2 + block_size*(request->block.x_size/2 + (request->block.x - 1));
  block_pose.position.y = -dispenser_y_size/2 + block_size*(request->block.y_size/2 + (request->block.y - 1));
  block_pose.position.z = dispenser_z_size/2 + block_size/2 + request->block.z;

  block_object.primitives.push_back(primitive);
  block_object.primitive_poses.push_back(block_pose);
  block_object.operation = block_object.ADD;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(block_object, request->block.color);
  
  response->output = request->block.number;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming add_block request\n\n\nx: %.2f" " y: %.2f" " z: %.2f" " x_size: %.0f" " y_size: %.0f",
                request->block.x, request->block.y, request->block.z, request->block.x_size, request->block.y_size);                                       
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: created block number [%ld]", (long int)response->output);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("block_service_server");

  //Look here if I can create many services inside this node like kohama did
  rclcpp::Service<environment_interface::srv::BlockCreate>::SharedPtr add_service =
    node->create_service<environment_interface::srv::BlockCreate>("add_block_service",  &add_block);
  
  rclcpp::Service<environment_interface::srv::BlockRemove>::SharedPtr remove_service =
    node->create_service<environment_interface::srv::BlockRemove>("remove_block_service",  &remove_block);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to do block services.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}