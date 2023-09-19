#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/color_rgba.hpp>
#include "environment_interface/srv/block_create.hpp"
#include "environment_interface/srv/block_remove.hpp"
#include "environment_interface/srv/block_remove_all.hpp"
#include "environment_interface/srv/get_block_color.hpp"
#include "environment_interface/msg/block.hpp"
#include "environment_information.h"
#include <memory>

void get_color(const std::shared_ptr<environment_interface::srv::GetBlockColor::Request> request,
          std::shared_ptr<environment_interface::srv::GetBlockColor::Response>       response)
{
  std_msgs::msg::ColorRGBA block_color;
  std::string color_name;
  switch (request->index)
  {
  case WHITE:
    block_color.r = 1;
    block_color.g = 1;
    block_color.b = 1;
    block_color.a = 1;
    color_name = "White";
    break;
  case RED:
    block_color.r = 1;
    block_color.g = 0;
    block_color.b = 0;
    block_color.a = 1;
    color_name = "Red";
    break;
  case ORANGE:
    block_color.r = 1;
    block_color.g = 0.65;
    block_color.b = 0;
    block_color.a = 1;
    color_name = "Orange";
    break;
  case YELLOW:
    block_color.r = 1;
    block_color.g = 1;
    block_color.b = 0;
    block_color.a = 1;
    color_name = "Yellow";
    break;
  case GREEN:
    block_color.r = 0;
    block_color.g = 1;
    block_color.b = 0;
    block_color.a = 1;
    color_name = "Green";
    break;
  case BLUE:
    block_color.r = 0;
    block_color.g = 0;
    block_color.b = 1;
    block_color.a = 1;
    color_name = "Blue";
    break;
  case BLACK:
    block_color.r = 0.2;
    block_color.g = 0.2;
    block_color.b = 0.2;
    block_color.a = 1;
    color_name = "Black";
    break;
  case SUPPORT:
    block_color.r = 0.5;
    block_color.g = 0.5;
    block_color.b = 0.5;
    block_color.a = 0.1;
    color_name = "Support block color";
    break;
  default:
    block_color.r = 0.2;
    block_color.g = 0.2;
    block_color.b = 0.2;
    block_color.a = 1;
    color_name = "Non Defined";
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "COLOR NOT DEFINED");
    break;
  }
  
  response->color = block_color;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming get color request. \n"); 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: color is %s", color_name.c_str());
}

void remove_block(const std::shared_ptr<environment_interface::srv::BlockRemove::Request> request,
          std::shared_ptr<environment_interface::srv::BlockRemove::Response>       response)
{
  moveit_msgs::msg::CollisionObject block_object;
  block_object.id = request->block.name;
  block_object.operation = block_object.REMOVE;
  
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(block_object);
  
  response->output = request->block.number;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming remove block request \nBlock name: %s" , request->block.name.c_str()); 
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: removed block");
}

void remove_block_all(const std::shared_ptr<environment_interface::srv::BlockRemoveAll::Request> request,
          std::shared_ptr<environment_interface::srv::BlockRemoveAll::Response>       response)
{
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects = request->blocks;
  
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(collision_objects);
  
  response->output = 1;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming remove all block request"); 
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
  primitive.dimensions[primitive.BOX_Z] = block_size_z - minimum_resolution;

  // Define the pose of the box (relative to the frame_id)

  geometry_msgs::msg::Pose block_pose;
  block_pose.orientation.x = 0;
  block_pose.orientation.y = 0;
  block_pose.orientation.z = 0;
  block_pose.position.x = request->block.x;
  block_pose.position.y = request->block.y;
  block_pose.position.z = request->block.z;

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

  rclcpp::Service<environment_interface::srv::BlockCreate>::SharedPtr add_service =
    node->create_service<environment_interface::srv::BlockCreate>("add_block_service",  &add_block);
  
  rclcpp::Service<environment_interface::srv::BlockRemove>::SharedPtr remove_service =
    node->create_service<environment_interface::srv::BlockRemove>("remove_block_service",  &remove_block);

  rclcpp::Service<environment_interface::srv::BlockRemoveAll>::SharedPtr remove_all_service =
    node->create_service<environment_interface::srv::BlockRemoveAll>("remove_all_blocks_service",  &remove_block_all);

  rclcpp::Service<environment_interface::srv::GetBlockColor>::SharedPtr get_color_service =
    node->create_service<environment_interface::srv::GetBlockColor>("get_block_color_service",  &get_color);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to do block services.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}