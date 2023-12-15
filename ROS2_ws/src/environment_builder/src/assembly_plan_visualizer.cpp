#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_create.hpp"
#include "environment_interface/srv/block_remove.hpp"
#include "environment_interface/srv/get_block_color.hpp"
#include "environment_information.h"
#include <chrono>
#include <string>
#include <fstream>
#include <iostream>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("assembly_plan_visualizer");

class APV_Node
{
public:
  
  APV_Node(const rclcpp::NodeOptions &options);
  moveit_msgs::msg::CollisionObject request_block(environment_interface::msg::Block& block);
  std::vector<std::vector<int>> read_assembly_plan();
private:
  rclcpp::Node::SharedPtr node_;
};

APV_Node::APV_Node(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("APV_node", options)}
{
}

moveit_msgs::msg::CollisionObject APV_Node::request_block(environment_interface::msg::Block& block)
{ 
  moveit_msgs::msg::CollisionObject block_object;
  block_object.header.frame_id = "base";
  block_object.id = "block_" + std::to_string(block.number);
  shape_msgs::msg::SolidPrimitive primitive;

  if(block.is_support)
  {
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = block_size_z - minimum_resolution;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = block_size_x/2 - minimum_resolution;
  }else{
    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = block_size_x*block.x_size - minimum_resolution;
    primitive.dimensions[primitive.BOX_Y] = block_size_x*block.y_size - minimum_resolution;
    primitive.dimensions[primitive.BOX_Z] = block_size_z - minimum_resolution;

    // Define the pose of the box (relative to the frame_id)
  }
  geometry_msgs::msg::Pose block_pose;
  block_pose.orientation.x = 0;
  block_pose.orientation.y = 0;
  block_pose.orientation.z = 0;
  block_pose.position.x = -base_x_size/2 + block_size_x*((block.x + BASE_CORRECTION_VALUE) + block.x_size/2);
  block_pose.position.y = -base_y_size/2 + block_size_x*((block.y + BASE_CORRECTION_VALUE) + block.y_size/2);
  block_pose.position.z = base_z_size/2 + block_size_z/2 + block_size_z*(block.z + BASE_CORRECTION_VALUE);
  
  block_object.primitives.push_back(primitive);
  block_object.primitive_poses.push_back(block_pose);
  block_object.operation = block_object.ADD;

  return block_object;
}

std_msgs::msg::ColorRGBA getColor(int color_index)
{
  std_msgs::msg::ColorRGBA block_color;
  std::string color_name;
  switch (color_index)
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
  return block_color;
}

std::vector<std::vector<int>> APV_Node::read_assembly_plan()
{
  std::string myFilePath = node_->get_parameter("csv_file_path").as_string();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "parameter name: %s", myFilePath.c_str());
  std::ifstream assemblyfile;
  assemblyfile.open(myFilePath);
  
  std::vector<std::vector<int>> assembly_plan;
  if(assemblyfile.fail())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Csv file not found.");
    return assembly_plan;
  }
  int assembly_size = 0;
  while(assemblyfile.peek()!=EOF)
  {
    std::string records;
    assembly_plan.push_back(std::vector<int>());
    if (getline (assemblyfile, records)) {            /* read line of input into line */
        int itmp;                       /* temporary integer to fill */
        std::stringstream ss (records);    /* create stringstream from line */
        
        while (ss >> itmp) {            /* read integer value from ss */
            std::string stmp {};        /* temporary string to hold delim */
            assembly_plan[assembly_size].push_back(itmp);          /* add to vector */
            getline (ss, stmp, ',');  /* read delimiter */
        }
        assembly_size++;
    }
  }
  assemblyfile.close();
  return assembly_plan;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto visualizer_node = std::make_shared<APV_Node>(options);

  std::vector<std::vector<int>> assembly_plan = visualizer_node->read_assembly_plan();
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::vector<moveit_msgs::msg::ObjectColor> object_colors;
  int assembly_size = assembly_plan.size();
  
  //      0      1 2 3   4     5     6       7           8        9      10     11 
  //AssemblyArea,X,Y,Z,SizeX,SizeY,SizeZ,ColorIndex,IsSupport,CanPress,ShiftX,ShiftY

  for (int i = 0; i < assembly_size; i++)
  {
    environment_interface::msg::Block block;
    moveit_msgs::msg::ObjectColor object_color;
    //std::stringstream result;
    //std::copy(assembly_plan[i].begin(), assembly_plan[i].end(), std::ostream_iterator<int>(result, " "));

    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Block %d information:: %s", i+1 ,result.str().c_str());
    
    block.x = assembly_plan[i][1] + 1;
    block.y = assembly_plan[i][2] + 1;
    block.z = assembly_plan[i][3];
    if(assembly_plan[i][8] == 1)
    {
      object_color.color = getColor(SUPPORT);
      block.is_support = 1;
    }else 
    {
      object_color.color = getColor(assembly_plan[i][7]);
    }
    block.number = i+1;

    block.x_size = assembly_plan[i][4]; // Correct the sizes for the assembly
    block.y_size = assembly_plan[i][5];
    collision_objects.insert(collision_objects.end(),visualizer_node->request_block(block));
    object_colors.insert(object_colors.end(),object_color);
  }

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(collision_objects,object_colors);
  rclcpp::shutdown();
  return 0;
}