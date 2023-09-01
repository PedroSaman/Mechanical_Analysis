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

  void request_block(environment_interface::msg::Block& block);
  void setupTable();
  std::vector<std::vector<int>> read_assembly_plan();
private:
  rclcpp::Node::SharedPtr node_;
};

APV_Node::APV_Node(const rclcpp::NodeOptions &options)
    : node_{std::make_shared<rclcpp::Node>("APV_node", options)}
{
}

void APV_Node::setupTable()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = "table";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = table_x_size;
  primitive.dimensions[primitive.BOX_Y] = table_y_size;
  primitive.dimensions[primitive.BOX_Z] = table_z_size;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = table_x_position;
  pose.position.y = 0.0;
  pose.position.z = table_z_size/2;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 0.5;
  color.g = 0.5;
  color.b = 0.5;
  color.a = 1;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object, color);
}

void APV_Node::request_block(environment_interface::msg::Block& block)
{ 
  moveit_msgs::msg::CollisionObject block_object;
  block_object.header.frame_id = "table";
  block_object.id = "block_" + std::to_string(block.number);
  shape_msgs::msg::SolidPrimitive primitive;

  if(block.is_support)
  {
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = block_size_z - minimum_resolution;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = block_size/2 - minimum_resolution;
  }else{
    // Define the size of the box in meters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = block_size*block.x_size - minimum_resolution;
    primitive.dimensions[primitive.BOX_Y] = block_size*block.y_size - minimum_resolution;
    primitive.dimensions[primitive.BOX_Z] = block_size_z - minimum_resolution;

    // Define the pose of the box (relative to the frame_id)
  }
  geometry_msgs::msg::Pose block_pose;
  block_pose.orientation.x = 0;
  block_pose.orientation.y = 0;
  block_pose.orientation.z = 0;
  block_pose.position.x = -table_x_size/2 + block_size*((block.x + BASE_CORRECTION_VALUE) + block.x_size/2);
  block_pose.position.y = -table_y_size/2 + block_size*((block.y + BASE_CORRECTION_VALUE) + block.y_size/2);
  block_pose.position.z = table_z_size/2 + block_size_z/2 + block_size_z*(block.z + BASE_CORRECTION_VALUE);
  
  block_object.primitives.push_back(primitive);
  block_object.primitive_poses.push_back(block_pose);
  block_object.operation = block_object.ADD;
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(block_object, block.color);

  return;
}

std_msgs::msg::ColorRGBA getColor(int color_index)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("block_service_client");  
  rclcpp::Client<environment_interface::srv::GetBlockColor>::SharedPtr client =                
    node->create_client<environment_interface::srv::GetBlockColor>("get_block_color_service");          

  std_msgs::msg::ColorRGBA color;
  auto request = std::make_shared<environment_interface::srv::GetBlockColor::Request>();       
  
  request->index = color_index;
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      color.r = 1;
      color.g = 1;
      color.b = 1;
      color.a = 1;
      return color;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_block");    
  }
  return result.get()->color;
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
  int assembly_size = assembly_plan.size();
  
  //      0      1 2 3   4     5     6       7           8        9      10     11 
  //AssemblyArea,X,Y,Z,SizeX,SizeY,SizeZ,ColorIndex,IsSupport,CanPress,ShiftX,ShiftY
  
  //Create the table
  visualizer_node->setupTable();

  
  for (int i = 0; i < assembly_size; i++)
  {
    environment_interface::msg::Block block;
    std::stringstream result;
    std::copy(assembly_plan[i].begin(), assembly_plan[i].end(), std::ostream_iterator<int>(result, " "));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Block %d information:: %s", i+1 ,result.str().c_str());
    
    block.x = assembly_plan[i][1] + 1;
    block.y = assembly_plan[i][2] + 1;
    block.z = assembly_plan[i][3];
    if(assembly_plan[i][8] == 1)
    {
      block.color = getColor(SUPPORT);
      block.is_support = 1;
    }else 
    {
      block.color = getColor(assembly_plan[i][7]);
    }
    block.number = i+1;

    block.x_size = assembly_plan[i][4]; // Correct the sizes for the assembly
    block.y_size = assembly_plan[i][5];
    visualizer_node->request_block(block); //Refil the block in the feeder
  }

  rclcpp::shutdown();
  return 0;
}