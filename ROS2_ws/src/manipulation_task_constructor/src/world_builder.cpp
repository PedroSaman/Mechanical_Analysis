#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_create.hpp"
#include "environment_information.h"
//std::list<std::string> Blocks_List = {"28","24","14","23","13","22","12","11"};
std::list<std::string> Blocks_List = {"82","42","41","32","31","22","21","11"};
#include <chrono>
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("world_builder");

class Setup_Builder
{
public:
  Setup_Builder(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void setupTable();
  void setupBase();
  void setupDispenser();
  void setupBlocks(std::list<std::string> List);

private:
  rclcpp::Node::SharedPtr node_;
  void setupBlock(environment_interface::msg::Block block);
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Setup_Builder::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

Setup_Builder::Setup_Builder(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("world_builder", options) }
{
}

void Setup_Builder::setupBlocks(std::list<std::string> List)
{
  std::list<std::string>::iterator it;
  size_t iterator = 0;

  for (it = List.begin(); it != List.end(); it++)
  {
    environment_interface::msg::Block block;
    block.frame_id = "dispenser";
    block.name = it->c_str();
    block.x_size = std::stoi(it->c_str())/10;
    block.y_size = std::stoi(it->c_str()) - block.x_size*10;
    block.x = 1;
    block.y = 1 + 1.5*iterator;
    block.z = 0;
    block.number = 0;
    block.color.r = 0.25;
    block.color.g = 0.25;
    block.color.b = 0.25;
    block.color.a = 1;
    iterator = iterator + block.y_size + 1;
    setupBlock(block);
  }
}

void Setup_Builder::setupBlock(environment_interface::msg::Block block)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("block_service_client");  
  rclcpp::Client<environment_interface::srv::BlockCreate>::SharedPtr client =                
    node->create_client<environment_interface::srv::BlockCreate>("add_block_service");          

  auto request = std::make_shared<environment_interface::srv::BlockCreate::Request>();       
  request->block = block;
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_block");    
  }
}

void Setup_Builder::setupBase()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "table";
  object.id = "base";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = base_x_size;
  primitive.dimensions[primitive.BOX_Y] = base_y_size;
  primitive.dimensions[primitive.BOX_Z] = base_z_size;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = 0.0;
  pose.position.y = -table_y_size/2 + base_y_size/2 + table_y_size*0.05;
  pose.position.z = table_z_size/2 + base_z_size/2 + minimum_resolution;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 0.85;
  color.g = 0.85;
  color.b = 0.85;
  color.a = 0;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object, color);
}

void Setup_Builder::setupDispenser()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "base";
  object.id = "dispenser";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = dispenser_x_size;
  primitive.dimensions[primitive.BOX_Y] = dispenser_y_size;
  primitive.dimensions[primitive.BOX_Z] = dispenser_z_size;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = base_x_size/2 + dispenser_x_size/2 + minimum_resolution;
  pose.position.y = -base_y_size/2 + dispenser_y_size/2;
  pose.position.z = dispenser_z_size/2 -base_z_size/2;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 16;
  color.g = 16;
  color.b = 0;
  color.a = 1;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object, color);
}

void Setup_Builder::setupTable()
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
  pose.position.x = 0.0;
  pose.position.y = table_y_position;
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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto setup_builder = std::make_shared<Setup_Builder>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &setup_builder]() {
    executor.add_node(setup_builder->getNodeBaseInterface());
    executor.remove_node(setup_builder->getNodeBaseInterface());
  });

  setup_builder->setupTable();
  setup_builder->setupBase();
  setup_builder->setupDispenser();
  setup_builder->setupBlocks(Blocks_List);
  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}