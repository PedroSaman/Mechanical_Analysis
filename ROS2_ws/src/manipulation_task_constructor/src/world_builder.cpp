#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_service.hpp"
#include <chrono>
using namespace std::chrono_literals;

#define FRAME_ID "dispenser"
#define block_size 0.3
#define table_x_size 30.0
#define table_y_size 20.0
#define table_z_size 25.0
#define base_x_size 6.0
#define base_y_size 6.0
#define base_z_size 0.2
#define dispenser_x_size 2.4
#define dispenser_y_size 12.0
#define dispenser_z_size 0.2
#define minimum_resolution 0.01
std::list<std::string> Blocks_List = {"82","42","41","32","31","22","21","11"};

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
    block.color.r = 0;
    block.color.g = 0;
    block.color.b = 160;
    block.color.a = 1;
    iterator = iterator + block.y_size + 1;
    setupBlock(block);
  }
}

void Setup_Builder::setupBlock(environment_interface::msg::Block block)
{
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("block_service_client");  
  rclcpp::Client<environment_interface::srv::BlockService>::SharedPtr client =                
    node->create_client<environment_interface::srv::BlockService>("block_service");          

  auto request = std::make_shared<environment_interface::srv::BlockService::Request>();       
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
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Res: %ld", result.get()->output);
  } else {
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
  color.r = 255;
  color.g = 255;
  color.b = 255;
  color.a = 1;

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
  color.r = 160;
  color.g = 160;
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
  pose.position.y = 23.5;
  pose.position.z = table_z_size/2;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
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