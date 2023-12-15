#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_create.hpp"
#include "environment_information.h"
#include <moveit/robot_model/robot_model.h>
#include <geometric_shapes/shape_operations.h>
//std::list<std::string> Blocks_List = {"28","24","14","23","13","22","12","11"};
std::list<std::string> Blocks_List = {"82","42","41","32","31","22","21","11"};
//std::list<std::string> Blocks_List = {"22","21","11"};
std::list<size_t> Color_List = {WHITE,RED,ORANGE,YELLOW,GREEN,BLUE,BLACK,SUPPORT};
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
  void setupDispenser1();
  void setupDispenser2();
  void setupDispenser3();
  void setupDispenser4();
  void setupDispenser();
  void setupBloquer();
  void setupBar();
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
  pose.position.x = 0.5 + base_x_size/2 + dispenser_x_size/2 + minimum_resolution;
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

void Setup_Builder::setupBloquer()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "dispenser";
  object.id = "bloquer";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.3;
  primitive.dimensions[primitive.BOX_Y] = dispenser_y_size;
  primitive.dimensions[primitive.BOX_Z] = 5;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.position.x = 1 + dispenser_x_size/2;
  pose.position.y = 0;
  pose.position.z = 5/2 ;

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
    block.x = 0;
    block.y = -dispenser_y_size/2 + block_size_x + 1*iterator;
    block.z = dispenser_z_size/2 + block_size_z/2 + 5*minimum_resolution;
    block.number = 0;
    block.color.r = 0.7;
    block.color.g = 0;
    block.color.b = 1;
    block.color.a = 1;
    iterator = iterator + block.y_size/2 + 2*block_size_x;
    setupBlock(block);
  }
  
 /*
  environment_interface::msg::Block block;
  block.frame_id = "dispenser2by1";
  block.name = "21";
  block.x_size = 2;
  block.y_size = 1;
  block.x = 0.1 + block_size_x*block.x_size/2;
  block.y = 0;
  block.z = 0.51 + block_size_x/2;
  block.number = 0;
  block.color.r = 0.7;
  block.color.g = 0;
  block.color.b = 1;
  block.color.a = 1;
  setupBlock(block);

  
  environment_interface::msg::Block block1;
  block1.frame_id = "dispenser2by2";
  block1.name = "22";
  block1.x_size = 2;
  block1.y_size = 2;
  block1.x = 0.1 + block_size_x*block1.x_size/2;
  block1.y = 0;
  block1.z = 0.51 + block_size_x/2;
  block1.number = 0;
  block1.color.r = 0;
  block1.color.g = 0.7;
  block1.color.b = 1;
  block1.color.a = 1;
  setupBlock(block1);

  environment_interface::msg::Block block2;
  block2.frame_id = "dispenser1by1";
  block2.name = "11";
  block2.x_size = 1;
  block2.y_size = 1;
  block2.x = 0.1 + block_size_x*block2.x_size/2;
  block2.y = 0.3;
  block2.z = 0.51 + block_size_x/2;
  block2.number = 0;
  block2.color.r = 1;
  block2.color.g = 0;
  block2.color.b = 0.7;
  block2.color.a = 1;
  setupBlock(block2);

  environment_interface::msg::Block block3;
  block3.frame_id = "dispenser8by2";
  block3.name = "28";
  block3.x_size = 2;
  block3.y_size = 8;
  block3.x = 0.52;
  block3.y = 0;
  block3.z = 0.2 + block_size_x/2;
  block3.number = 0;
  block3.color.r = 1;
  block3.color.g = 0.7;
  block3.color.b = 0;
  block3.color.a = 1;
  setupBlock(block3);
*/
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
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0;
  pose.position.x = -2.94;//measured in the real setup
  pose.position.y = 6.26;//measured in the real setup
  pose.position.z = table_z_size/2 + base_z_size/2 + minimum_resolution;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 0.85;
  color.g = 0.85;
  color.b = 0.85;
  color.a = 1;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object, color);
}

void Setup_Builder::setupBar()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "table";
  object.id = "bar";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = bar_x_size;
  primitive.dimensions[primitive.BOX_Y] = bar_y_size;
  primitive.dimensions[primitive.BOX_Z] = bar_z_size;

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose pose;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0;
  pose.position.x = table_x_size/2 - bar_x_size/2 + 0.2;//measured in the real setup
  pose.position.y = -0.5;//measured in the real setup
  pose.position.z = table_z_size/2 + bar_z_size/2 + minimum_resolution;

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

void Setup_Builder::setupDispenser1()
{
  moveit_msgs::msg::CollisionObject collision_object2;
  collision_object2.header.frame_id = "bar";
  collision_object2.id = "dispenser2by1";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/2x1_lanes_base.dae", scale);
  shape_msgs::msg::Mesh shelf_mesh;
  shapes::ShapeMsg shelf_mesh_msg;
  shapes::constructMsgFromShape(m,shelf_mesh_msg);
  shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);
  std_msgs::msg::ColorRGBA color;
  color.r = 1;
  color.g = 1;
  color.b = 0;
  color.a = 1;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::msg::Pose shelf_pose;
  shelf_pose.orientation.w = 1;
  shelf_pose.orientation.x = 0;
  shelf_pose.orientation.y = 0;
  shelf_pose.orientation.z = 0;
  shelf_pose.position.x = -4.2; //The parts feeder origin is not in its geometric center. I do not know why. Looked this using meshlab an selecting to draw axes in world coordinates.
  shelf_pose.position.y = 1.1; //Measured from the setup
  shelf_pose.position.z = bar_z_size/2 + minimum_resolution;

  collision_object2.meshes.push_back(shelf_mesh);
  collision_object2.mesh_poses.push_back(shelf_pose);
  collision_object2.operation = collision_object2.ADD;
  // Add object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(collision_object2, color);
}

void Setup_Builder::setupDispenser2()
{
  moveit_msgs::msg::CollisionObject collision_object2;
  collision_object2.header.frame_id = "dispenser2by1";
  collision_object2.id = "dispenser2by2";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/2x2_lanes_base.dae", scale);
  shape_msgs::msg::Mesh shelf_mesh;
  shapes::ShapeMsg shelf_mesh_msg;
  shapes::constructMsgFromShape(m,shelf_mesh_msg);
  shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);
  std_msgs::msg::ColorRGBA color;
  color.r = 1;
  color.g = 0;
  color.b = 1;
  color.a = 1;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::msg::Pose shelf_pose;
  shelf_pose.orientation.w = 1;
  shelf_pose.orientation.x = 0;
  shelf_pose.orientation.y = 0;
  shelf_pose.orientation.z = 0;
  shelf_pose.position.y = 5.72 + minimum_resolution; //In regard to the central 2x1 parts feeder

  collision_object2.meshes.push_back(shelf_mesh);
  collision_object2.mesh_poses.push_back(shelf_pose);
  collision_object2.operation = collision_object2.ADD;
  // Add object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(collision_object2, color);
}

void Setup_Builder::setupDispenser3()
{
  moveit_msgs::msg::CollisionObject collision_object2;
  collision_object2.header.frame_id = "dispenser2by1";
  collision_object2.id = "dispenser1by1";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/1x1_lanes_base.dae", scale);
  shape_msgs::msg::Mesh shelf_mesh;
  shapes::ShapeMsg shelf_mesh_msg;
  shapes::constructMsgFromShape(m,shelf_mesh_msg);
  shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);
  std_msgs::msg::ColorRGBA color;
  color.r = 0;
  color.g = 1;
  color.b = 1;
  color.a = 1;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::msg::Pose shelf_pose;
  shelf_pose.orientation.w = 1;
  shelf_pose.orientation.x = 0;
  shelf_pose.orientation.y = 0;
  shelf_pose.orientation.z = 0;
  shelf_pose.position.x = 0.40; //In regard to the central 2x1 parts feeder
  shelf_pose.position.y = - 4.63 - minimum_resolution; //In regard to the central 2x1 parts feeder

  collision_object2.meshes.push_back(shelf_mesh);
  collision_object2.mesh_poses.push_back(shelf_pose);
  collision_object2.operation = collision_object2.ADD;
  // Add object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(collision_object2, color);
}

void Setup_Builder::setupDispenser4()
{
  moveit_msgs::msg::CollisionObject collision_object2;
  collision_object2.header.frame_id = "dispenser2by1";
  collision_object2.id = "dispenser8by2";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/8x2_lanes_base_test.dae", scale);
  shape_msgs::msg::Mesh shelf_mesh;
  shapes::ShapeMsg shelf_mesh_msg;
  shapes::constructMsgFromShape(m,shelf_mesh_msg);
  shelf_mesh = boost::get<shape_msgs::msg::Mesh>(shelf_mesh_msg);
  std_msgs::msg::ColorRGBA color;
  color.r = 0;
  color.g = 1;
  color.b = 0;
  color.a = 1;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::msg::Pose shelf_pose;
  shelf_pose.orientation.w = 1;
  shelf_pose.orientation.x = 0;
  shelf_pose.orientation.y = 0;
  shelf_pose.orientation.z = 0;
  shelf_pose.position.z = 0.32; //In regard to the central 2x1 parts feeder
  shelf_pose.position.y = 12.5 + minimum_resolution; //In regard to the central 2x1 parts feeder

  collision_object2.meshes.push_back(shelf_mesh);
  collision_object2.mesh_poses.push_back(shelf_pose);
  collision_object2.operation = collision_object2.ADD;
  // Add object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(collision_object2, color);
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
  setup_builder->setupBar();
  setup_builder->setupDispenser();
  //setup_builder->setupDispenser1();
  //setup_builder->setupDispenser2();
  //setup_builder->setupDispenser3();
  //setup_builder->setupDispenser4();
  setup_builder->setupBlocks(Blocks_List);
  setup_builder->setupBloquer();
  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}