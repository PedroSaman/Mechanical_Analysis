#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_create.hpp"
#include "environment_information.h"
#include <moveit/robot_model/robot_model.h>
#include <geometric_shapes/shape_operations.h>
#include <chrono>
//std::list<std::string> Blocks_List = {"28","24","14","23","13","22","12","11"};
//std::list<std::string> Blocks_List = {"82","42","41","32","31","22","21","11"};
//std::list<std::string> Blocks_List = {"22","21","11"};
//std::list<size_t> Color_List = {WHITE,RED,ORANGE,YELLOW,GREEN,BLUE,BLACK,SUPPORT};
using namespace std::chrono_literals;
using namespace moveit_msgs::msg;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("world_builder");

class Setup_Builder
{
public:
  Setup_Builder(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void setupWorld();
  void setupBlocks();//std::list<std::string> List);

private:
  rclcpp::Node::SharedPtr node_;
  std::pair<CollisionObject, ObjectColor> setupTable();
  std::pair<CollisionObject, ObjectColor> setupBase();
  std::pair<CollisionObject, ObjectColor> setupBar();
  std::pair<CollisionObject, ObjectColor> setupDispenser1();
  std::pair<CollisionObject, ObjectColor> setupDispenser2();
  std::pair<CollisionObject, ObjectColor> setupDispenser3();
  std::pair<CollisionObject, ObjectColor> setupDispenser4();
  std::pair<CollisionObject, ObjectColor> setupDispenser5();
  std::pair<CollisionObject, ObjectColor> setupDispenser6();
  std::pair<CollisionObject, ObjectColor> setupDispenser7();
  std::pair<CollisionObject, ObjectColor> setupDispenser8();
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

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupTable()
{
  CollisionObject object;
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
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 97.0/255;
  rgba_color.g = 54.0/255;
  rgba_color.b = 19.0/255;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object, object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupBase()
{
  CollisionObject object;
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
  pose.position.x = -0.94;//measured in the real setup
  pose.position.y = 0;//measured in the real setup
  pose.position.z = table_z_size/2 + base_z_size/2 + minimum_resolution;

  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 0.11372549019;
  rgba_color.g = 0.65098039215;
  rgba_color.b = 0.93333333333 ;
  rgba_color.a = 0.4;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupBar()
{
  CollisionObject object;
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
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 0;
  rgba_color.g = 0;
  rgba_color.b = 0;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser1()
{
  CollisionObject object;
  object.header.frame_id = "bar";
  object.id = "dispenser2by1";
  Eigen::Vector3d scale(100,100,100);

  // Create the object from the mesh file
  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/2x1_lanes_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  object_pose.position.x = -4.2; //The parts feeder origin is not in its geometric center. I do not know why. Looked this using meshlab an selecting to draw axes in world coordinates.
  object_pose.position.y = 1.1; //Measured from the setup
  object_pose.position.z = bar_z_size/2 + minimum_resolution;

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser2()
{
  CollisionObject object;
  object.header.frame_id = "dispenser2by1";
  object.id = "dispenser2by2";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/2x2_lanes_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  object_pose.position.y = 5.72 + minimum_resolution; //In regard to the central 2x1 parts feeder

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser3()
{
  CollisionObject object;
  object.header.frame_id = "dispenser2by1";
  object.id = "dispenser1by1";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/1x1_lanes_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  object_pose.position.x = 0.40; //In regard to the central 2x1 parts feeder
  object_pose.position.y = - 4.63 - minimum_resolution; //In regard to the central 2x1 parts feeder

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser4()
{
  CollisionObject object;
  object.header.frame_id = "world";
  object.id = "dispenser8by2";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/8x2_lane_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  //in regard to world
  object_pose.position.x = 35.14;
  object_pose.position.y = 11.79;
  object_pose.position.z = 21.34;

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser5()
{
  CollisionObject object;
  object.header.frame_id = "world";
  object.id = "dispenser3by1";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/3x2_lane_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  //in regard to world
  object_pose.position.x = 35.14;
  object_pose.position.y = -9.04;
  object_pose.position.z = 21.34;

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser6()
{
  CollisionObject object;
  object.header.frame_id = "world";
  object.id = "dispenser3by2";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/3x2_lane_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  //in regard to world
  object_pose.position.x = 35.14;
  object_pose.position.y = -7.38;
  object_pose.position.z = 21.34;

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser7()
{
  CollisionObject object;
  object.header.frame_id = "world";
  object.id = "dispenser4by1";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/4x2_lane_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  //in regard to world
  object_pose.position.x = 35.14;
  object_pose.position.y = 16.60;
  object_pose.position.z = 21.34;

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

std::pair<CollisionObject, ObjectColor> Setup_Builder::setupDispenser8()
{
  CollisionObject object;
  object.header.frame_id = "world";
  object.id = "dispenser4by2";
  Eigen::Vector3d scale(100,100,100);

  shapes::Mesh * m = shapes::createMeshFromResource("package://environment_builder/meshes/4x2_lane_base.dae", scale);
  shape_msgs::msg::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m,object_mesh_msg);
  object_mesh = boost::get<shape_msgs::msg::Mesh>(object_mesh_msg);

  // The pose for the object relative to frame_id
  geometry_msgs::msg::Pose object_pose;
  object_pose.orientation.w = 1;
  object_pose.orientation.x = 0;
  object_pose.orientation.y = 0;
  object_pose.orientation.z = 0;
  //in regard to world
  object_pose.position.x = 35.14;
  object_pose.position.y = 14.60;
  object_pose.position.z = 21.34;

  object.meshes.push_back(object_mesh);
  object.mesh_poses.push_back(object_pose);
  object.operation = object.ADD;

  // Add color to the object
  ObjectColor object_color;
  std_msgs::msg::ColorRGBA rgba_color;
  rgba_color.r = 1;
  rgba_color.g = 1;
  rgba_color.b = 1;
  rgba_color.a = 1;
  object_color.color = rgba_color;

  return std::make_pair(object,object_color);
}

void Setup_Builder::setupBlocks()//std::list<std::string> List)
{

  environment_interface::msg::Block block;
  block.frame_id = "dispenser2by1";
  block.name = "21";
  block.x_size = 2;
  block.y_size = 1;
  block.x = 0.1 + block_size_x*block.x_size/2;
  block.y = 0;
  block.z = 0.65 + block_size_x/2;
  block.number = 0;
  block.color.r = 0.5;
  block.color.g = 0.5;
  block.color.b = 0.5;
  block.color.a = 1;
  setupBlock(block);

  
  environment_interface::msg::Block block1;
  block1.frame_id = "dispenser2by2";
  block1.name = "22";
  block1.x_size = 2;
  block1.y_size = 2;
  block1.x = 0.1 + block_size_x*block1.x_size/2;
  block1.y = 0;
  block1.z = 0.65 + block_size_x/2;
  block1.number = 0;
  block1.color.r = 0.5;
  block1.color.g = 0.5;
  block1.color.b = 0.5;
  block1.color.a = 1;
  setupBlock(block1);

  environment_interface::msg::Block block2;
  block2.frame_id = "dispenser1by1";
  block2.name = "11";
  block2.x_size = 1;
  block2.y_size = 1;
  block2.x = 0.1 + block_size_x*block2.x_size/2;
  block2.y = 0.3;
  block2.z = 0.65 + block_size_x/2;
  block2.number = 0;
  block2.color.r = 0.5;
  block2.color.g = 0.5;
  block2.color.b = 0.5;
  block2.color.a = 1;
  setupBlock(block2);

  environment_interface::msg::Block block3;
  block3.frame_id = "dispenser8by2";
  block3.name = "28";
  block3.x_size = 2;
  block3.y_size = 8;
  block3.x = 0.52;
  block3.y = 0;
  block3.z = 0.34 + block_size_x/2;
  block3.number = 0;
  block3.color.r = 0.5;
  block3.color.g = 0.5;
  block3.color.b = 0.5;
  block3.color.a = 1;
  setupBlock(block3);

  environment_interface::msg::Block block4;
  block4.frame_id = "dispenser4by2";
  block4.name = "24";
  block4.x_size = 2;
  block4.y_size = 4;
  block4.x = 0.52;
  block4.y = 0;
  block4.z = 0.34 + block_size_x/2;
  block4.number = 0;
  block4.color.r = 0.5;
  block4.color.g = 0.5;
  block4.color.b = 0.5;
  block4.color.a = 1;
  setupBlock(block4);

  environment_interface::msg::Block block5;
  block5.frame_id = "dispenser4by1";
  block5.name = "14";
  block5.x_size = 1;
  block5.y_size = 4;
  block5.x = 0.52;
  block5.y = 0;
  block5.z = 0.34 + block_size_x/2;
  block5.number = 0;
  block5.color.r = 0.5;
  block5.color.g = 0.5;
  block5.color.b = 0.5;
  block5.color.a = 1;
  setupBlock(block5);

  environment_interface::msg::Block block6;
  block6.frame_id = "dispenser3by1";
  block6.name = "13";
  block6.x_size = 1;
  block6.y_size = 3;
  block6.x = 0.52;
  block6.y = 0;
  block6.z = 0.34 + block_size_x/2;
  block6.number = 0;
  block6.color.r = 0.5;
  block6.color.g = 0.5;
  block6.color.b = 0.5;
  block6.color.a = 1;
  setupBlock(block6);

  environment_interface::msg::Block block7;
  block7.frame_id = "dispenser3by2";
  block7.name = "23";
  block7.x_size = 2;
  block7.y_size = 3;
  block7.x = 0.52;
  block7.y = 0;
  block7.z = 0.34 + block_size_x/2;
  block7.number = 0;
  block7.color.r = 0.5;
  block7.color.g = 0.5;
  block7.color.b = 0.5;
  block7.color.a = 1;
  setupBlock(block7);
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

void Setup_Builder::setupWorld()
{
  std::vector<CollisionObject> collision_objects;
  std::vector<ObjectColor> object_colors;
  std::pair<CollisionObject, ObjectColor> object;

  object = setupTable();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupBase();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupBar();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser1();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser2();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser3();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser4();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser5();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser6();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser7();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  object = setupDispenser8();
  collision_objects.insert(collision_objects.end(),object.first);
  object_colors.insert(object_colors.end(),object.second);

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObjects(collision_objects,object_colors);
  sleep(1);
  return;
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
    //executor.spin();
    executor.remove_node(setup_builder->getNodeBaseInterface());
  });
  
  setup_builder->setupWorld();
  setup_builder->setupBlocks();//Blocks_List);
  
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}