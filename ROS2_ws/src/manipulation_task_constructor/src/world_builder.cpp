#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/color_rgba.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("world_builder");

class Setup_Builder
{
public:
  Setup_Builder(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void setupTable();
  void setupBlock(size_t block_number);

private:
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr Setup_Builder::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

Setup_Builder::Setup_Builder(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("world_builder", options) }
{
}

void Setup_Builder::setupBlock(size_t block_number)
{
  moveit_msgs::msg::CollisionObject block_object;
  block_object.header.frame_id = "world";
  std::string s = std::to_string(block_number);
  block_object.id = "block_" + s;
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.01;
  primitive.dimensions[primitive.BOX_Y] = 0.01;
  primitive.dimensions[primitive.BOX_Z] = 0.01;

  // Define the pose of the box (relative to the frame_id)
  float pos = static_cast< float >(block_number)/100;

  geometry_msgs::msg::Pose block_pose;
  block_pose.orientation.x = 0;
  block_pose.orientation.y = 0;
  block_pose.orientation.z = 0;
  block_pose.position.x = 0.25;
  block_pose.position.y = 0.15 - pos;
  block_pose.position.z = 0.205;

  block_object.primitives.push_back(primitive);
  block_object.primitive_poses.push_back(block_pose);
  block_object.operation = block_object.ADD;

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 0;
  color.g = 0;
  color.b = 160;
  color.a = 1;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(block_object, color);
}


void Setup_Builder::setupTable()
{
  moveit_msgs::msg::CollisionObject table_object;
  table_object.header.frame_id = "world";
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

  // Add color to the object
  std_msgs::msg::ColorRGBA color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = 1;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(table_object, color);
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
  size_t i;
  size_t count = 1;
  for (i = 1; i <= count; i++)
  {
    setup_builder->setupBlock(i);
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}