#include <rclcpp/rclcpp.hpp>
#include <string>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "environment_interface/msg/block.hpp"
#include "environment_interface/srv/block_remove.hpp"
#include "environment_information.h"
#include <chrono>
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("world_builder");

class Setup_Builder
{
public:
  Setup_Builder(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void cleanBlocks(std::string object_name);

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

void Setup_Builder::cleanBlocks(std::string object_name)
{
 std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("remove_block_client");
  rclcpp::Client<environment_interface::srv::BlockRemove>::SharedPtr client =              
    node->create_client<environment_interface::srv::BlockRemove>("remove_block_service");        

  auto request = std::make_shared<environment_interface::srv::BlockRemove::Request>();
  environment_interface::msg::Block object;
  object.name = object_name;
  request->block = object;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "block services not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_block");  
  }
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

    moveit::planning_interface::PlanningSceneInterface psi;
    std::map<std::string, moveit_msgs::msg::CollisionObject> object_name_map;
    object_name_map = psi.getObjects();
    std::map<std::string, moveit_msgs::msg::CollisionObject>::iterator it = object_name_map.begin();
    std::string block_name_ref = "block_";

    while(it != object_name_map.end())
    {
        std::string object_name = it->first;
        if(object_name.length() > block_name_ref.length() && object_name != "dispenser")
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Removing object: %s", object_name.c_str());
            setup_builder->cleanBlocks(object_name);
        }
        it++;
    }

    spin_thread->join();
    rclcpp::shutdown();
    return 0;
}