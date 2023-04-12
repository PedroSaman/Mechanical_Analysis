#include "rclcpp/rclcpp.hpp"
#include "environment_interface/srv/block_create.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 7) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: create_block_client X Y Z x_size y_size block_number");
      return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("create_block_client");  // CHANGE
  rclcpp::Client<environment_interface::srv::BlockCreate>::SharedPtr client =                // CHANGE
    node->create_client<environment_interface::srv::BlockCreate>("create_block");          // CHANGE

  auto request = std::make_shared<environment_interface::srv::BlockCreate::Request>();       // CHANGE
  environment_interface::msg::Block block;
  block.frame_id = "dispenser";
  block.name = "test";
  block.x_size = atoll(argv[4]);
  block.y_size = atoll(argv[5]);
  block.x = atoll(argv[1]);
  block.y = atoll(argv[2]);
  block.z = atoll(argv[3]);
  block.number = atoll(argv[6]);
  block.color.r = 0;
  block.color.g = 0;
  block.color.b = 160;
  block.color.a = 1;
  request->block = block;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
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
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service create_block");    // CHANGE
  }

  rclcpp::shutdown();
  return 0;
}