#include "mini_robot_driver/robot_mode_client.hpp"

#include <chrono>
#include <memory>

#include "mini_robot_interfaces/srv/set_robot_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace Mrb {

using namespace std::chrono_literals;

int run_robot_mode_client(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc != 2) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_mode_client"),
                 "Usage: ros2 run mini_robot_driver robot_mode_client <mode>");
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<rclcpp::Node>("robot_mode_client");
  auto client = node->create_client<mini_robot_interfaces::srv::SetRobotMode>(
      "/robot/set_mode");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for /robot/set_mode");
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for /robot/set_mode service...");
  }

  auto request =
      std::make_shared<mini_robot_interfaces::srv::SetRobotMode::Request>();
  request->mode = argv[1];

  RCLCPP_INFO(node->get_logger(), "Requesting robot mode: %s",
              request->mode.c_str());
  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Failed to call /robot/set_mode");
    rclcpp::shutdown();
    return 1;
  }

  const auto response = future.get();
  if (response->success) {
    RCLCPP_INFO(node->get_logger(), "Mode set successfully: %s",
                response->message.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(), "Failed to set mode: %s",
                 response->message.c_str());
  }

  rclcpp::shutdown();
  return response->success ? 0 : 1;
}

}  // namespace Mrb

int main(int argc, char* argv[]) {
  return Mrb::run_robot_mode_client(argc, argv);
}
