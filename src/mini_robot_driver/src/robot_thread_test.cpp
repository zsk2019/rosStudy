#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "mini_robot_interfaces/srv/set_robot_mode.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RobotThreadTest : public rclcpp::Node {
 public:
  RobotThreadTest() : Node("robot_thread_test") {
    status_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    // service_callback_group_ =
    //  create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = status_callback_group_;
    status_subscription_ =
        create_subscription<mini_robot_interfaces::msg::RobotStatus>(
            "robot/status", 10,
            [this](const mini_robot_interfaces::msg::RobotStatus& message) {
              RCLCPP_INFO(
                  get_logger(),
                  "Received status: robot_id=%s, mode=%s, battery=%.1f%%",
                  message.robot_id.c_str(), message.mode.c_str(),
                  message.battery);
            },
            subscription_options);

    test_service_ = create_service<mini_robot_interfaces::srv::SetRobotMode>(
        "robot/thread_test",
        [this](
            const std::shared_ptr<
                mini_robot_interfaces::srv::SetRobotMode::Request>
                request,
            std::shared_ptr<mini_robot_interfaces::srv::SetRobotMode::Response>
                response) {
          RCLCPP_INFO(
              get_logger(),
              "Test service started (request mode: %s), waiting 2 seconds...",
              request->mode.c_str());
          std::this_thread::sleep_for(2s);
          response->success = true;
          response->message = "Test service completed after 2 seconds";
          RCLCPP_INFO(get_logger(), "Test service completed");
        },
        rclcpp::ServicesQoS(), status_callback_group_);
  }

 private:
  rclcpp::CallbackGroup::SharedPtr status_callback_group_;
  // rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::Subscription<mini_robot_interfaces::msg::RobotStatus>::SharedPtr
      status_subscription_;
  rclcpp::Service<mini_robot_interfaces::srv::SetRobotMode>::SharedPtr
      test_service_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotThreadTest>();
  // rclcpp::executors::SingleThreadedExecutor executor;
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
