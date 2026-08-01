#include <memory>

#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotMonitorNode : public rclcpp::Node {
 public:
  RobotMonitorNode() : Node("robot_monitor_node") {
    status_subscription_ =
        create_subscription<mini_robot_interfaces::msg::RobotStatus>(
            "robot/status", 10,
            [this](const mini_robot_interfaces::msg::RobotStatus& message) {
              monitor_battery(message.battery);
            });
  }

 private:
  void monitor_battery(float battery) const {
    if (battery >= 30.0F) {
      RCLCPP_INFO(get_logger(), "Battery: %.1f%%, status: NORMAL", battery);
    } else if (battery >= 15.0F) {
      RCLCPP_WARN(get_logger(), "Battery: %.1f%%, status: WARNING", battery);
    } else {
      RCLCPP_ERROR(get_logger(), "Battery: %.1f%%, status: ERROR", battery);
    }
  }

  rclcpp::Subscription<mini_robot_interfaces::msg::RobotStatus>::SharedPtr
      status_subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
