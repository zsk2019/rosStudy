#pragma once

#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace Mrb {

class RobotMonitorNode : public rclcpp::Node {
 public:
  RobotMonitorNode();

 private:
  void monitor_battery(float battery) const;

  rclcpp::Subscription<mini_robot_interfaces::msg::RobotStatus>::SharedPtr
      status_subscription_;
};

}  // namespace Mrb
