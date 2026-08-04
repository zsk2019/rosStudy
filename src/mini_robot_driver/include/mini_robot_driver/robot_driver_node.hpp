#pragma once

#include <memory>
#include <string>
#include <vector>

#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "mini_robot_interfaces/srv/set_robot_mode.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

namespace Mrb {

class RobotDriverNode : public rclcpp::Node {
 public:
  RobotDriverNode();

 private:
  void set_robot_mode(
      mini_robot_interfaces::srv::SetRobotMode::Request::SharedPtr request,
      mini_robot_interfaces::srv::SetRobotMode::Response::SharedPtr response);
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter>& parameters);
  void update_publish_timer();
  void print_robot_status();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
  rclcpp::Publisher<mini_robot_interfaces::msg::RobotStatus>::SharedPtr
      status_publisher_;
  rclcpp::Service<mini_robot_interfaces::srv::SetRobotMode>::SharedPtr
      mode_service_;
  std::string robot_id_;
  std::string mode_;
  double publish_frequency_;
  double initial_battery_;
  double battery_;
  double battery_consumption_rate_;
  bool emergency_stop_;
  double max_linear_velocity_;
};

}  // namespace Mrb
