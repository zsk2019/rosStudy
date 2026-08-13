#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "mini_robot_interfaces/action/execute_task.hpp"
#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "mini_robot_interfaces/srv/set_robot_mode.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_state_machine.hpp"
#include "std_msgs/msg/float32.hpp"

namespace Mrb {

class RobotControllerNode : public rclcpp::Node {
 public:
  using ExecuteTask = mini_robot_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  RobotControllerNode();

 private:
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecuteTask::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  void handle_accepted(
      const std::shared_ptr<GoalHandleExecuteTask> goal_handle);
  void execute_step();
  void set_robot_mode(
      mini_robot_interfaces::srv::SetRobotMode::Request::SharedPtr request,
      mini_robot_interfaces::srv::SetRobotMode::Response::SharedPtr response);
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter>& parameters);
  void update_status_publish_timer();
  void publish_robot_status();

  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr execution_timer_;
  rclcpp::TimerBase::SharedPtr status_publish_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
  rclcpp::Publisher<mini_robot_interfaces::msg::RobotStatus>::SharedPtr
      status_publisher_;
  rclcpp::Service<mini_robot_interfaces::srv::SetRobotMode>::SharedPtr
      mode_service_;
  std::shared_ptr<GoalHandleExecuteTask> active_goal_handle_;
  std::string robot_id_;
  std::string mode_;
  double publish_frequency_;
  double initial_battery_;
  double battery_;
  double battery_consumption_rate_;
  bool emergency_stop_;
  double max_linear_velocity_;
  int32_t current_step_{0};
  bool goal_active_{false};
  RobotStateMachine state_machine_;
};

}  // namespace Mrb
