#pragma once

#include <cstdint>
#include <memory>

#include "mini_robot_interfaces/action/execute_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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

  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr execution_timer_;
  std::shared_ptr<GoalHandleExecuteTask> active_goal_handle_;
  int32_t current_step_{0};
  bool goal_active_{false};
};

}  // namespace Mrb
