#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "mini_robot_interfaces/action/execute_task.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace Mrb {

class RobotControllerClient : public rclcpp::Node {
 public:
  using ExecuteTask = mini_robot_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ClientGoalHandle<ExecuteTask>;

  RobotControllerClient();
  bool send_goal(const std::string& task_name, int32_t target_steps,
                 int32_t cancel_after_step);

 private:
  void goal_response_callback(
      const GoalHandleExecuteTask::SharedPtr& goal_handle);
  void feedback_callback(
      GoalHandleExecuteTask::SharedPtr goal_handle,
      const std::shared_ptr<const ExecuteTask::Feedback> feedback);
  void result_callback(const GoalHandleExecuteTask::WrappedResult& result);

  rclcpp_action::Client<ExecuteTask>::SharedPtr action_client_;
  GoalHandleExecuteTask::SharedPtr goal_handle_;
  int32_t cancel_after_step_{0};
  bool cancel_requested_{false};
};

int run_robot_controller_client(int argc, char* argv[]);

}  // namespace Mrb
