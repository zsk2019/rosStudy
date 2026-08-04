#include "mini_robot_driver/robot_controller_client.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>

namespace Mrb {

using namespace std::chrono_literals;

RobotControllerClient::RobotControllerClient()
    : Node("robot_controller_client") {
  action_client_ = rclcpp_action::create_client<ExecuteTask>(
      this, "/robot/execute_task");
}

bool RobotControllerClient::send_goal(const std::string& task_name,
                                      int32_t target_steps,
                                      int32_t cancel_after_step) {
  if (!action_client_->wait_for_action_server(5s)) {
    RCLCPP_ERROR(get_logger(),
                 "Action server /robot/execute_task is not available");
    return false;
  }

  ExecuteTask::Goal goal;
  goal.task_name = task_name;
  goal.target_steps = target_steps;
  cancel_after_step_ = cancel_after_step;

  rclcpp_action::Client<ExecuteTask>::SendGoalOptions options;
  options.goal_response_callback =
      std::bind(&RobotControllerClient::goal_response_callback, this,
                std::placeholders::_1);
  options.feedback_callback =
      std::bind(&RobotControllerClient::feedback_callback, this,
                std::placeholders::_1, std::placeholders::_2);
  options.result_callback =
      std::bind(&RobotControllerClient::result_callback, this,
                std::placeholders::_1);

  RCLCPP_INFO(get_logger(), "Sending task %s with %d steps",
              task_name.c_str(), target_steps);
  action_client_->async_send_goal(goal, options);
  return true;
}

void RobotControllerClient::goal_response_callback(
    const GoalHandleExecuteTask::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by the action server");
    rclcpp::shutdown();
    return;
  }

  goal_handle_ = goal_handle;
  RCLCPP_INFO(get_logger(), "Goal accepted by the action server");
}

void RobotControllerClient::feedback_callback(
    GoalHandleExecuteTask::SharedPtr goal_handle,
    const std::shared_ptr<const ExecuteTask::Feedback> feedback) {
  RCLCPP_INFO(get_logger(), "Feedback: step=%d, progress=%.1f%%, state=%s",
              feedback->current_step, feedback->progress,
              feedback->current_state.c_str());

  if (cancel_after_step_ > 0 &&
      feedback->current_step >= cancel_after_step_ && !cancel_requested_) {
    cancel_requested_ = true;
    RCLCPP_INFO(get_logger(), "Canceling goal after step %d",
                feedback->current_step);
    action_client_->async_cancel_goal(goal_handle);
  }
}

void RobotControllerClient::result_callback(
    const GoalHandleExecuteTask::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Result: success=%s, message=%s",
                  result.result->success ? "true" : "false",
                  result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted: %s",
                   result.result->message.c_str());
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(get_logger(), "Goal was canceled: %s",
                  result.result->message.c_str());
      break;
    default:
      RCLCPP_ERROR(get_logger(), "Received an unknown result code");
      break;
  }
  rclcpp::shutdown();
}

int run_robot_controller_client(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  if (argc < 3 || argc > 4) {
    RCLCPP_ERROR(
        rclcpp::get_logger("robot_controller_client"),
        "Usage: ros2 run mini_robot_driver robot_controller_client "
        "<task_name> <target_steps> [cancel_after_step]");
    rclcpp::shutdown();
    return 1;
  }

  int32_t target_steps;
  int32_t cancel_after_step = 0;
  try {
    target_steps = std::stoi(argv[2]);
    if (argc == 4) {
      cancel_after_step = std::stoi(argv[3]);
    }
  } catch (const std::exception&) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_controller_client"),
                 "Steps must be valid integers");
    rclcpp::shutdown();
    return 1;
  }

  if (target_steps <= 0 || cancel_after_step < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("robot_controller_client"),
                 "target_steps must be positive and cancel_after_step must "
                 "not be negative");
    rclcpp::shutdown();
    return 1;
  }

  auto node = std::make_shared<RobotControllerClient>();
  if (!node->send_goal(argv[1], target_steps, cancel_after_step)) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  return 0;
}

}  // namespace Mrb

int main(int argc, char* argv[]) {
  return Mrb::run_robot_controller_client(argc, argv);
}
