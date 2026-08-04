#include "mini_robot_driver/robot_controller_node.hpp"

#include <chrono>
#include <functional>

namespace Mrb {

using namespace std::chrono_literals;

RobotControllerNode::RobotControllerNode() : Node("robot_controller_node") {
  using std::placeholders::_1;
  using std::placeholders::_2;

  action_server_ = rclcpp_action::create_server<ExecuteTask>(
      this, "/robot/execute_task",
      std::bind(&RobotControllerNode::handle_goal, this, _1, _2),
      std::bind(&RobotControllerNode::handle_cancel, this, _1),
      std::bind(&RobotControllerNode::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(), "Action server /robot/execute_task is ready");
}

rclcpp_action::GoalResponse RobotControllerNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const ExecuteTask::Goal> goal) {
  if (goal_active_) {
    RCLCPP_WARN(get_logger(),
                "Rejecting task because another task is still active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  const bool valid_task =
      goal->task_name == ExecuteTask::Goal::TASK_MOVE_TO_POINT ||
      goal->task_name == ExecuteTask::Goal::TASK_PATROL ||
      goal->task_name == ExecuteTask::Goal::TASK_RETURN_HOME;

  if (!valid_task) {
    RCLCPP_WARN(get_logger(), "Rejecting unsupported task: %s",
                goal->task_name.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (goal->target_steps <= 0) {
    RCLCPP_WARN(get_logger(), "Rejecting task with invalid target_steps: %d",
                goal->target_steps);
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Accepted task %s with %d steps",
              goal->task_name.c_str(), goal->target_steps);
  goal_active_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControllerNode::handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTask>) {
  RCLCPP_INFO(get_logger(), "Received task cancellation request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControllerNode::handle_accepted(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle) {
  active_goal_handle_ = goal_handle;
  current_step_ = 0;
  execution_timer_ =
      create_wall_timer(1s, [this]() { execute_step(); });
}

void RobotControllerNode::execute_step() {
  if (!active_goal_handle_) {
    execution_timer_->cancel();
    goal_active_ = false;
    return;
  }

  const auto goal = active_goal_handle_->get_goal();
  auto feedback = std::make_shared<ExecuteTask::Feedback>();
  auto result = std::make_shared<ExecuteTask::Result>();

  if (active_goal_handle_->is_canceling()) {
    result->success = false;
    result->message =
        "Task canceled at step " + std::to_string(current_step_);
    active_goal_handle_->canceled(result);
    execution_timer_->cancel();
    active_goal_handle_.reset();
    goal_active_ = false;
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    return;
  }

  ++current_step_;
  feedback->current_step = current_step_;
  feedback->progress = 100.0F * static_cast<float>(current_step_) /
                       static_cast<float>(goal->target_steps);
  feedback->current_state =
      "Executing " + goal->task_name + " (step " +
      std::to_string(current_step_) + "/" +
      std::to_string(goal->target_steps) + ")";
  active_goal_handle_->publish_feedback(feedback);

  RCLCPP_INFO(get_logger(), "%s, progress: %.1f%%",
              feedback->current_state.c_str(), feedback->progress);

  if (current_step_ >= goal->target_steps) {
    result->success = true;
    result->message = "Task " + goal->task_name + " completed";
    active_goal_handle_->succeed(result);
    execution_timer_->cancel();
    active_goal_handle_.reset();
    goal_active_ = false;
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
  }
}

}  // namespace Mrb

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mrb::RobotControllerNode>());
  rclcpp::shutdown();
  return 0;
}
