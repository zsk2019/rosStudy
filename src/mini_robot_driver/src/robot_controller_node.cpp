#include "mini_robot_driver/robot_controller_node.hpp"

#include <chrono>
#include <functional>

#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace Mrb {

using namespace std::chrono_literals;

RobotControllerNode::RobotControllerNode() : Node("robot_controller_node") {
  using std::placeholders::_1;
  using std::placeholders::_2;

  robot_id_ = declare_parameter<std::string>("robot_id", "mini_robot_01");
  mode_ = mini_robot_interfaces::msg::RobotStatus::MODE_IDLE;
  publish_frequency_ = declare_parameter<double>("publish_frequency", 10.0);

  rcl_interfaces::msg::ParameterDescriptor battery_descriptor;
  battery_descriptor.description = "Initial battery percentage";
  rcl_interfaces::msg::FloatingPointRange battery_range;
  battery_range.from_value = 0.0;
  battery_range.to_value = 100.0;
  battery_range.step = 0.0;
  battery_descriptor.floating_point_range = {battery_range};
  initial_battery_ =
      declare_parameter<double>("initial_battery", 100.0, battery_descriptor);
  battery_ = initial_battery_;
  battery_consumption_rate_ =
      declare_parameter<double>("battery_consumption_rate", 0.1);
  emergency_stop_ = declare_parameter<bool>("emergency_stop", false);

  rcl_interfaces::msg::ParameterDescriptor velocity_descriptor;
  velocity_descriptor.description = "Maximum linear velocity in m/s";
  rcl_interfaces::msg::FloatingPointRange velocity_range;
  velocity_range.from_value = 0.0;
  velocity_range.to_value = 5.0;
  velocity_range.step = 0.0;
  velocity_descriptor.floating_point_range = {velocity_range};
  max_linear_velocity_ = declare_parameter<double>("max_linear_velocity", 1.5,
                                                   velocity_descriptor);

  if (publish_frequency_ <= 0.0) {
    RCLCPP_WARN(get_logger(),
                "publish_frequency must be greater than zero; using 10.0 Hz");
    publish_frequency_ = 10.0;
  }

  battery_publisher_ =
      create_publisher<std_msgs::msg::Float32>("/robot/battery", 10);
  status_publisher_ = create_publisher<mini_robot_interfaces::msg::RobotStatus>(
      "robot/status", 10);
  mode_service_ = create_service<mini_robot_interfaces::srv::SetRobotMode>(
      "/robot/set_mode",
      [this](const mini_robot_interfaces::srv::SetRobotMode::Request::SharedPtr
                 request,
             mini_robot_interfaces::srv::SetRobotMode::Response::SharedPtr
                 response) { set_robot_mode(request, response); });
  parameter_callback_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& parameters) {
        return on_parameters_changed(parameters);
      });
  update_status_publish_timer();

  action_server_ = rclcpp_action::create_server<ExecuteTask>(
      this, "/robot/execute_task",
      std::bind(&RobotControllerNode::handle_goal, this, _1, _2),
      std::bind(&RobotControllerNode::handle_cancel, this, _1),
      std::bind(&RobotControllerNode::handle_accepted, this, _1));

  RCLCPP_INFO(get_logger(),
              "robot_id: %s, publish_frequency: %.1f Hz, "
              "initial_battery: %.1f%%, battery_consumption_rate: %.1f, "
              "max_linear_velocity: %.1f m/s",
              robot_id_.c_str(), publish_frequency_, battery_,
              battery_consumption_rate_, max_linear_velocity_);
  RCLCPP_INFO(get_logger(), "Service /robot/set_mode is ready");
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
  execution_timer_ = create_wall_timer(1s, [this]() { execute_step(); });
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
    result->message = "Task canceled at step " + std::to_string(current_step_);
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
  feedback->current_state = "Executing " + goal->task_name + " (step " +
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

void RobotControllerNode::set_robot_mode(
    const mini_robot_interfaces::srv::SetRobotMode::Request::SharedPtr request,
    mini_robot_interfaces::srv::SetRobotMode::Response::SharedPtr response) {
  const auto& requested_mode = request->mode;
  const bool valid_mode =
      requested_mode == mini_robot_interfaces::msg::RobotStatus::MODE_IDLE ||
      requested_mode == mini_robot_interfaces::msg::RobotStatus::MODE_RUNNING ||
      requested_mode ==
          mini_robot_interfaces::msg::RobotStatus::MODE_CHARGING ||
      requested_mode == mini_robot_interfaces::msg::RobotStatus::MODE_ERROR ||
      requested_mode == mini_robot_interfaces::msg::RobotStatus::MODE_AUTO ||
      requested_mode == mini_robot_interfaces::msg::RobotStatus::MODE_EMERGENCY;

  if (!valid_mode) {
    response->success = false;
    response->message =
        "Invalid mode: " + requested_mode +
        ". Allowed modes: IDLE, RUNNING, CHARGING, ERROR, AUTO, EMERGENCY";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (emergency_stop_ &&
      requested_mode !=
          mini_robot_interfaces::msg::RobotStatus::MODE_EMERGENCY) {
    response->success = false;
    response->message =
        "Emergency stop is active; only EMERGENCY mode is allowed";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  if (battery_ < 15.0 &&
      requested_mode == mini_robot_interfaces::msg::RobotStatus::MODE_AUTO) {
    response->success = false;
    response->message = "Battery is below 15%; AUTO mode is not allowed";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return;
  }

  mode_ = requested_mode;
  response->success = true;
  response->message = "Robot mode set to " + mode_;
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

rcl_interfaces::msg::SetParametersResult
RobotControllerNode::on_parameters_changed(
    const std::vector<rclcpp::Parameter>& parameters) {
  auto robot_id = robot_id_;
  auto publish_frequency = publish_frequency_;
  auto initial_battery = initial_battery_;
  auto battery_consumption_rate = battery_consumption_rate_;
  auto emergency_stop = emergency_stop_;
  auto max_linear_velocity = max_linear_velocity_;
  bool reset_battery = false;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;

  for (const auto& parameter : parameters) {
    const auto& name = parameter.get_name();
    if (name == "robot_id") {
      robot_id = parameter.as_string();
      if (robot_id.empty()) {
        result.reason = "robot_id must not be empty";
        return result;
      }
    } else if (name == "publish_frequency") {
      publish_frequency = parameter.as_double();
      if (publish_frequency <= 0.0) {
        result.reason = "publish_frequency must be greater than zero";
        return result;
      }
    } else if (name == "initial_battery") {
      initial_battery = parameter.as_double();
      if (initial_battery < 0.0 || initial_battery > 100.0) {
        result.reason = "initial_battery must be between 0.0 and 100.0";
        return result;
      }
      reset_battery = true;
    } else if (name == "battery_consumption_rate") {
      battery_consumption_rate = parameter.as_double();
      if (battery_consumption_rate < 0.0) {
        result.reason = "battery_consumption_rate must not be negative";
        return result;
      }
    } else if (name == "emergency_stop") {
      emergency_stop = parameter.as_bool();
    } else if (name == "max_linear_velocity") {
      max_linear_velocity = parameter.as_double();
      if (max_linear_velocity < 0.0 || max_linear_velocity > 5.0) {
        result.reason = "max_linear_velocity must be between 0.0 and 5.0";
        return result;
      }
    }
  }

  const bool frequency_changed = publish_frequency != publish_frequency_;
  robot_id_ = robot_id;
  publish_frequency_ = publish_frequency;
  initial_battery_ = initial_battery;
  battery_consumption_rate_ = battery_consumption_rate;
  emergency_stop_ = emergency_stop;
  max_linear_velocity_ = max_linear_velocity;
  if (reset_battery) {
    battery_ = initial_battery_;
  }
  if (frequency_changed) {
    update_status_publish_timer();
  }

  result.successful = true;
  result.reason = "parameters updated successfully";
  return result;
}

void RobotControllerNode::update_status_publish_timer() {
  if (status_publish_timer_) {
    status_publish_timer_->cancel();
  }
  const auto publish_period =
      std::chrono::duration<double>(1.0 / publish_frequency_);
  status_publish_timer_ =
      create_wall_timer(publish_period, [this]() { publish_robot_status(); });
}

void RobotControllerNode::publish_robot_status() {
  battery_ -= battery_consumption_rate_;
  if (battery_ < 0.0) {
    battery_ = 0.0;
  }

  std_msgs::msg::Float32 battery_message;
  battery_message.data = static_cast<float>(battery_);
  battery_publisher_->publish(battery_message);

  mini_robot_interfaces::msg::RobotStatus status_message;
  status_message.stamp = now();
  status_message.robot_id = robot_id_;
  status_message.mode = mode_;
  status_message.battery = static_cast<float>(battery_);
  status_message.linear_velocity = 0.0F;
  status_message.angular_velocity = 0.0F;
  status_message.emergency_stop = emergency_stop_;
  status_publisher_->publish(status_message);

  RCLCPP_INFO(get_logger(), "robot_id: %s\nmode: %s\nbattery: %.1f%%",
              robot_id_.c_str(), mode_.c_str(), battery_);
}

}  // namespace Mrb

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mrb::RobotControllerNode>());
  rclcpp::shutdown();
  return 0;
}
