#include <chrono>
#include <memory>
#include <string>

#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class RobotDriverNode : public rclcpp::Node {
 public:
  RobotDriverNode() : Node("robot_driver_node") {
    robot_id_ = declare_parameter<std::string>("robot_id", "mini_robot_01");
    publish_frequency_ = declare_parameter<double>("publish_frequency", 10.0);

    rcl_interfaces::msg::ParameterDescriptor battery_descriptor;
    battery_descriptor.description = "Initial battery percentage";
    rcl_interfaces::msg::FloatingPointRange battery_range;
    battery_range.from_value = 0.0;
    battery_range.to_value = 100.0;
    battery_range.step = 0.0;
    battery_descriptor.floating_point_range = {battery_range};
    initial_battery_ = declare_parameter<double>(
        "initial_battery", 100.0, battery_descriptor);
    battery_ = initial_battery_;
    battery_consumption_rate_ =
        declare_parameter<double>("battery_consumption_rate", 0.1);

    rcl_interfaces::msg::ParameterDescriptor velocity_descriptor;
    velocity_descriptor.description = "Maximum linear velocity in m/s";
    rcl_interfaces::msg::FloatingPointRange velocity_range;
    velocity_range.from_value = 0.0;
    velocity_range.to_value = 5.0;
    velocity_range.step = 0.0;
    velocity_descriptor.floating_point_range = {velocity_range};
    max_linear_velocity_ = declare_parameter<double>(
        "max_linear_velocity", 1.5, velocity_descriptor);

    if (publish_frequency_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "publish_frequency must be greater than zero; using 10.0 Hz");
      publish_frequency_ = 10.0;
    }

    battery_publisher_ =
        create_publisher<std_msgs::msg::Float32>("/robot/battery", 10);
    status_publisher_ =
        create_publisher<mini_robot_interfaces::msg::RobotStatus>(
            "robot/status", 10);
    parameter_callback_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& parameters) {
          return on_parameters_changed(parameters);
        });
    update_publish_timer();

    RCLCPP_INFO(get_logger(),
                "robot_id: %s, publish_frequency: %.1f Hz, "
                "initial_battery: %.1f%%, battery_consumption_rate: %.1f, "
                "max_linear_velocity: %.1f m/s",
                robot_id_.c_str(), publish_frequency_, battery_,
                battery_consumption_rate_, max_linear_velocity_);
  }

 private:
  rcl_interfaces::msg::SetParametersResult on_parameters_changed(
      const std::vector<rclcpp::Parameter>& parameters) {
    auto robot_id = robot_id_;
    auto publish_frequency = publish_frequency_;
    auto initial_battery = initial_battery_;
    auto battery_consumption_rate = battery_consumption_rate_;
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
      } else if (name == "max_linear_velocity") {
        max_linear_velocity = parameter.as_double();
        if (max_linear_velocity < 0.0 || max_linear_velocity > 5.0) {
          result.reason =
              "max_linear_velocity must be between 0.0 and 5.0";
          return result;
        }
      }
    }

    const bool frequency_changed = publish_frequency != publish_frequency_;
    robot_id_ = robot_id;
    publish_frequency_ = publish_frequency;
    initial_battery_ = initial_battery;
    battery_consumption_rate_ = battery_consumption_rate;
    max_linear_velocity_ = max_linear_velocity;
    if (reset_battery) {
      battery_ = initial_battery_;
    }
    if (frequency_changed) {
      update_publish_timer();
    }

    result.successful = true;
    result.reason = "parameters updated successfully";
    return result;
  }

  void update_publish_timer() {
    if (timer_) {
      timer_->cancel();
    }
    const auto publish_period =
        std::chrono::duration<double>(1.0 / publish_frequency_);
    timer_ = create_wall_timer(publish_period,
                               [this]() { print_robot_status(); });
  }

  void print_robot_status() {
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
    status_message.mode = mini_robot_interfaces::msg::RobotStatus::MODE_IDLE;
    status_message.battery = static_cast<float>(battery_);
    status_message.linear_velocity = 0.0F;
    status_message.angular_velocity = 0.0F;
    status_message.emergency_stop = false;
    status_publisher_->publish(status_message);

    RCLCPP_INFO(get_logger(),
                "robot_id: %s\nmode: IDLE\nbattery: %.1f%%",
                robot_id_.c_str(), battery_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      parameter_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
  rclcpp::Publisher<mini_robot_interfaces::msg::RobotStatus>::SharedPtr
      status_publisher_;
  std::string robot_id_;
  double publish_frequency_;
  double initial_battery_;
  double battery_;
  double battery_consumption_rate_;
  double max_linear_velocity_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDriverNode>());
  rclcpp::shutdown();
  return 0;
}
