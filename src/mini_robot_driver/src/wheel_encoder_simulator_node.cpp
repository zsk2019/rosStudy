#include <algorithm>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace Mrb {

using namespace std::chrono_literals;

class WheelEncoderSimulator : public rclcpp::Node {
 public:
  WheelEncoderSimulator()
      : Node("wheel_encoder_simulator"),
        left_wheel_joint_name_(declare_parameter<std::string>(
            "left_wheel_joint_name", "left_wheel_joint")),
        right_wheel_joint_name_(declare_parameter<std::string>(
            "right_wheel_joint_name", "right_wheel_joint")) {
    wheel_speed_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/wheel_speeds", 10,
        std::bind(&WheelEncoderSimulator::wheelSpeedCallback, this,
                  std::placeholders::_1));
    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = create_wall_timer(
        20ms, std::bind(&WheelEncoderSimulator::publishJointState, this));
  }

 private:
  void wheelSpeedCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const auto left_velocity = findJointVelocity(*msg, left_wheel_joint_name_);
    const auto right_velocity = findJointVelocity(*msg, right_wheel_joint_name_);
    if (!left_velocity.first || !right_velocity.first) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "JointState missing velocity for '%s' or '%s'",
          left_wheel_joint_name_.c_str(), right_wheel_joint_name_.c_str());
      return;
    }

    left_wheel_velocity_ = left_velocity.second;
    right_wheel_velocity_ = right_velocity.second;
  }

  void publishJointState() {
    const rclcpp::Time current_time = now();
    const double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    left_wheel_position_ += left_wheel_velocity_ * dt;
    right_wheel_position_ += right_wheel_velocity_ * dt;

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = current_time;
    joint_state.name = {left_wheel_joint_name_, right_wheel_joint_name_};
    joint_state.position = {left_wheel_position_, right_wheel_position_};
    joint_state.velocity = {left_wheel_velocity_, right_wheel_velocity_};
    joint_state_pub_->publish(joint_state);
  }

  std::pair<bool, double> findJointVelocity(
      const sensor_msgs::msg::JointState& msg,
      const std::string& joint_name) const {
    const auto name_iter =
        std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (name_iter == msg.name.end()) {
      return {false, 0.0};
    }

    const auto index = static_cast<size_t>(name_iter - msg.name.begin());
    if (index >= msg.velocity.size()) {
      return {false, 0.0};
    }

    return {true, msg.velocity[index]};
  }

  const std::string left_wheel_joint_name_;
  const std::string right_wheel_joint_name_;
  double left_wheel_velocity_{0.0};
  double right_wheel_velocity_{0.0};
  double left_wheel_position_{0.0};
  double right_wheel_position_{0.0};
  rclcpp::Time last_update_time_{now()};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      wheel_speed_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace Mrb

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mrb::WheelEncoderSimulator>());
  rclcpp::shutdown();
  return 0;
}
