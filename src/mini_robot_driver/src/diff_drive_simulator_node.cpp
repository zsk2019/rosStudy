#include <functional>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace Mrb {

class DiffDriveSimulator : public rclcpp::Node {
 public:
  DiffDriveSimulator()
      : Node("diff_drive_simulator"),
        wheel_radius_(declare_parameter<double>(
            "wheel_radius", 0.05,
            makeReadOnlyDescriptor("Wheel radius in meters"))),
        wheel_separation_(declare_parameter<double>(
            "wheel_separation", 0.44,
            makeReadOnlyDescriptor("Distance between wheels in meters"))),
        left_wheel_joint_name_(declare_parameter<std::string>(
            "left_wheel_joint_name", "left_wheel_joint")),
        right_wheel_joint_name_(declare_parameter<std::string>(
            "right_wheel_joint_name", "right_wheel_joint")) {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&DiffDriveSimulator::cmdVelCallback, this,
                  std::placeholders::_1));
    wheel_speed_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("/wheel_speeds", 10);
  }

 private:
  static rcl_interfaces::msg::ParameterDescriptor makeReadOnlyDescriptor(
      const std::string& description) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    const double linear_velocity = msg->linear.x;
    const double angular_velocity = msg->angular.z;

    const double left_wheel_velocity =
        (linear_velocity - angular_velocity * wheel_separation_ / 2.0) /
        wheel_radius_;

    const double right_wheel_velocity =
        (linear_velocity + angular_velocity * wheel_separation_ / 2.0) /
        wheel_radius_;

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name = {left_wheel_joint_name_, right_wheel_joint_name_};
    joint_state.velocity = {left_wheel_velocity, right_wheel_velocity};
    wheel_speed_pub_->publish(joint_state);

    RCLCPP_INFO(get_logger(), "left=%.2f rad/s right=%.2f rad/s",
                left_wheel_velocity, right_wheel_velocity);
  }

  const double wheel_radius_;
  const double wheel_separation_;
  const std::string left_wheel_joint_name_;
  const std::string right_wheel_joint_name_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_speed_pub_;
};
}  // namespace Mrb

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mrb::DiffDriveSimulator>());
  rclcpp::shutdown();
  return 0;
}
