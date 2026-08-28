#include <cmath>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

class ArmJointStatePublisher : public rclcpp::Node
{
public:
  ArmJointStatePublisher()
  : Node("arm_joint_state_publisher"), start_time_(now())
  {
    joint_name_ = declare_parameter<std::string>("joint_name", "arm_joint");
    amplitude_ = declare_parameter<double>("amplitude", 1.0);
    period_ = declare_parameter<double>("period", 0.5);
    publish_interval_ = declare_parameter<double>("publish_interval", 0.02);

    if (period_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "period must be positive, using 0.5 seconds");
      period_ = 0.5;
    }
    if (publish_interval_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "publish_interval must be positive, using 0.02 seconds");
      publish_interval_ = 0.02;
    }

    publisher_ = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(publish_interval_)),
      std::bind(&ArmJointStatePublisher::publishJointState, this));
  }

private:
  void publishJointState()
  {
    const double elapsed = (now() - start_time_).seconds();
    const double position = amplitude_ * std::sin(2.0 * kPi * elapsed / period_);
    const double velocity = amplitude_ * 2.0 * kPi / period_ *
      std::cos(2.0 * kPi * elapsed / period_);

    sensor_msgs::msg::JointState joint_state;
    joint_state.header.stamp = now();
    joint_state.name.push_back(joint_name_);
    joint_state.position.push_back(position);
    joint_state.velocity.push_back(velocity);

    publisher_->publish(joint_state);
  }

  rclcpp::Time start_time_;
  std::string joint_name_;
  double amplitude_;
  double period_;
  double publish_interval_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmJointStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
