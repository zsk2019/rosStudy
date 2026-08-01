#include <chrono>
#include <memory>

#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class RobotDriverNode : public rclcpp::Node {
 public:
  RobotDriverNode() : Node("robot_driver_node"), battery_(100.0) {
    battery_publisher_ =
        create_publisher<std_msgs::msg::Float32>("/robot/battery", 10);
    status_publisher_ =
        create_publisher<mini_robot_interfaces::msg::RobotStatus>(
            "robot/status", 10);
    timer_ = create_wall_timer(1s, [this]() { print_robot_status(); });
  }

 private:
  void print_robot_status() {
    battery_ -= 0.5;
    if (battery_ < 0.0) {
      battery_ = 0.0;
    }

    std_msgs::msg::Float32 battery_message;
    battery_message.data = static_cast<float>(battery_);
    battery_publisher_->publish(battery_message);

    mini_robot_interfaces::msg::RobotStatus status_message;
    status_message.stamp = now();
    status_message.robot_id = "mini_robot_01";
    status_message.mode = mini_robot_interfaces::msg::RobotStatus::MODE_IDLE;
    status_message.battery = static_cast<float>(battery_);
    status_message.linear_velocity = 0.0F;
    status_message.angular_velocity = 0.0F;
    status_message.emergency_stop = false;
    status_publisher_->publish(status_message);

    RCLCPP_INFO(get_logger(),
                "robot_id: mini_robot_01\nmode: IDLE\nbattery: %.1f%%",
                battery_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_publisher_;
  rclcpp::Publisher<mini_robot_interfaces::msg::RobotStatus>::SharedPtr
      status_publisher_;
  double battery_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDriverNode>());
  rclcpp::shutdown();
  return 0;
}
