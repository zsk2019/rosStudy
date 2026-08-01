#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class RobotDriverNode : public rclcpp::Node
{
public:
  RobotDriverNode()
  : Node("robot_driver_node")
  {
    timer_ = create_wall_timer(1s, [this]() { print_robot_status(); });
  }

private:
  void print_robot_status() const
  {
    RCLCPP_INFO(
      get_logger(),
      "robot_id: mini_robot_01\nmode: IDLE\nbattery: 100%%");
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotDriverNode>());
  rclcpp::shutdown();
  return 0;
}
