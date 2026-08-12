#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mini_robot_sensor {

class LaserMockNode : public rclcpp::Node {
 public:
  LaserMockNode() : Node("laser_mock", "/mini_robot") {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(15)).reliable();
    scan_publisher_ =
        create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&LaserMockNode::publish_scan, this));

    RCLCPP_INFO(get_logger(),
                "Laser mock started: publishing /mini_robot/scan at 50 Hz "
                "with Reliable reliability");
  }

 private:
  void publish_scan() {
    constexpr std::size_t kSampleCount = 360;
    constexpr float kPi = 3.14159265358979323846F;

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = now();
    scan.header.frame_id = "laser_link";
    scan.angle_min = -kPi;
    scan.angle_max = kPi;
    scan.angle_increment =
        (scan.angle_max - scan.angle_min) / static_cast<float>(kSampleCount);
    scan.time_increment = 0.02F / static_cast<float>(kSampleCount);
    scan.scan_time = 0.02F;
    scan.range_min = 0.05F;
    scan.range_max = 12.0F;

    scan.ranges.resize(kSampleCount);
    scan.intensities.resize(kSampleCount);
    for (std::size_t index = 0; index < kSampleCount; ++index) {
      const float angle =
          scan.angle_min + static_cast<float>(index) * scan.angle_increment;
      scan.ranges[index] = 4.0F + 0.5F * std::sin(3.0F * angle);
      scan.intensities[index] = 100.0F;
    }

    scan_publisher_->publish(scan);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mini_robot_sensor

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mini_robot_sensor::LaserMockNode>());
  rclcpp::shutdown();
  return 0;
}
