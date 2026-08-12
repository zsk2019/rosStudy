#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mini_robot_sensor {

class LaserListenerNode : public rclcpp::Node {
 public:
  LaserListenerNode() : Node("laser_listener", "/mini_robot") {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos,
        std::bind(&LaserListenerNode::scan_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "Listening to /mini_robot/scan with Best Effort reliability");
  }

 private:
  void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    float closest_range = std::numeric_limits<float>::infinity();
    std::size_t valid_sample_count = 0;

    for (const float range : scan->ranges) {
      if (std::isfinite(range) && range >= scan->range_min &&
          range <= scan->range_max) {
        closest_range = std::min(closest_range, range);
        ++valid_sample_count;
      }
    }

    if (valid_sample_count == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "Received scan without valid ranges");
      return;
    }

    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Received scan: %zu samples, %zu valid, closest range: %.2f m",
        scan->ranges.size(), valid_sample_count, closest_range);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_;
};

}  // namespace mini_robot_sensor

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mini_robot_sensor::LaserListenerNode>());
  rclcpp::shutdown();
  return 0;
}
