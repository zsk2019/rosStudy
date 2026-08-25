#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace Mrb
{

class SensorTfBroadcaster : public rclcpp::Node
{
public:
  SensorTfBroadcaster()
  : Node("sensor_tf_broadcaster")
  {
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publishSensorTransforms();
  }

private:
  geometry_msgs::msg::TransformStamped makeTransform(
    const std::string & child_frame,
    const double x,
    const double y,
    const double z) const
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = child_frame;
    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = z;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    return transform;
  }

  void publishSensorTransforms()
  {
    const auto laser_frame = declare_parameter<std::string>("laser_frame", "laser_link");
    const auto camera_frame = declare_parameter<std::string>("camera_frame", "camera_link");

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(makeTransform(laser_frame, 0.18, 0.0, 0.16));
    transforms.push_back(makeTransform(camera_frame, 0.12, 0.0, 0.28));

    static_broadcaster_->sendTransform(transforms);
  }

  std::string base_frame_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

}  // namespace Mrb

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mrb::SensorTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
