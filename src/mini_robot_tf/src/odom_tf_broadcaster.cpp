#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

namespace
{

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

namespace Mrb
{

class OdomTfBroadcaster : public rclcpp::Node
{
public:
  OdomTfBroadcaster()
  : Node("odom_tf_broadcaster"), x_(0.0), y_(0.0), yaw_(0.0)
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    linear_velocity_ = declare_parameter<double>("linear_velocity", 0.2);
    angular_velocity_ = declare_parameter<double>("angular_velocity", 0.4);
    publish_period_ = declare_parameter<double>("publish_period", 0.05);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    last_update_time_ = now();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(publish_period_)),
      std::bind(&OdomTfBroadcaster::publishTransform, this));
  }

private:
  void publishTransform()
  {
    const auto current_time = now();
    const double dt = (current_time - last_update_time_).seconds();
    last_update_time_ = current_time;

    yaw_ += angular_velocity_ * dt;
    x_ += linear_velocity_ * std::cos(yaw_) * dt;
    y_ += linear_velocity_ * std::sin(yaw_) * dt;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = odom_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = yawToQuaternion(yaw_);

    tf_broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_;
  std::string base_frame_;
  double linear_velocity_;
  double angular_velocity_;
  double publish_period_;
  double x_;
  double y_;
  double yaw_;
  rclcpp::Time last_update_time_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace Mrb

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mrb::OdomTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
