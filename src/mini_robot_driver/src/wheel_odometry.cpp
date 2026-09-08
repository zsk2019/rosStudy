#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

geometry_msgs::msg::Quaternion yawToQuaternion(const double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace

namespace Mrb {

class WheelOdometry : public rclcpp::Node {
 public:
  WheelOdometry()
      : Node("wheel_odometry"),
        wheel_radius_(declare_parameter<double>("wheel_radius", 0.05)),
        wheel_separation_(declare_parameter<double>("wheel_separation", 0.44)),
        left_wheel_joint_name_(declare_parameter<std::string>(
            "left_wheel_joint_name", "left_wheel_joint")),
        right_wheel_joint_name_(declare_parameter<std::string>(
            "right_wheel_joint_name", "right_wheel_joint")),
        odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
        base_frame_(declare_parameter<std::string>("base_frame", "base_link")) {
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&WheelOdometry::jointStateCallback, this,
                  std::placeholders::_1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

 private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    const auto left_position = findJointPosition(*msg, left_wheel_joint_name_);
    const auto right_position =
        findJointPosition(*msg, right_wheel_joint_name_);
    if (!left_position.first || !right_position.first) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "JointState missing '%s' or '%s'",
          left_wheel_joint_name_.c_str(), right_wheel_joint_name_.c_str());
      return;
    }

    const rclcpp::Time current_time =
        msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0
            ? now()
            : rclcpp::Time(msg->header.stamp);

    if (!has_last_position_) {
      last_left_position_ = left_position.second;
      last_right_position_ = right_position.second;
      last_update_time_ = current_time;
      has_last_position_ = true;
      return;
    }

    const double dt = (current_time - last_update_time_).seconds();
    if (dt <= 0.0) {
      last_left_position_ = left_position.second;
      last_right_position_ = right_position.second;
      last_update_time_ = current_time;
      return;
    }

    const double delta_left =
        (left_position.second - last_left_position_) * wheel_radius_;
    const double delta_right =
        (right_position.second - last_right_position_) * wheel_radius_;
    const double delta_center = (delta_right + delta_left) * 0.5;
    const double delta_yaw = (delta_right - delta_left) / wheel_separation_;
    const double mid_yaw = yaw_ + delta_yaw * 0.5;

    x_ += delta_center * std::cos(mid_yaw);
    y_ += delta_center * std::sin(mid_yaw);
    yaw_ += delta_yaw;

    const double linear_velocity = delta_center / dt;
    const double angular_velocity = delta_yaw / dt;

    publishOdometry(current_time, linear_velocity, angular_velocity);
    publishTransform(current_time);

    last_left_position_ = left_position.second;
    last_right_position_ = right_position.second;
    last_update_time_ = current_time;
  }

  std::pair<bool, double> findJointPosition(
      const sensor_msgs::msg::JointState& msg,
      const std::string& joint_name) const {
    const auto name_iter =
        std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (name_iter == msg.name.end()) {
      return {false, 0.0};
    }

    const auto index = static_cast<size_t>(name_iter - msg.name.begin());
    if (index >= msg.position.size()) {
      return {false, 0.0};
    }

    return {true, msg.position[index]};
  }

  void publishOdometry(const rclcpp::Time& stamp, const double linear_velocity,
                       const double angular_velocity) {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = yawToQuaternion(yaw_);
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = angular_velocity;
    odom_pub_->publish(odom);
  }

  void publishTransform(const rclcpp::Time& stamp) {
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = stamp;
    odom_tf.header.frame_id = odom_frame_;
    odom_tf.child_frame_id = base_frame_;
    odom_tf.transform.translation.x = x_;
    odom_tf.transform.translation.y = y_;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation = yawToQuaternion(yaw_);
    tf_broadcaster_->sendTransform(odom_tf);
  }

  const double wheel_radius_;
  const double wheel_separation_;
  const std::string left_wheel_joint_name_;
  const std::string right_wheel_joint_name_;
  const std::string odom_frame_;
  const std::string base_frame_;

  bool has_last_position_{false};
  double last_left_position_{0.0};
  double last_right_position_{0.0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace Mrb

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mrb::WheelOdometry>());
  rclcpp::shutdown();
  return 0;
}
