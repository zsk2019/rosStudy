#pragma once

#include "mini_robot_interfaces/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace Mrb {

enum class RobotMode { IDLE, MANUAL, AUTO, CHARGING, EMERGENCY };

class RobotStateMachine {
 public:
  RobotStateMachine();

  bool setMode(RobotMode mode);

  bool canExecuteTask() const;

  void setBattery(float battery);

  void setEmergencyStop(bool enabled);

  RobotMode getMode() const;

  float getBattery() const;

  bool isEmergencyStop() const;

 private:
  RobotMode mode_;
  float battery_;
  bool emergency_stop_;
};

}  // namespace Mrb