#pragma once

#include <optional>
#include <string>

#include "mini_robot_interfaces/msg/robot_status.hpp"

namespace Mrb {

enum class RobotMode { IDLE, RUNNING, CHARGING, ERROR, AUTO, EMERGENCY };

class RobotStateMachine {
 public:
  RobotStateMachine();

  static std::optional<RobotMode> fromStatusModeString(
      const std::string& status_mode);

  static std::string toStatusModeString(RobotMode mode);

  bool setMode(RobotMode mode);

  bool setModeFromStatusString(const std::string& status_mode);

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
