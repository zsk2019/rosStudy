#include "mini_robot_driver/robot_state_machine.hpp"

namespace Mrb {

RobotStateMachine::RobotStateMachine()
    : mode_(RobotMode::IDLE), battery_(100.0F), emergency_stop_(false) {}

std::optional<RobotMode> RobotStateMachine::fromStatusModeString(
    const std::string& status_mode) {
  using mini_robot_interfaces::msg::RobotStatus;

  if (status_mode == RobotStatus::MODE_IDLE) {
    return RobotMode::IDLE;
  }
  if (status_mode == RobotStatus::MODE_RUNNING) {
    return RobotMode::RUNNING;
  }
  if (status_mode == RobotStatus::MODE_CHARGING) {
    return RobotMode::CHARGING;
  }
  if (status_mode == RobotStatus::MODE_ERROR) {
    return RobotMode::ERROR;
  }
  if (status_mode == RobotStatus::MODE_AUTO) {
    return RobotMode::AUTO;
  }
  if (status_mode == RobotStatus::MODE_EMERGENCY) {
    return RobotMode::EMERGENCY;
  }

  return std::nullopt;
}

std::string RobotStateMachine::toStatusModeString(RobotMode mode) {
  using mini_robot_interfaces::msg::RobotStatus;

  switch (mode) {
    case RobotMode::IDLE:
      return RobotStatus::MODE_IDLE;
    case RobotMode::RUNNING:
      return RobotStatus::MODE_RUNNING;
    case RobotMode::CHARGING:
      return RobotStatus::MODE_CHARGING;
    case RobotMode::ERROR:
      return RobotStatus::MODE_ERROR;
    case RobotMode::AUTO:
      return RobotStatus::MODE_AUTO;
    case RobotMode::EMERGENCY:
      return RobotStatus::MODE_EMERGENCY;
  }

  return RobotStatus::MODE_ERROR;
}

bool RobotStateMachine::setMode(RobotMode mode) {
  if (emergency_stop_ && mode != RobotMode::EMERGENCY) {
    return false;
  }

  if (battery_ < 15.0F && mode == RobotMode::AUTO) {
    return false;
  }

  mode_ = mode;

  return true;
}

bool RobotStateMachine::setModeFromStatusString(
    const std::string& status_mode) {
  const auto mode = fromStatusModeString(status_mode);
  if (!mode.has_value()) {
    return false;
  }

  return setMode(*mode);
}

bool RobotStateMachine::canExecuteTask() const {
  if (emergency_stop_) {
    return false;
  }

  if (battery_ < 15.0F) {
    return false;
  }

  if (mode_ != RobotMode::AUTO) {
    return false;
  }

  return true;
}

void RobotStateMachine::setBattery(float battery) { battery_ = battery; }

void RobotStateMachine::setEmergencyStop(bool enabled) {
  emergency_stop_ = enabled;

  if (enabled) {
    mode_ = RobotMode::EMERGENCY;
  }
}

RobotMode RobotStateMachine::getMode() const { return mode_; }

float RobotStateMachine::getBattery() const { return battery_; }

bool RobotStateMachine::isEmergencyStop() const { return emergency_stop_; }

}  // namespace Mrb
