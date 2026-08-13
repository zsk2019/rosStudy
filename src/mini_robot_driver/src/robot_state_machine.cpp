#include "mini_robot_driver/robot_state_machine.hpp"

namespace Mrb {

RobotStateMachine::RobotStateMachine()
    : mode_(RobotMode::IDLE), battery_(100.0F), emergency_stop_(false) {}

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