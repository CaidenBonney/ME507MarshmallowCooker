#include "Task_Z_Motor.h"

#include <cstdio>

TaskZMotor::TaskZMotor()
    : z_motor_driver_(),
      z_limit_switches_(Z_TOP_GPIO_Port, Z_TOP_Pin, Z_BOT_GPIO_Port, Z_BOT_Pin) {
}

void TaskZMotor::run() {
  const uint32_t now_ms = HAL_GetTick();

  if (state_ == State::Uninitialized) {
    z_motor_driver_.begin();
    z_motor_driver_.setLimitsActiveLow(false);
    z_motor_driver_.enable();
    z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);

    state_ = State::Idle;
    print_str("Z motor task initialized. Send 'home' before cooking.\r\n");
    return;
  }

  // Keep the stepper driver updated as often as the main loop calls this task.
  z_motor_driver_.update();

  if ((now_ms - last_update_ms_) < kUpdatePeriodMs) {
    return;
  }

  last_update_ms_ = now_ms;

  handleDriverLimitState();

  if (state_ == State::Fault) {
    z_motor_driver_.stop();
    return;
  }

  switch (state_) {
    case State::Uninitialized:
      break;

    case State::Idle:
      break;

    case State::Homing:
      if (topLimitPressed()) {
        z_motor_driver_.stop();
        z_motor_driver_.zeroPosition();
        homed_ = true;
        state_ = State::Idle;
        print_str("Z homed at top limit. Z position = 0 steps.\r\n");
      } else if (bottomLimitPressed()) {
        z_motor_driver_.stop();
        state_ = State::Fault;
        print_str("Z fault: bottom limit hit while homing upward.\r\n");
      }
      break;

    case State::MovingToStartPosition:
      if (!z_motor_driver_.isBusy()) {
        resetPid();
        state_ = State::ControllingFlameTemp;
        print_str("Z reached starting cook position. PID control active.\r\n");
      }
      break;

    case State::ControllingFlameTemp:
      if (bottomLimitPressed()) {
        z_motor_driver_.stop();
        resetPid();
        state_ = State::Idle;
        print_str("Z bottom limit reached during flame control.\r\n");
        break;
      }

      updatePidControl(now_ms);
      break;

    case State::MovingToRemovalHeight:
      if (!z_motor_driver_.isBusy()) {
        state_ = State::Idle;
        print_str("Z reached removal height.\r\n");
      }
      break;

    case State::Fault:
      break;
  }
}

void TaskZMotor::update() {
  run();
}

Task::Status TaskZMotor::getStatus() const {
  if (state_ == State::Uninitialized) {
    return Task::Status::Uninitialized;
  }

  if (state_ == State::Fault) {
    return Task::Status::Fault;
  }

  return Task::Status::Running;
}

TaskZMotor::State TaskZMotor::getState() const {
  return state_;
}

void TaskZMotor::startHoming() {
  if (state_ == State::Fault) {
    print_str("Z home rejected: task is faulted. Send reset first.\r\n");
    return;
  }

  homed_ = false;
  resetPid();

  z_motor_driver_.setSpeedStepsPerSecond(kHomeSpeedStepsPerSecond);
  z_motor_driver_.home(ZMotorDriver::Direction::Up, kHomeSpeedStepsPerSecond);

  state_ = State::Homing;
  print_str("Z homing upward toward top limit.\r\n");
}

void TaskZMotor::startTemperatureControl(int32_t target_flame_temp_fx100) {
  if (state_ == State::Fault) {
    print_str("Z temperature control rejected: task is faulted.\r\n");
    return;
  }

  if (!homed_) {
    enterFault("temperature control requested before Z home");
    return;
  }

  target_flame_temp_fx100_ = target_flame_temp_fx100;
  resetPid();
  moveToStartPosition();
}

void TaskZMotor::setMeasuredFlameTempFx100(int32_t measured_flame_temp_fx100) {
  measured_flame_temp_fx100_ = measured_flame_temp_fx100;
  valid_flame_temp_ = true;
}

void TaskZMotor::setPidDebugEnabled(bool enabled) {
  pid_debug_enabled_ = enabled;

  if (pid_debug_enabled_) {
    print_str("Z PID debug enabled\r\n");
  } else {
    print_str("Z PID debug disabled\r\n");
  }
}

void TaskZMotor::moveToRemovalHeight() {
  if (state_ == State::Fault) {
    print_str("Z removal move rejected: task is faulted.\r\n");
    return;
  }

  if (!homed_) {
    enterFault("removal height move requested before Z home");
    return;
  }

  resetPid();
  z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);
  z_motor_driver_.moveTo(kRemovalHeightSteps);
  state_ = State::MovingToRemovalHeight;
}

void TaskZMotor::moveToStartPosition() {
  if (state_ == State::Fault) {
    print_str("Z start-position move rejected: task is faulted.\r\n");
    return;
  }

  if (!homed_) {
    enterFault("start-position move requested before Z home");
    return;
  }

  z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);
  z_motor_driver_.moveTo(kStartCookingPositionSteps);
  state_ = State::MovingToStartPosition;
}

void TaskZMotor::jogRelativeSteps(int32_t relative_steps) {
  if (state_ == State::Fault) {
    print_str("Z jog rejected: task is faulted. Send reset first.\r\n");
    return;
  }

  if (!homed_) {
    print_str("Z jog rejected: home Z first.\r\n");
    return;
  }

  if (state_ == State::Homing || state_ == State::MovingToStartPosition || state_ == State::ControllingFlameTemp ||
      state_ == State::MovingToRemovalHeight || z_motor_driver_.isBusy()) {
    print_str("Z jog rejected: Z is busy.\r\n");
    return;
  }

  int32_t target_steps = z_motor_driver_.getPositionSteps() + relative_steps;

  if (target_steps > 0) {
    target_steps = 0;
  }

  if (target_steps < kMinCookPositionSteps) {
    target_steps = kMinCookPositionSteps;
  }

  if (topLimitPressed() && target_steps > z_motor_driver_.getPositionSteps()) {
    print_str("Z jog up rejected: top limit is pressed.\r\n");
    return;
  }

  if (bottomLimitPressed() && target_steps < z_motor_driver_.getPositionSteps()) {
    print_str("Z jog down rejected: bottom limit is pressed.\r\n");
    return;
  }

  z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);
  z_motor_driver_.moveTo(target_steps);

  std::snprintf(print_buf,
                100,
                "Z jog: moving from %ld to %ld steps.\r\n",
                static_cast<long>(z_motor_driver_.getPositionSteps()),
                static_cast<long>(target_steps));
  print_str(print_buf);
}

void TaskZMotor::stopMotion() {
  z_motor_driver_.stop();
  resetPid();

  if (state_ != State::Fault) {
    state_ = State::Idle;
  }
}

void TaskZMotor::emergencyStop() {
  z_motor_driver_.stop();
  resetPid();
  state_ = State::Fault;
  print_str("Z emergency stop. Task entered fault state.\r\n");
}

void TaskZMotor::resetFault() {
  z_motor_driver_.stop();
  resetPid();
  homed_ = false;
  valid_flame_temp_ = false;
  state_ = State::Idle;
  print_str("Z fault reset. Re-home before cooking.\r\n");
}

void TaskZMotor::setPidGains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  resetPid();
}

void TaskZMotor::resetPid() {
  integral_error_ = 0.0f;
  previous_error_f_ = 0.0f;
  previous_error_valid_ = false;
  last_pid_update_ms_ = HAL_GetTick();
}

bool TaskZMotor::isBusy() const {
  return state_ == State::Homing || state_ == State::MovingToStartPosition || state_ == State::ControllingFlameTemp ||
         state_ == State::MovingToRemovalHeight || z_motor_driver_.isBusy();
}

bool TaskZMotor::isFaulted() const {
  return state_ == State::Fault || z_motor_driver_.getState() == ZMotorDriver::State::Fault;
}

bool TaskZMotor::isHomed() const {
  return homed_;
}

int32_t TaskZMotor::getPositionSteps() const {
  return z_motor_driver_.getPositionSteps();
}

int32_t TaskZMotor::getTargetSteps() const {
  return z_motor_driver_.getTargetSteps();
}

void TaskZMotor::enterFault(const char* reason) {
  z_motor_driver_.stop();
  resetPid();

  print_str("Z fault: ");
  print_str(reason);
  print_str("\r\n");

  state_ = State::Fault;
}

void TaskZMotor::handleDriverLimitState() {
  const ZMotorDriver::State driver_state = z_motor_driver_.getState();

  if (driver_state == ZMotorDriver::State::HitBottomLimit) {
    if (state_ == State::ControllingFlameTemp) {
      z_motor_driver_.stop();
      resetPid();
      state_ = State::Idle;
      print_str("Z bottom limit reached during flame control.\r\n");
      return;
    }

    if (state_ == State::MovingToStartPosition || state_ == State::MovingToRemovalHeight) {
      enterFault("bottom limit switch hit");
    }
  }

  if (driver_state == ZMotorDriver::State::HitTopLimit) {
    if (state_ == State::Homing) {
      return;
    }

    if (state_ == State::ControllingFlameTemp || state_ == State::MovingToRemovalHeight) {
      z_motor_driver_.stop();
    }
  }
}

void TaskZMotor::updatePidControl(uint32_t now_ms) {
  if (!valid_flame_temp_) {
    return;
  }

  const uint32_t elapsed_ms = now_ms - last_pid_update_ms_;

  if (elapsed_ms < kPidUpdatePeriodMs) {
    return;
  }

  last_pid_update_ms_ = now_ms;

  const float dt_s = static_cast<float>(elapsed_ms) / 1000.0f;

  if (dt_s <= 0.0f) {
    return;
  }

  const float target_f = static_cast<float>(target_flame_temp_fx100_) / 100.0f;
  const float measured_f = static_cast<float>(measured_flame_temp_fx100_) / 100.0f;
  const float error_f = target_f - measured_f;

  integral_error_ += error_f * dt_s;

  if (integral_error_ > kIntegralErrorLimit) {
    integral_error_ = kIntegralErrorLimit;
  } else if (integral_error_ < -kIntegralErrorLimit) {
    integral_error_ = -kIntegralErrorLimit;
  }

  const float derivative_error = previous_error_valid_ ? ((error_f - previous_error_f_) / dt_s) : 0.0f;

  previous_error_f_ = error_f;
  previous_error_valid_ = true;

  const float output_steps_f = (kp_ * error_f) + (ki_ * integral_error_) + (kd_ * derivative_error);
  const int32_t output_steps = clampPidOutputSteps(output_steps_f);

  if (output_steps <= kPidDeadbandSteps && output_steps >= -kPidDeadbandSteps) {
    return;
  }

  int32_t base_steps = z_motor_driver_.isBusy() ? z_motor_driver_.getTargetSteps() : z_motor_driver_.getPositionSteps();
  int32_t next_target_steps = base_steps - output_steps;

  if (next_target_steps > 0) {
    next_target_steps = 0;
  }

  if (next_target_steps < kMinCookPositionSteps) {
    z_motor_driver_.stop();
    state_ = State::Fault;
    print_str("Z fault: PID reached minimum allowed cook height. Flame target unreachable.\r\n");
    return;
  }

  if (topLimitPressed() && next_target_steps > z_motor_driver_.getPositionSteps()) {
    next_target_steps = z_motor_driver_.getPositionSteps();
  }

  if (bottomLimitPressed() && next_target_steps < z_motor_driver_.getPositionSteps()) {
    z_motor_driver_.stop();
    resetPid();
    state_ = State::Idle;
    print_str("Z bottom limit reached during PID control.\r\n");
    return;
  }

  if (next_target_steps == z_motor_driver_.getTargetSteps()) {
    return;
  }

  z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);
  z_motor_driver_.moveTo(next_target_steps);

  const int32_t error_fx100 = target_flame_temp_fx100_ - measured_flame_temp_fx100_;

  if (pid_debug_enabled_) {
    std::snprintf(print_buf,
                  100,
                  "Z PID: target=%ld.%02ldF measured=%ld.%02ldF error=%ld.%02ldF cmd=%ld pos=%ld target_steps=%ld\r\n",
                  static_cast<long>(target_flame_temp_fx100_ / 100),
                  static_cast<long>(target_flame_temp_fx100_ >= 0 ? target_flame_temp_fx100_ % 100
                                                                  : -(target_flame_temp_fx100_ % 100)),
                  static_cast<long>(measured_flame_temp_fx100_ / 100),
                  static_cast<long>(measured_flame_temp_fx100_ >= 0 ? measured_flame_temp_fx100_ % 100
                                                                    : -(measured_flame_temp_fx100_ % 100)),
                  static_cast<long>(error_fx100 / 100),
                  static_cast<long>(error_fx100 >= 0 ? error_fx100 % 100 : -(error_fx100 % 100)),
                  static_cast<long>(output_steps),
                  static_cast<long>(z_motor_driver_.getPositionSteps()),
                  static_cast<long>(next_target_steps));
    print_str(print_buf);
  }
}

int32_t TaskZMotor::clampPidOutputSteps(float output_steps) const {
  if (output_steps > static_cast<float>(kPidOutputLimitSteps)) {
    return kPidOutputLimitSteps;
  }

  if (output_steps < -static_cast<float>(kPidOutputLimitSteps)) {
    return -kPidOutputLimitSteps;
  }

  return static_cast<int32_t>(output_steps);
}

bool TaskZMotor::topLimitPressed() const {
  return HAL_GPIO_ReadPin(Z_TOP_GPIO_Port, Z_TOP_Pin) == GPIO_PIN_SET;
}

bool TaskZMotor::bottomLimitPressed() const {
  return HAL_GPIO_ReadPin(Z_BOT_GPIO_Port, Z_BOT_Pin) == GPIO_PIN_SET;
}
