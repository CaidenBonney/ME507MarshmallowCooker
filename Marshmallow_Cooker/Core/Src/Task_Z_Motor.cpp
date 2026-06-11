#include "Task_Z_Motor.h"

TaskZMotor::TaskZMotor()
    : z_motor_driver_(),
      z_limit_switches_(Z_TOP_GPIO_Port, Z_TOP_Pin, Z_BOT_GPIO_Port, Z_BOT_Pin) {
}

void TaskZMotor::run() {
  const uint32_t now_ms = HAL_GetTick();

  if ((now_ms - last_update_ms_) < kUpdatePeriodMs) {
    return;
  }

  last_update_ms_ = now_ms;

  handleLimitSwitches();
  z_motor_driver_.update();

  if (stop_requested_) {
    z_motor_driver_.stop();
    stop_requested_ = false;

    if (state_ != State::Fault) {
      state_ = State::Idle;
    }
  }

  switch (state_) {
    case State::Uninitialized:
      z_motor_driver_.begin();
      z_motor_driver_.enable();
      z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);

      print_str("Z motor driver initialized\r\n");
      state_ = State::Idle;
      break;

    case State::Idle:
      if (home_requested_) {
        home_requested_ = false;
        homed_ = false;

        print_str("Z homing up to top limit\r\n");
        z_motor_driver_.homeUp(kHomeSpeedStepsPerSecond);
        state_ = State::FindingTopLimit;
      } else if (removal_height_requested_) {
        removal_height_requested_ = false;

        if (!homed_) {
          print_str("Z removal move rejected: not homed\r\n");
          state_ = State::Fault;
        } else {
          print_str("Z moving to removal height\r\n");
          z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);
          z_motor_driver_.moveTo(kRemovalHeightSteps);
          state_ = State::MovingToRemovalHeight;
        }
      } else if (target_move_requested_) {
        target_move_requested_ = false;

        if (!homed_) {
          print_str("Z target move rejected: not homed\r\n");
          state_ = State::Fault;
        } else {
          z_motor_driver_.setSpeedStepsPerSecond(kMoveSpeedStepsPerSecond);
          z_motor_driver_.moveTo(requested_target_steps_);
          state_ = State::MovingToTarget;
        }
      } else if (temp_control_requested_) {
        if (!homed_) {
          print_str("Z temp control rejected: not homed\r\n");
          temp_control_requested_ = false;
          state_ = State::Fault;
        } else {
          state_ = State::ControllingFlameTemp;
        }
      }
      break;

    case State::FindingTopLimit:
      if (z_motor_driver_.getState() == ZMotorDriver::State::HitTopLimit) {
        z_motor_driver_.zeroPosition();
        homed_ = true;
        print_str("Z homed at top limit, position = 0\r\n");
        state_ = State::Idle;
      } else if (z_motor_driver_.isFaulted()) {
        state_ = State::Fault;
      }
      break;

    case State::MovingToRemovalHeight:
    case State::MovingToTarget:
      if (z_motor_driver_.isFaulted()) {
        state_ = State::Fault;
      } else if (!z_motor_driver_.isBusy()) {
        state_ = State::Idle;
      }
      break;

    case State::ControllingFlameTemp:
      if (!temp_control_requested_) {
        z_motor_driver_.stop();
        state_ = State::Idle;
      } else if (z_motor_driver_.isFaulted()) {
        state_ = State::Fault;
      } else {
        updateTemperatureControl();
      }
      break;

    case State::Fault:
      z_motor_driver_.stop();
      break;
  }
}

void TaskZMotor::update() {
  z_motor_driver_.update();
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
    return;
  }

  home_requested_ = true;
}

void TaskZMotor::moveToRemovalHeight() {
  if (state_ == State::Fault) {
    return;
  }

  removal_height_requested_ = true;
}

void TaskZMotor::moveToTarget(int32_t target_position_steps) {
  if (state_ == State::Fault) {
    return;
  }

  requested_target_steps_ = target_position_steps;
  target_move_requested_ = true;
}

void TaskZMotor::startTemperatureControl(int16_t target_flame_temp_fx100) {
  if (state_ == State::Fault) {
    return;
  }

  target_flame_temp_fx100_ = target_flame_temp_fx100;
  temp_control_requested_ = true;
}

void TaskZMotor::stopMotion() {
  temp_control_requested_ = false;
  stop_requested_ = true;
}

void TaskZMotor::emergencyStop() {
  z_motor_driver_.stop();
  temp_control_requested_ = false;
  stop_requested_ = false;
  home_requested_ = false;
  removal_height_requested_ = false;
  target_move_requested_ = false;
  homed_ = false;
  state_ = State::Fault;
}

void TaskZMotor::resetFault() {
  if (state_ == State::Fault) {
    homed_ = false;
    state_ = State::Uninitialized;
  }
}

void TaskZMotor::setMeasuredFlameTempFx100(int16_t flame_temp_fx100) {
  measured_flame_temp_fx100_ = flame_temp_fx100;
  has_flame_temp_ = true;
}

bool TaskZMotor::isHomed() const {
  return homed_;
}

bool TaskZMotor::isBusy() const {
  return state_ == State::FindingTopLimit || state_ == State::MovingToRemovalHeight ||
         state_ == State::MovingToTarget || state_ == State::ControllingFlameTemp;
}

bool TaskZMotor::isFaulted() const {
  return state_ == State::Fault || z_motor_driver_.isFaulted();
}

int32_t TaskZMotor::getPositionSteps() const {
  return z_motor_driver_.getPositionSteps();
}

void TaskZMotor::handleLimitSwitches() {
  if (z_limit_switches_.isTopTriggered()) {
    if (state_ != State::FindingTopLimit) {
      print_str("Z top limit triggered\r\n");
    }
  }

  if (z_limit_switches_.isBottomTriggered()) {
    print_str("Z bottom limit triggered\r\n");
  }
}

void TaskZMotor::updateTemperatureControl() {
  if (!has_flame_temp_) {
    return;
  }

  if (z_motor_driver_.isBusy()) {
    return;
  }

  const int32_t error_fx100 = static_cast<int32_t>(target_flame_temp_fx100_) - measured_flame_temp_fx100_;

  if (error_fx100 > kPidDeadbandFx100) {
    // Measured flame temperature is too low, move down closer to flame.
    z_motor_driver_.moveSteps(-kPidCorrectionStepLimit);
  } else if (error_fx100 < -kPidDeadbandFx100) {
    // Measured flame temperature is too high, move up away from flame.
    z_motor_driver_.moveSteps(kPidCorrectionStepLimit);
  }
}
