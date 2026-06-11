#include "Task_R_Motor.h"

TaskRMotor::TaskRMotor()
  : r_motor_driver_() {
}

void TaskRMotor::run() {
  const uint32_t now_ms = HAL_GetTick();

  if ((now_ms - last_update_ms_) < kUpdatePeriodMs) {
    return;
  }

  last_update_ms_ = now_ms;

  if (emergency_stop_requested_) {
    r_motor_driver_.stop();
    state_ = State::Fault;
    return;
  }

  r_motor_driver_.update();

  switch (state_) {
    case State::Uninitialized:
      if (r_motor_driver_.begin() != HAL_OK) {
        state_ = State::Fault;
        return;
      }

      state_ = State::Idle;
      break;

    case State::Idle:
      if (cooking_rotation_requested_) {
        r_motor_driver_.moveDegrees(kCookRotationDegrees,
                                    cook_duty_,
                                    kMoveTimeoutMs);
        state_ = State::RotatingForward;
      }
      break;

    case State::RotatingForward:
      if (stop_requested_) {
        r_motor_driver_.stop();
        cooking_rotation_requested_ = false;
        stop_requested_ = false;
        state_ = State::Idle;
      } else if (r_motor_driver_.isFaulted()) {
        state_ = State::Fault;
      } else if (!r_motor_driver_.isBusy()) {
        r_motor_driver_.moveDegrees(-kCookRotationDegrees,
                                    cook_duty_,
                                    kMoveTimeoutMs);
        state_ = State::RotatingBackward;
      }
      break;

    case State::RotatingBackward:
      if (stop_requested_) {
        r_motor_driver_.stop();
        cooking_rotation_requested_ = false;
        stop_requested_ = false;
        state_ = State::Idle;
      } else if (r_motor_driver_.isFaulted()) {
        state_ = State::Fault;
      } else if (!r_motor_driver_.isBusy()) {
        r_motor_driver_.moveDegrees(kCookRotationDegrees,
                                    cook_duty_,
                                    kMoveTimeoutMs);
        state_ = State::RotatingForward;
      }
      break;

    case State::Stopping:
      r_motor_driver_.stop();
      cooking_rotation_requested_ = false;
      stop_requested_ = false;
      state_ = State::Idle;
      break;

    case State::Fault:
      r_motor_driver_.stop();
      break;
  }
}

void TaskRMotor::update() {
  r_motor_driver_.update();
}

TaskRMotor::State TaskRMotor::getState() const {
  return state_;
}

Task::Status TaskRMotor::getStatus() const {
  if (state_ == State::Uninitialized) {
    return Task::Status::Uninitialized;
  }

  if (state_ == State::Fault) {
    return Task::Status::Fault;
  }

  return Task::Status::Running;
}

void TaskRMotor::startCookingRotation() {
  cooking_rotation_requested_ = true;
  stop_requested_ = false;
}

void TaskRMotor::stopCookingRotation() {
  stop_requested_ = true;
}

void TaskRMotor::emergencyStop() {
  emergency_stop_requested_ = true;
}

bool TaskRMotor::isBusy() const {
  return state_ == State::RotatingForward ||
         state_ == State::RotatingBackward;
}

TaskRMotor::State TaskRMotor::getState() const {
  return state_;
}