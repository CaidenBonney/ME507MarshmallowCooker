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

  r_motor_driver_.update();

  switch (state_) {
    case State::Uninitialized:
      if (r_motor_driver_.begin() != HAL_OK) {
        print_str("R motor driver init failed\r\n");
        state_ = State::Fault;
        return;
      }

      print_str("R motor driver initialized\r\n");
      state_ = State::Idle;
      break;

    case State::Idle:
      if (cooking_rotation_requested_) {
        print_str("R cooking rotation +360\r\n");
        r_motor_driver_.moveDegrees(kCookRotationDegrees, cook_duty_, kMoveTimeoutMs);
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
        print_str("R motor fault during forward rotation\r\n");
        state_ = State::Fault;
      } else if (!r_motor_driver_.isBusy()) {
        print_str("R cooking rotation -360\r\n");
        r_motor_driver_.moveDegrees(-kCookRotationDegrees, cook_duty_, kMoveTimeoutMs);
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
        print_str("R motor fault during backward rotation\r\n");
        state_ = State::Fault;
      } else if (!r_motor_driver_.isBusy()) {
        print_str("R cooking rotation +360\r\n");
        r_motor_driver_.moveDegrees(kCookRotationDegrees, cook_duty_, kMoveTimeoutMs);
        state_ = State::RotatingForward;
      }
      break;

    case State::Fault:
      r_motor_driver_.stop();
      cooking_rotation_requested_ = false;
      stop_requested_ = false;
      break;
  }
}

void TaskRMotor::update() {
  r_motor_driver_.update();
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

TaskRMotor::State TaskRMotor::getState() const {
  return state_;
}

void TaskRMotor::startCookingRotation() {
  if (state_ == State::Fault) {
    return;
  }

  cooking_rotation_requested_ = true;
  stop_requested_ = false;
}

void TaskRMotor::stopCookingRotation() {
  stop_requested_ = true;
}

void TaskRMotor::emergencyStop() {
  r_motor_driver_.stop();
  cooking_rotation_requested_ = false;
  stop_requested_ = false;
  state_ = State::Fault;
}

void TaskRMotor::resetFault() {
  if (state_ == State::Fault) {
    state_ = State::Uninitialized;
  }
}

void TaskRMotor::setCookingDuty(int16_t duty) {
  cook_duty_ = duty;
}

bool TaskRMotor::isBusy() const {
  return state_ == State::RotatingForward || state_ == State::RotatingBackward;
}

bool TaskRMotor::isFaulted() const {
  return state_ == State::Fault || r_motor_driver_.isFaulted();
}
