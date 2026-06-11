/**
 * @file Task_R_Motor.cpp
 * @brief Implementation of the rotisserie motor task.
 */

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

    case State::ReturningToInitialRotation:
      if (r_motor_driver_.isFaulted()) {
        print_str("R motor fault while returning to initial rotation\r\n");
        state_ = State::Fault;
      } else if (!r_motor_driver_.isBusy()) {
        state_ = State::Idle;
        stop_requested_ = false;
        cooking_rotation_requested_ = false;
        print_str("R returned to initial rotation.\r\n");
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
  run();
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
    print_str("R cooking rotation rejected: task is faulted.\r\n");
    return;
  }

  if (state_ != State::Idle) {
    return;
  }

  cooking_rotation_requested_ = true;
  stop_requested_ = false;
}

void TaskRMotor::stopCookingRotation() {
  cooking_rotation_requested_ = false;
  stop_requested_ = true;
  r_motor_driver_.stop();

  if (state_ != State::Fault) {
    state_ = State::Idle;
  }

  print_str("R cooking rotation stopped.\r\n");
}

void TaskRMotor::returnToInitialRotation() {
  if (state_ == State::Fault) {
    print_str("R return-to-initial rejected: task is faulted.\r\n");
    return;
  }

  cooking_rotation_requested_ = false;
  stop_requested_ = true;

  r_motor_driver_.moveToDegrees(0, cook_duty_, kReturnToInitialTimeoutMs);
  state_ = State::ReturningToInitialRotation;

  print_str("R returning to initial rotation.\r\n");
}

void TaskRMotor::emergencyStop() {
  r_motor_driver_.stop();
  cooking_rotation_requested_ = false;
  stop_requested_ = false;
  state_ = State::Fault;
  print_str("R emergency stop. Task entered fault state.\r\n");
}

void TaskRMotor::resetFault() {
  r_motor_driver_.stop();
  cooking_rotation_requested_ = false;
  stop_requested_ = false;

  if (state_ == State::Fault) {
    state_ = State::Uninitialized;
  } else {
    state_ = State::Idle;
  }
}

void TaskRMotor::setCookingDuty(int16_t duty) {
  cook_duty_ = duty;
}

bool TaskRMotor::isBusy() const {
  return state_ == State::RotatingForward || state_ == State::RotatingBackward ||
         state_ == State::ReturningToInitialRotation || r_motor_driver_.isBusy();
}

bool TaskRMotor::isFaulted() const {
  return state_ == State::Fault || r_motor_driver_.isFaulted();
}
