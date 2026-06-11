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

  if (z_limit_switches_.isTopTriggered()) {
    print_str("TOP LIMIT\r\n");

    if (state_ == State::MovingUp) {
      z_motor_driver_.stop();
      state_ = State::HitTopLimit;
    }
  }

  if (z_limit_switches_.isBottomTriggered()) {
    print_str("BOTTOM LIMIT\r\n");

    if (state_ == State::MovingDown || state_ == State::HomingDown) {
      z_motor_driver_.stop();
      z_motor_driver_.zeroPosition();
      state_ = State::HitBottomLimit;
    }
  }

  switch (state_) {
    case State::Uninitialized:
      z_motor_driver_.begin();
      z_motor_driver_.enable();
      z_motor_driver_.setSpeedStepsPerSecond(500);

      print_str("Z motor driver initialized\r\n");

      state_ = State::Idle;
      break;

    case State::Idle:
      if (!test_move_started_) {
        test_move_started_ = true;

        z_motor_driver_.moveSteps(1600);
        state_ = State::MovingUp;

        print_str("Z moving up\r\n");
      }
      break;

    case State::MovingUp:
      z_motor_driver_.update();

      if (!z_motor_driver_.isBusy()) {
        z_motor_driver_.moveSteps(-1600);
        state_ = State::MovingDown;

        print_str("Z moving down\r\n");
      }
      break;

    case State::MovingDown:
      z_motor_driver_.update();

      if (!z_motor_driver_.isBusy()) {
        z_motor_driver_.disable();
        state_ = State::Idle;

        print_str("Z test move complete\r\n");
      }
      break;

    case State::HomingDown:
      z_motor_driver_.update();

      if (!z_motor_driver_.isBusy()) {
        state_ = State::Idle;
      }
      break;

    case State::HitTopLimit:
      z_motor_driver_.disable();
      state_ = State::Idle;
      break;

    case State::HitBottomLimit:
      z_motor_driver_.disable();
      state_ = State::Idle;
      break;

    case State::Fault:
      z_motor_driver_.disable();
      break;
  }
}

void TaskZMotor::update() {
  z_motor_driver_.update();
}

TaskZMotor::State TaskZMotor::getState() const {
  return state_;
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