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

  switch (state_) {
    case State::Uninitialized:
      if (r_motor_driver_.begin() != HAL_OK) {
        state_ = State::Fault;
        Error_Handler();
        return;
      }

      print_str("R motor driver initialized\r\n");
      state_ = State::Idle;
      break;

    case State::Idle:
      if (!test_move_started_) {
        test_move_started_ = true;

        print_str("R move +360 degrees\r\n");
        r_motor_driver_.moveDegrees(360, 1000, 8000);

        state_ = State::MovingPositive;
      }
      break;

    case State::MovingPositive:
      r_motor_driver_.update();

      if (r_motor_driver_.isFaulted()) {
        print_str("R motor fault during +360\r\n");
        state_ = State::Fault;
      } else if (!r_motor_driver_.isBusy()) {
        sprintf(print_buf,
                "Done +360: counts=%ld deg=%ld\r\n",
                static_cast<long>(r_motor_driver_.getPosition()),
                static_cast<long>(r_motor_driver_.getPositionDegrees()));
        print_str(print_buf);

        print_str("R move -360 degrees\r\n");
        r_motor_driver_.moveDegrees(-360, 1000, 8000);

        state_ = State::MovingNegative;
      }
      break;

    case State::MovingNegative:
      r_motor_driver_.update();

      if (r_motor_driver_.isFaulted()) {
        print_str("R motor fault during -360\r\n");
        state_ = State::Fault;
      } else if (!r_motor_driver_.isBusy()) {
        sprintf(print_buf,
                "Done -360: counts=%ld deg=%ld\r\n",
                static_cast<long>(r_motor_driver_.getPosition()),
                static_cast<long>(r_motor_driver_.getPositionDegrees()));
        print_str(print_buf);

        state_ = State::Idle;
      }
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