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

        print_str("Move +360 degrees\r\n");

        /*
         * Your current driver has moveDegreesBlocking().
         * This still blocks, but now it only happens once.
         * Later, this should become a non-blocking move if your
         * RMotorDriver supports update() and isBusy().
         */
        r_motor_driver_.moveDegreesBlocking(360, 1000, 8000);

        sprintf(print_buf,
                "Done +360: counts=%ld deg=%ld\r\n",
                static_cast<long>(r_motor_driver_.getPosition()),
                static_cast<long>(r_motor_driver_.getPositionDegrees()));
        print_str(print_buf);

        state_ = State::MovingNegative;
      }
      break;

    case State::MovingPositive:
      /*
       * Placeholder for future non-blocking R motor movement.
       */
      state_ = State::Idle;
      break;

    case State::MovingNegative:
      print_str("Move -360 degrees\r\n");

      r_motor_driver_.moveDegreesBlocking(-360, 1000, 8000);

      sprintf(print_buf,
              "Done -360: counts=%ld deg=%ld\r\n",
              static_cast<long>(r_motor_driver_.getPosition()),
              static_cast<long>(r_motor_driver_.getPositionDegrees()));
      print_str(print_buf);

      state_ = State::Idle;
      break;

    case State::Fault:
      break;
  }
}

void TaskRMotor::update() {
  /*
   * Add this later if RMotorDriver gets a non-blocking update function.
   */
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