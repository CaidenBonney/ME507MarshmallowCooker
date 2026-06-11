#include "Task_R_Motor.h"

TaskRMotor::TaskRMotor()
    : r_motor_driver_() {
}

void TaskRMotor::run() {
  // TODO: use state enum type to track state

  // Initialize R Motor
  if (r_motor_driver_.begin() != HAL_OK) {
    Error_Handler();
  }
  print_str("R motor driver initialized\r\n");


  // ===============================  Testing R Motor ===============================
  // Test R motor by moving +360 degrees, then -360 degrees
  print_str("Move +360 degrees\r\n");

  r_motor_driver_.moveDegreesBlocking(360, 1000, 8000);

  sprintf(print_buf,
          "Done +360: counts=%ld deg=%ld\r\n",
          static_cast<long>(r_motor_driver_.getPosition()),
          static_cast<long>(r_motor_driver_.getPositionDegrees()));
  print_str(print_buf);

  HAL_Delay(2000);

  print_str("Move -360 degrees\r\n");

  r_motor_driver_.moveDegreesBlocking(-360, 1000, 8000);

  sprintf(print_buf,
          "Done -360: counts=%ld deg=%ld\r\n",
          static_cast<long>(r_motor_driver_.getPosition()),
          static_cast<long>(r_motor_driver_.getPositionDegrees()));
  print_str(print_buf);
}

void TaskRMotor::update() {
  // TODO: make non-blocking update during motor movement
}