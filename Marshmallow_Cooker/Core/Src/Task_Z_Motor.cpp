#include "Task_Z_Motor.h"

ZLimitSwitches z_limit_switches(Z_TOP_GPIO_Port, Z_TOP_Pin, Z_BOT_GPIO_Port, Z_BOT_Pin);

TaskZMotor::TaskZMotor()
    : z_motor_driver_(),
      z_limit_switches_(Z_TOP_GPIO_Port, Z_TOP_Pin, Z_BOT_GPIO_Port, Z_BOT_Pin) {
}

void TaskZMotor::run() {
  // TODO: use state enum type to track state

  // Initialize Z Motor
  if (z_motor_driver_.begin() != HAL_OK) {
    Error_Handler();
  }
  print_str("Z motor driver initialized\r\n");


  // ===============================  Testing Z Motor ===============================
  if (z_limit_switches_.isTopTriggered()) {
    print_str("TOP LIMIT\r\n");
  }

  if (z_limit_switches_.isBottomTriggered()) {
    print_str("BOTTOM LIMIT\r\n");
  }

  // Z MOTOR
  z_motor_driver_.begin();
  z_motor_driver_.enable();

  z_motor_driver_.setSpeedStepsPerSecond(500);
  z_motor_driver_.moveSteps(1600);  // rotates 360 degrees (1600 steps with 1/16 microstepping)

  while (z_motor_driver_.isBusy()) {
    z_motor_driver_.update();
  }

  HAL_Delay(1000);

  z_motor_driver_.moveSteps(-1600);  // rotates -360 degrees (-1600 steps with 1/16 microstepping)

  update();

  z_motor_driver_.disable();
}

void TaskZMotor::update() {
  // TODO: make non-blocking update during motor movement
  while (z_motor_driver_.isBusy()) {
    z_motor_driver_.update();
  }
}