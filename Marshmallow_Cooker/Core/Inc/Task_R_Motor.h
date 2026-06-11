#ifndef TASK_R_MOTOR_H
#define TASK_R_MOTOR_H

// Parent class include
#include "Task.h"

// User created includes
#include "r_motor_driver.h" // Motor driver for rotating motor

// additional includes
#include "stdio.h" // For sprintf

// externs
extern void Error_Handler();
extern void print_str(const char* str);
extern char print_buf[100];

class TaskRMotor : public Task {
public:
  TaskRMotor();

  void run();
  void update();

private:
  // TODO: add enum for init then run state

  static constexpr uint32_t kUpdatePeriodMs = 10;

  RMotorDriver r_motor_driver_;
  uint32_t last_update_ms_;
};

#endif /* TASK_R_MOTOR_H */