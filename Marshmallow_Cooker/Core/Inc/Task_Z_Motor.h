#ifndef TASK_Z_MOTOR_H
#define TASK_Z_MOTOR_H

// Parent class include
#include "Task.h"

// User created includes
#include "Z_Limit_Switches.h" // Limit switch driver for z-axis
#include "z_motor_driver.h" // Motor driver for veritcal stepper motor (z-axis)

// externs
extern void print_str(const char* str);

class TaskZMotor : public Task {
public:
  TaskZMotor();

  void run();
  void update();

private:
  // TODO: add enum for init then run state

  static constexpr uint32_t kUpdatePeriodMs = 10;

  ZMotorDriver z_motor_driver_;
  ZLimitSwitches z_limit_switches_;
  uint32_t last_update_ms_;
};

#endif /* TASK_Z_MOTOR_H */