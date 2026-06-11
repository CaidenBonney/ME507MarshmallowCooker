#ifndef TASK_R_MOTOR_H
#define TASK_R_MOTOR_H

// Parent class include
#include "Task.h"

// User created includes
#include "r_motor_driver.h"

// additional includes
#include "stdio.h"
#include "stm32f4xx_hal.h"

// externs
extern void Error_Handler();
extern void print_str(const char* str);
extern char print_buf[100];

class TaskRMotor : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    MovingPositive,
    MovingNegative,
    Fault
  };

  TaskRMotor();

  void run() override;
  void update();

  Status getStatus() const override;
  State getState() const;

private:
  static constexpr uint32_t kUpdatePeriodMs = 10;

  State state_ = State::Uninitialized;

  RMotorDriver r_motor_driver_;

  uint32_t last_update_ms_ = 0;
  bool test_move_started_ = false;
};

#endif /* TASK_R_MOTOR_H */