#ifndef TASK_Z_MOTOR_H
#define TASK_Z_MOTOR_H

// Parent class include
#include "Task.h"

// User created includes
#include "Z_Limit_Switches.h"
#include "z_motor_driver.h"

// additional includes
#include "stm32f4xx_hal.h"

// externs
extern void Error_Handler();
extern void print_str(const char* str);

class TaskZMotor : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    MovingUp,
    MovingDown,
    HomingDown,
    HitTopLimit,
    HitBottomLimit,
    Fault
  };

  TaskZMotor();

  void run() override;
  void update();

  Status getStatus() const override;
  State getState() const;

private:
  static constexpr uint32_t kUpdatePeriodMs = 10;

  State state_ = State::Uninitialized;

  ZMotorDriver z_motor_driver_;
  ZLimitSwitches z_limit_switches_;

  uint32_t last_update_ms_ = 0;
  bool test_move_started_ = false;
};

#endif /* TASK_Z_MOTOR_H */