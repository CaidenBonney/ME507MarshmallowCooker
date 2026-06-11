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
    RotatingForward,
    RotatingBackward,
    Stopping,
    Fault
  };

  TaskRMotor();

  void run();
  void update();

  void startCookingRotation();
  void stopCookingRotation();
  void emergencyStop();

  bool isBusy() const;
  State getState() const;

private:
  static constexpr uint32_t kUpdatePeriodMs = 10;
  static constexpr int32_t kCookRotationDegrees = 360;
  static constexpr int16_t kDefaultCookDuty = 700;
  static constexpr uint32_t kMoveTimeoutMs = 8000;

  State state_ = State::Uninitialized;

  RMotorDriver r_motor_driver_;

  uint32_t last_update_ms_ = 0;

  bool cooking_rotation_requested_ = false;
  bool stop_requested_ = false;
  bool emergency_stop_requested_ = false;

  int16_t cook_duty_ = kDefaultCookDuty;
};

#endif /* TASK_R_MOTOR_H */