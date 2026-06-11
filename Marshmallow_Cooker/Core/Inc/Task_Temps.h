#ifndef TASK_TEMPS_H
#define TASK_TEMPS_H

// Parent class include
#include "Task.h"

// User created includes
#include "MCP9600.h"
#include "MLX90614.h"

// additional includes
#include "stdio.h"
#include "stm32f4xx_hal.h"
#include <cstdlib>

// externs
extern void print_str(const char* str);
extern char print_buf[100];

class TaskTemps : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    Reading,
    Fault
  };

  TaskTemps();

  volatile HAL_StatusTypeDef status = HAL_ERROR;

  void run() override;
  void update();

  Status getStatus() const override;
  State getState() const;

private:
  State state_ = State::Uninitialized;

  MCP9600 tc_sensor_;
  MLX90614 ir_sensor_;

  static constexpr uint32_t kUpdatePeriodMs = 500;
  uint32_t last_update_ms_ = 0;
};

#endif /* TASK_TEMPS_H */