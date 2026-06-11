#ifndef TASK_TEMPS_H
#define TASK_TEMPS_H

// Parent class include
#include "Task.h"

// User created includes
#include "MCP9600.h" // Thermocouple temperature sensor
#include "MLX90614.h" // Infrared temperature sensor

// additional includes
#include "stdio.h" // For sprintf
#include "stm32f4xx_hal.h"  // For HAL_GetTick

// externs
extern void print_str(const char* str);
extern char print_buf[100];

class TaskTemps : public Task {
public:
  TaskTemps();

  void run();
  void update();

private:
  // TODO: add enum for init then run state
  static constexpr uint32_t kUpdatePeriodMs = 500;

  MCP9600 tc_sensor_;
  MLX90614 ir_sensor_;
  uint32_t last_update_ms_;
};

#endif /* TASK_TEMPS_H */