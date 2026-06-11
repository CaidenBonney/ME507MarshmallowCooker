#ifndef TASK_TEMPS_H
#define TASK_TEMPS_H

// Parent class include
#include "Task.h"

// User created includes
#include "MCP9600.h" // Thermocouple temperature sensor
#include "MLX90614.h" // Infrared temperature sensor

// Additional includes
#include "stdio.h" // For sprintf
#include <cstdlib> // For abs

// Externs

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

  bool hasValidThermocoupleReading() const;
  bool hasValidIrReading() const;

  int32_t getThermocoupleHotFx100() const;
  int32_t getThermocoupleColdFx100() const;
  int32_t getIrObjectFx100() const;
  void printTemperatures() const;

private:
  State state_ = State::Uninitialized;

  MCP9600 tc_sensor_;
  MLX90614 ir_sensor_;

  static constexpr uint32_t kUpdatePeriodMs = 500;
  uint32_t last_update_ms_ = 0;

  bool valid_tc_reading_ = false;
  bool valid_ir_reading_ = false;

  int32_t tc_hot_fx100_ = 0;
  int32_t tc_cold_fx100_ = 0;
  int32_t ir_object_fx100_ = 0;
};

#endif /* TASK_TEMPS_H */
