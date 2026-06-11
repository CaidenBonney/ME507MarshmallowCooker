#ifndef TASK_TEMPS_H
#define TASK_TEMPS_H

/**
 * @file Task_Temps.h
 * @brief Temperature sensor polling task for thermocouple and IR readings.
 */

#include "Task.h"
#include "MCP9600.h"   ///< Thermocouple temperature sensor driver.
#include "MLX90614.h"  ///< Infrared temperature sensor driver.

#include <cstdint>

/**
 * @brief Periodically reads temperature sensors and stores the latest values.
 *
 * Values are stored as fixed-point degrees Fahrenheit multiplied by 100. For
 * example, 160.00 F is represented as 16000.
 */
class TaskTemps : public Task {
public:
  /** @brief Internal sensor task state. */
  enum class State {
    Uninitialized,  ///< Sensors have not been initialized.
    Idle,           ///< Waiting for the next scheduled read.
    Reading,        ///< Reserved for sensor read-in-progress state.
    Fault           ///< Sensor task has faulted.
  };

  /** @brief Construct the temperature task. */
  TaskTemps();

  /** @brief Last HAL status reported by a sensor operation. */
  volatile HAL_StatusTypeDef status = HAL_ERROR;

  /** @copydoc Task::run */
  void run() override;

  /** @brief Compatibility wrapper for run(). */
  void update();

  /** @copydoc Task::getStatus */
  Status getStatus() const override;

  /** @brief Get the current temperature task state. */
  State getState() const;

  /** @brief Check whether the thermocouple reading is valid. */
  bool hasValidThermocoupleReading() const;

  /** @brief Check whether the IR object reading is valid. */
  bool hasValidIrReading() const;

  /** @brief Get the latest thermocouple hot-junction temperature in F x 100. */
  int32_t getThermocoupleHotFx100() const;

  /** @brief Get the latest thermocouple cold-junction temperature in F x 100. */
  int32_t getThermocoupleColdFx100() const;

  /** @brief Get the latest IR object temperature in F x 100. */
  int32_t getIrObjectFx100() const;

  /** @brief Print the latest stored temperatures over UART. */
  void printTemperatures() const;

private:
  State state_ = State::Uninitialized;

  MCP9600 tc_sensor_;
  MLX90614 ir_sensor_;

  static constexpr uint32_t kUpdatePeriodMs = 500;  ///< Sensor polling interval.
  uint32_t last_update_ms_ = 0;

  bool valid_tc_reading_ = false;
  bool valid_ir_reading_ = false;

  int32_t tc_hot_fx100_ = 0;
  int32_t tc_cold_fx100_ = 0;
  int32_t ir_object_fx100_ = 0;
};

#endif /* TASK_TEMPS_H */
