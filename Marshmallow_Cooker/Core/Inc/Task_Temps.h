#ifndef TASK_TEMPS_H
#define TASK_TEMPS_H

/**
 * @file Task_Temps.h
 * @brief Cooperative task for thermocouple and IR temperature sensing.
 * @details
 *   TaskTemps owns the MCP9600 thermocouple amplifier and MLX90614 infrared
 *   thermometer drivers. It periodically refreshes both sensors, caches the
 *   most recent valid readings, and exposes fixed-point Fahrenheit values to
 *   the cooker control task.
 */

#include "MCP9600.h" // Thermocouple temperature sensor
#include "MLX90614.h" // Infrared temperature sensor
#include "Task.h"

#include "stdio.h" // For sprintf
#include <cstdlib> // For abs

/**
 * @class TaskTemps
 * @brief Periodic temperature acquisition task.
 * @details
 *   The task initializes the sensor drivers, reads the thermocouple hot/cold
 *   junction temperatures, and reads the IR object temperature. Readings are
 *   cached so other tasks can query values without directly blocking on I2C.
 */
class TaskTemps : public Task {
public:
  /**
   * @enum State
   * @brief Detailed temperature task state.
   */
  enum class State {
    Uninitialized, /**< Sensors have not yet been initialized. */
    Idle, /**< Task is initialized and waiting for the next sample period. */
    Reading, /**< A sensor update is in progress. */
    Fault /**< Sensor initialization or read failure forced a task fault. */
  };

  /** @brief Construct the temperature task and embedded sensor drivers. */
  TaskTemps();

  /** @brief Last HAL status observed during sensor communication. */
  volatile HAL_StatusTypeDef status = HAL_ERROR;

  /**
   * @brief Execute one non-blocking temperature task update.
   * @details
   *   Initializes sensors on first run and then refreshes cached temperature
   *   values at kUpdatePeriodMs. Successful reads mark the associated validity
   *   flag so the cooker can ignore stale or unavailable sensors.
   */
  void run() override;

  /** @brief Compatibility wrapper that calls run(). */
  void update();

  /**
   * @brief Get generic task health status.
   * @return Uninitialized before sensor setup, Fault on sensor fault, otherwise Running.
   */
  Status getStatus() const override;

  /**
   * @brief Get the detailed temperature task state.
   * @return Current TaskTemps::State value.
   */
  State getState() const;

  /**
   * @brief Check whether a valid thermocouple reading has been cached.
   * @return true when thermocouple hot/cold values are available.
   */
  bool hasValidThermocoupleReading() const;

  /**
   * @brief Check whether a valid infrared object reading has been cached.
   * @return true when an IR object temperature is available.
   */
  bool hasValidIrReading() const;

  /**
   * @brief Get the cached thermocouple hot-junction temperature.
   * @return Hot-junction temperature in degrees F x100.
   */
  int16_t getThermocoupleHotFx100() const;

  /**
   * @brief Get the cached thermocouple cold-junction temperature.
   * @return Cold-junction temperature in degrees F x100.
   */
  int16_t getThermocoupleColdFx100() const;

  /**
   * @brief Get the cached IR object temperature.
   * @return IR object temperature in degrees F x100.
   */
  int16_t getIrObjectFx100() const;

private:
  /** @brief Current detailed state of the temperature task. */
  State state_ = State::Uninitialized;

  /** @brief Thermocouple amplifier driver. */
  MCP9600 tc_sensor_;

  /** @brief Non-contact IR thermometer driver. */
  MLX90614 ir_sensor_;

  /** @brief Time between sensor refresh attempts in milliseconds. */
  static constexpr uint32_t kUpdatePeriodMs = 500;

  /** @brief HAL tick timestamp of the most recent update attempt. */
  uint32_t last_update_ms_ = 0;

  /** @brief True after at least one successful thermocouple update. */
  bool valid_tc_reading_ = false;

  /** @brief True after at least one successful IR update. */
  bool valid_ir_reading_ = false;

  /** @brief Cached hot-junction temperature in degrees F x100. */
  int16_t tc_hot_fx100_ = 0;

  /** @brief Cached cold-junction temperature in degrees F x100. */
  int16_t tc_cold_fx100_ = 0;

  /** @brief Cached IR object temperature in degrees F x100. */
  int16_t ir_object_fx100_ = 0;
};

#endif /* TASK_TEMPS_H */
