#ifndef Z_MOTOR_DRIVER_H
#define Z_MOTOR_DRIVER_H

/**
 * @file z_motor_driver.h
 * @brief Non-blocking Z-axis stepper motor driver wrapper around a TMC2209.
 */

#include "TMC2209.h"
#include "stm32f4xx_hal.h"
#include <cstdint>

/**
 * @brief Position-mode and homing driver for the Z-axis stepper motor.
 *
 * The driver tracks software position in steps, checks top and bottom limit
 * switches, and emits step pulses through the TMC2209 helper.
 */
class ZMotorDriver {
public:
  /** @brief Logical Z motion directions. */
  enum class Direction {
    Up, ///< Move toward the top/home limit.
    Down ///< Move toward the bottom/flame side.
  };

  /** @brief Low-level Z driver state. */
  enum class State {
    Disabled, ///< Driver output is disabled.
    Idle, ///< Driver is enabled but not moving.
    Moving, ///< A position or homing move is active.
    Fault, ///< Driver fault input is active.
    HitTopLimit, ///< Motion stopped because the top limit was hit.
    HitBottomLimit ///< Motion stopped because the bottom limit was hit.
  };

  /** @brief Construct the Z motor driver with project pin assignments. */
  ZMotorDriver();

  /** @brief Initialize GPIO, direction, speed, and state defaults. */
  void begin();

  /** @brief Enable the TMC2209 motor output. */
  void enable();

  /** @brief Disable the TMC2209 motor output. */
  void disable();

  /** @brief Run one non-blocking driver update and emit a step if due. */
  void update();

  /** @brief Move by a signed number of steps relative to current position. */
  void moveSteps(int32_t steps);

  /** @brief Move to an absolute software step position. */
  void moveTo(int32_t target_position_steps);

  /** @brief Begin continuous jogging in one direction. */
  void jog(Direction direction, uint32_t speed_steps_per_second);

  /** @brief Stop motion and enter Idle. */
  void stop();

  /** @brief Start homing in the requested direction. */
  void home(Direction direction, uint32_t speed_steps_per_second);

  /** @brief Convenience wrapper for upward homing. */
  void homeUp(uint32_t speed_steps_per_second);

  /** @brief Convenience wrapper for downward homing. */
  void homeDown(uint32_t speed_steps_per_second);

  /** @brief Set the current software position and target to zero. */
  void zeroPosition();

  /** @brief Set step speed in steps per second. */
  void setSpeed(uint32_t steps_per_second);

  /** @brief Set step speed in steps per second. */
  void setSpeedStepsPerSecond(uint32_t steps_per_second);

  /** @brief Store the configured microstep setting. */
  void setMicrosteps(uint16_t microsteps);

  /** @brief Configure whether limit switches are active-low. */
  void setLimitsActiveLow(bool active_low);

  /** @brief Configure whether direction output is inverted. */
  void setDirectionInverted(bool inverted);

  /** @brief Get the current software position in steps. */
  int32_t getPositionSteps() const;

  /** @brief Get the current target position in steps. */
  int32_t getTargetSteps() const;

  /** @brief Get the current driver state. */
  State getState() const;

  /** @brief Return true while a motion command is active. */
  bool isBusy() const;

  /** @brief Return true when the top limit is logically pressed. */
  bool topLimitPressed() const;

  /** @brief Return true when the bottom limit is logically pressed. */
  bool bottomLimitPressed() const;

  /** @brief Return true when the TMC2209 diagnostic input indicates a fault. */
  bool driverFaultActive() const;

private:
  static constexpr uint32_t kDefaultSpeedStepsPerSecond = 250; ///< Default motion speed.
  static constexpr uint16_t kDefaultMicrosteps = 8; ///< Default microstep setting.
  static constexpr int32_t kFullStepsPerRev = 200; ///< Full steps per motor revolution.

  /** @brief Set the hardware direction pin for a logical direction. */
  void setDirection(Direction direction);

  /** @brief Attempt one step in the requested direction if safe. */
  bool tryStep(Direction direction);

  /** @brief Return true if a limit switch blocks the requested direction. */
  bool limitBlocksDirection(Direction direction) const;

  /** @brief Return true if the current state represents active motion. */
  bool motionCommandActive() const;

  TMC2209 tmc_;

  int32_t position_steps_;
  int32_t target_steps_;
  uint32_t speed_steps_per_second_;
  uint16_t microsteps_;

  bool homing_;
  Direction homing_direction_;
  bool limits_active_low_;

  Direction current_direction_;
  State state_;
};

#endif /* Z_MOTOR_DRIVER_H */
