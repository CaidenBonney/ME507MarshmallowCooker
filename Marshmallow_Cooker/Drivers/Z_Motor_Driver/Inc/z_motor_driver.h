#ifndef Z_MOTOR_DRIVER_H
#define Z_MOTOR_DRIVER_Hƒ

/**
 * @file z_motor_driver.h
 * @brief Position driver for the Z-axis TMC2209 stepper motor.
 * @details
 *   ZMotorDriver tracks the software position of the vertical axis and issues
 *   step/direction commands to a TMC2209 stepper driver. It supports relative
 *   moves, absolute moves, jogging, homing, limit switch safety checks, and
 *   direction inversion for mechanical bring-up.
 */

#include "TMC2209.h" // PWM driver for Z motor
#include "stm32f4xx_hal.h"
#include <cstdint>

/**
 * @class ZMotorDriver
 * @brief Low-level stepper position and limit-switch driver for the Z axis.
 * @details
 *   The driver uses the TMC2209 wrapper to generate step pulses and maintains a
 *   signed position count in microsteps. Home is normally established at the top
 *   limit switch as 0 steps. Upward motion is positive and downward motion is
 *   negative in the surrounding task code.
 */
class ZMotorDriver {
public:
  /**
   * @enum Direction
   * @brief Logical Z-axis motion direction.
   */
  enum class Direction {
    Up, /**< Motion toward the top limit switch and away from the flame. */
    Down /**< Motion toward the bottom limit switch and flame. */
  };

  /**
   * @enum State
   * @brief Internal Z motor driver state.
   */
  enum class State {
    Disabled, /**< Driver has not been enabled. */
    Idle, /**< Driver is enabled but no motion command is active. */
    Moving, /**< A relative, absolute, jog, or home move is active. */
    Fault, /**< Driver fault or unsafe condition has stopped motion. */
    HitTopLimit, /**< Motion stopped because the top limit was reached. */
    HitBottomLimit /**< Motion stopped because the bottom limit was reached. */
  };

  /** @brief Construct a Z motor driver using project-specific GPIO pins. */
  ZMotorDriver();

  /** @brief Initialize the TMC2209 driver and internal state. */
  void begin();

  /** @brief Enable the TMC2209 outputs for motion. */
  void enable();

  /** @brief Disable the TMC2209 outputs and stop motion. */
  void disable();

  /**
   * @brief Service active stepper motion.
   * @details
   *   This method should be called frequently. It checks whether a step is due,
   *   verifies limits, issues at most one step, and updates software position.
   */
  void update();

  /**
   * @brief Command a relative move.
   * @param steps Signed step delta. Positive moves up, negative moves down.
   */
  void moveSteps(int32_t steps);

  /**
   * @brief Command an absolute move.
   * @param target_position_steps Target position in software steps.
   */
  void moveTo(int32_t target_position_steps);

  /**
   * @brief Jog continuously in one direction.
   * @param direction Direction to jog.
   * @param speed_steps_per_second Jog step rate.
   */
  void jog(Direction direction, uint32_t speed_steps_per_second);

  /** @brief Stop active motion and leave the driver idle. */
  void stop();

  /**
   * @brief Start a homing move in the requested direction.
   * @param direction Limit direction used for homing.
   * @param speed_steps_per_second Homing step rate.
   * @details
   *   For this project, Direction::Up is used so the top limit becomes Z = 0.
   */
  void home(Direction direction, uint32_t speed_steps_per_second);

  /**
   * @brief Start homing upward toward the top limit switch.
   * @param speed_steps_per_second Homing step rate.
   */
  void homeUp(uint32_t speed_steps_per_second);

  /**
   * @brief Start homing downward toward the bottom limit switch.
   * @param speed_steps_per_second Homing step rate.
   */
  void homeDown(uint32_t speed_steps_per_second);

  /** @brief Set the current software position to zero. */
  void zeroPosition();

  /**
   * @brief Set the step rate used for future moves.
   * @param steps_per_second Desired step rate.
   */
  void setSpeed(uint32_t steps_per_second);

  /**
   * @brief Set the step rate used for future moves.
   * @param steps_per_second Desired step rate.
   */
  void setSpeedStepsPerSecond(uint32_t steps_per_second);

  /**
   * @brief Store the configured microstep setting.
   * @param microsteps Microsteps per full step used by the hardware driver.
   */
  void setMicrosteps(uint16_t microsteps);

  /**
   * @brief Configure limit switch polarity.
   * @param active_low true if pressed switches read GPIO_PIN_RESET.
   */
  void setLimitsActiveLow(bool active_low);

  /**
   * @brief Configure motor direction inversion.
   * @param inverted true to swap the electrical direction used for logical up/down.
   */
  void setDirectionInverted(bool inverted);

  /**
   * @brief Get the current software position.
   * @return Position in steps relative to the most recent zeroPosition().
   */
  int32_t getPositionSteps() const;

  /**
   * @brief Get the active target position.
   * @return Target position in steps.
   */
  int32_t getTargetSteps() const;

  /**
   * @brief Get the internal driver state.
   * @return Current ZMotorDriver::State value.
   */
  State getState() const;

  /**
   * @brief Check whether a motion command is active.
   * @return true if the driver is currently moving.
   */
  bool isBusy() const;

  /**
   * @brief Check whether the logical top limit is pressed.
   * @return true when the top limit input is active after polarity handling.
   */
  bool topLimitPressed() const;

  /**
   * @brief Check whether the logical bottom limit is pressed.
   * @return true when the bottom limit input is active after polarity handling.
   */
  bool bottomLimitPressed() const;

  /**
   * @brief Check whether the TMC2209 diagnostic pin indicates a fault.
   * @return true when the TMC2209 diagnostic input is active.
   */
  bool driverFaultActive() const;

private:
  /** @brief Default step rate used before a task configures speed. */
  static constexpr uint32_t kDefaultSpeedStepsPerSecond = 250;

  /** @brief Default microstep setting assumed by the software. */
  static constexpr uint16_t kDefaultMicrosteps = 8;

  /** @brief Full motor steps per revolution before microstepping. */
  static constexpr int32_t kFullStepsPerRev = 200;

  /**
   * @brief Set the electrical driver direction for a logical motion direction.
   * @param direction Logical direction to apply.
   */
  void setDirection(Direction direction);

  /**
   * @brief Attempt one step in the requested direction.
   * @param direction Logical direction for the step.
   * @return true if a step was emitted and position was updated.
   */
  bool tryStep(Direction direction);

  /**
   * @brief Check whether a limit switch prevents motion in a direction.
   * @param direction Direction being requested.
   * @return true if motion in direction is blocked by an active limit.
   */
  bool limitBlocksDirection(Direction direction) const;

  /**
   * @brief Check whether the current state represents an active motion command.
   * @return true when a move, jog, or home command is active.
   */
  bool motionCommandActive() const;

  /** @brief TMC2209 step/direction driver wrapper. */
  TMC2209 tmc_;

  /** @brief Current software position in steps. */
  int32_t position_steps_;

  /** @brief Target software position in steps. */
  int32_t target_steps_;

  /** @brief Requested step rate. */
  uint32_t speed_steps_per_second_;

  /** @brief Stored microstep setting. */
  uint16_t microsteps_;

  /** @brief True when a homing command is active. */
  bool homing_;

  /** @brief Direction used by the active homing command. */
  Direction homing_direction_;

  /** @brief true when limit switches are wired active-low. */
  bool limits_active_low_;

  /** @brief Last logical direction commanded to the driver. */
  Direction current_direction_;

  /** @brief Current internal driver state. */
  State state_;
};

#endif /* Z_MOTOR_DRIVER_H */
