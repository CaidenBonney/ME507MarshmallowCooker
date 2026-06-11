#ifndef R_MOTOR_DRIVER_H
#define R_MOTOR_DRIVER_H

/**
 * @file r_motor_driver.h
 * @brief Position and duty control driver for the R-axis gearmotor.
 */

#include "DRV8833.h"
#include "r_encoder_driver.h"
#include <cstdint>

/**
 * @brief Non-blocking driver for the rotisserie DC gearmotor.
 *
 * The driver combines a DRV8833 motor driver with an encoder to support
 * duty-control, relative degree moves, and absolute degree moves.
 */
class RMotorDriver {
public:
  /** @brief Low-level R motor driver state. */
  enum class State {
    Disabled,           ///< Driver has not been initialized.
    Idle,               ///< Motor is stopped and ready.
    MovingToPosition,   ///< Position move is active.
    RotatingContinuous, ///< Open-loop continuous rotation is active.
    Fault               ///< Driver fault or timeout occurred.
  };

  /** @brief Construct the R motor driver with project timer assignments. */
  RMotorDriver();

  /** @brief Initialize the encoder and DRV8833 driver. */
  HAL_StatusTypeDef begin();

  /** @brief Run one non-blocking control update. */
  void update();

  /** @brief Set signed DRV8833 duty command. */
  void setDuty(int16_t duty);

  /** @brief Actively brake the motor. */
  void brake();

  /** @brief Coast the motor output. */
  void coast();

  /** @brief Brake the motor and set the driver state to Idle. */
  void stop();

  /** @brief Move by a relative number of output shaft degrees. */
  void moveDegrees(int32_t degrees, int16_t duty, uint32_t timeout_ms);

  /** @brief Move to an absolute output shaft angle in degrees. */
  void moveToDegrees(int32_t target_degrees, int16_t duty, uint32_t timeout_ms);

  /** @brief Start open-loop continuous rotation at a signed duty command. */
  void rotateContinuous(int16_t duty);

  /** @brief Get the latest encoder velocity estimate. */
  int16_t getVelocity() const;

  /** @brief Get the latest encoder position in counts. */
  int32_t getPosition() const;

  /** @brief Get the latest encoder position converted to degrees. */
  int32_t getPositionDegrees() const;

  /** @brief Reset encoder position and velocity accumulation to zero. */
  void resetEncoder();

  /** @brief Return true while a move or continuous rotation is active. */
  bool isBusy() const;

  /** @brief Return true if the driver is faulted. */
  bool isFaulted() const;

  /** @brief Get the current low-level driver state. */
  State getState() const;

  /** @brief Blocking relative move helper retained for bring-up tests. */
  void moveDegreesBlocking(int32_t degrees, int16_t duty, uint32_t timeout_ms);

private:
  static constexpr int32_t kCountsPerOutputRev = 3626;     ///< Encoder counts per output revolution.
  static constexpr int16_t kMinMovingPower = 350;          ///< Minimum useful motor command.
  static constexpr int16_t kMaxPower = 1000;               ///< Maximum absolute motor command.
  static constexpr int32_t kPositionToleranceCounts = 8;   ///< Position move tolerance.

  /** @brief Convert output shaft degrees to encoder counts. */
  int32_t degreesToCounts(int32_t degrees) const;

  /** @brief Convert encoder counts to output shaft degrees. */
  int32_t countsToDegrees(int32_t counts) const;

  /** @brief Clamp a signed motor command to the allowed range. */
  int16_t clampPower(int32_t power) const;

  DRV8833 driver_;
  REncoderDriver encoder_;

  State state_ = State::Disabled;

  int32_t target_counts_ = 0;
  int32_t move_start_counts_ = 0;
  int16_t commanded_duty_ = 0;
  uint32_t move_start_ms_ = 0;
  uint32_t move_timeout_ms_ = 0;
};

#endif /* R_MOTOR_DRIVER_H */
