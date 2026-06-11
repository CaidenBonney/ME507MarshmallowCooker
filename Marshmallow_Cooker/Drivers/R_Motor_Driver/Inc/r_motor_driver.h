#ifndef R_MOTOR_DRIVER_H
#define R_MOTOR_DRIVER_H

/**
 * @file r_motor_driver.h
 * @brief Closed-loop position driver for the rotisserie DC gearmotor.
 * @details
 *   RMotorDriver combines a DRV8833 H-bridge driver with a quadrature encoder
 *   driver to provide non-blocking relative and absolute angular moves. The
 *   driver uses a simple bang-bang position controller with signed duty output,
 *   timeout protection, braking on arrival, and encoder position conversion
 *   between counts and output-shaft degrees.
 */

#include "DRV8833.h"
#include "r_encoder_driver.h"

/**
 * @class RMotorDriver
 * @brief Low-level driver for the R-axis brushed DC motor and encoder.
 * @details
 *   The class owns the PWM H-bridge object and encoder object. Tasks command
 *   motion using moveDegrees(), moveToDegrees(), or rotateContinuous(); repeated
 *   calls to update() refresh encoder data and drive the motor toward the active
 *   target until it reaches tolerance or times out.
 */
class RMotorDriver {
public:
  /**
   * @enum State
   * @brief Internal R motor driver state.
   */
  enum class State {
    Disabled, /**< Driver has not been initialized. */
    Idle, /**< Motor is stopped and ready. */
    MovingToPosition, /**< Closed-loop move toward target_counts_ is active. */
    RotatingContinuous, /**< Open-ended duty command is active. */
    Fault /**< Driver timed out or failed initialization. */
  };

  /** @brief Construct the R motor driver using project-specific timers. */
  RMotorDriver();

  /**
   * @brief Initialize the encoder and DRV8833 driver.
   * @return HAL_OK when PWM outputs start successfully, otherwise HAL_ERROR.
   */
  HAL_StatusTypeDef begin();

  /**
   * @brief Refresh encoder data and service the active motor state.
   * @details
   *   This method must be called periodically. During position moves it checks
   *   timeout, computes position error, applies the proper signed duty, and
   *   brakes when the target is reached.
   */
  void update();

  /**
   * @brief Apply a signed duty command directly to the H-bridge.
   * @param duty Signed duty command. Positive and negative values rotate opposite directions.
   */
  void setDuty(int16_t duty);

  /** @brief Actively brake the motor. */
  void brake();

  /** @brief Coast the motor by disabling active drive. */
  void coast();

  /** @brief Stop the motor and return the driver to Idle. */
  void stop();

  /**
   * @brief Start a non-blocking relative angular move.
   * @param degrees Relative output-shaft move in degrees.
   * @param duty Drive magnitude used during the move.
   * @param timeout_ms Maximum move time before a driver fault is declared.
   */
  void moveDegrees(int32_t degrees, int16_t duty, uint32_t timeout_ms);

  /**
   * @brief Start a non-blocking absolute angular move.
   * @param target_degrees Absolute encoder-referenced target in output-shaft degrees.
   * @param duty Drive magnitude used during the move.
   * @param timeout_ms Maximum move time before a driver fault is declared.
   */
  void moveToDegrees(int32_t target_degrees, int16_t duty, uint32_t timeout_ms);

  /**
   * @brief Command continuous rotation at a signed duty.
   * @param duty Signed duty command. Passing zero stops the motor.
   */
  void rotateContinuous(int16_t duty);

  /**
   * @brief Get the most recent encoder velocity estimate.
   * @return Encoder velocity in counts per update period as provided by REncoderDriver.
   */
  int16_t getVelocity() const;

  /**
   * @brief Get the current encoder position.
   * @return Position in encoder counts.
   */
  int32_t getPosition() const;

  /**
   * @brief Get the current output-shaft position.
   * @return Position in output-shaft degrees relative to the encoder zero.
   */
  int32_t getPositionDegrees() const;

  /** @brief Reset the encoder count to zero. */
  void resetEncoder();

  /**
   * @brief Check whether a move or continuous rotation is active.
   * @return true when the driver state indicates active motion.
   */
  bool isBusy() const;

  /**
   * @brief Check whether the driver is faulted.
   * @return true when the driver state is Fault.
   */
  bool isFaulted() const;

  /**
   * @brief Get the internal driver state.
   * @return Current RMotorDriver::State value.
   */
  State getState() const;

  /**
   * @brief Perform a blocking relative move for bench testing.
   * @param degrees Relative output-shaft move in degrees.
   * @param duty Drive magnitude used during the move.
   * @param timeout_ms Maximum blocking move time.
   * @details
   *   This helper is retained for temporary testing only. Cooperative tasks
   *   should use the non-blocking moveDegrees() and update() pattern.
   */
  void moveDegreesBlocking(int32_t degrees, int16_t duty, uint32_t timeout_ms);

private:
  /** @brief Encoder counts per output-shaft revolution after gearbox reduction. */
  static constexpr int32_t kCountsPerOutputRev = 3626;

  /** @brief Minimum useful moving power retained for future control improvements. */
  static constexpr int16_t kMinMovingPower = 350;

  /** @brief Maximum allowed signed power command. */
  static constexpr int16_t kMaxPower = 1000;

  /** @brief Position error band that counts as target reached. */
  static constexpr int32_t kPositionToleranceCounts = 8;

  /**
   * @brief Convert output-shaft degrees to encoder counts.
   * @param degrees Output-shaft angle in degrees.
   * @return Equivalent encoder count delta.
   */
  int32_t degreesToCounts(int32_t degrees) const;

  /**
   * @brief Convert encoder counts to output-shaft degrees.
   * @param counts Encoder count value.
   * @return Equivalent output-shaft angle in degrees.
   */
  int32_t countsToDegrees(int32_t counts) const;

  /**
   * @brief Clamp a signed power command to the supported duty range.
   * @param power Raw signed power command.
   * @return Clamped signed duty command.
   */
  int16_t clampPower(int32_t power) const;

  /** @brief DRV8833 H-bridge PWM driver. */
  DRV8833 driver_;

  /** @brief Quadrature encoder driver. */
  REncoderDriver encoder_;

  /** @brief Current driver state. */
  State state_ = State::Disabled;

  /** @brief Absolute target position for the active move in encoder counts. */
  int32_t target_counts_ = 0;

  /** @brief Encoder count at the beginning of the active move. */
  int32_t move_start_counts_ = 0;

  /** @brief Magnitude of duty command used during the active move. */
  int16_t commanded_duty_ = 0;

  /** @brief HAL tick timestamp when the active move began. */
  uint32_t move_start_ms_ = 0;

  /** @brief Timeout for the active move in milliseconds. */
  uint32_t move_timeout_ms_ = 0;
};

#endif /* R_MOTOR_DRIVER_H */
