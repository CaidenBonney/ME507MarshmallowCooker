#ifndef TASK_R_MOTOR_H
#define TASK_R_MOTOR_H

/**
 * @file Task_R_Motor.h
 * @brief Cooperative task for the marshmallow rotisserie motor.
 * @details
 *   TaskRMotor wraps the low-level RMotorDriver with a cooking-oriented state
 *   machine. It commands alternating forward and reverse rotations so the
 *   marshmallow turns without continuously winding the thermocouple or sensor
 *   wiring. The task also handles normal stops, emergency stops, and software
 *   fault recovery.
 */

#include "Task.h"
#include "r_motor_driver.h" // Motor driver for rotating motor

#include "stdio.h" // For sprintf

/**
 * @class TaskRMotor
 * @brief State-machine task for the R-axis rotisserie motor.
 * @details
 *   The R motor task issues relative 360 degree moves through RMotorDriver and
 *   alternates direction whenever a move finishes. The task remains
 *   non-blocking; the driver is updated periodically from run() and all motor
 *   movement occurs through state transitions.
 */
class TaskRMotor : public Task {
public:
  /**
   * @enum State
   * @brief Detailed rotisserie task state.
   */
  enum class State {
    Uninitialized, /**< Driver has not been initialized. */
    Idle, /**< Motor is stopped and ready for a command. */
    RotatingForward, /**< A forward cooking rotation is in progress. */
    RotatingBackward, /**< A reverse cooking rotation is in progress. */
    Fault /**< Driver or task fault has stopped operation. */
  };

  /** @brief Construct an R motor task with its embedded driver. */
  TaskRMotor();

  /**
   * @brief Execute one non-blocking update of the R motor state machine.
   * @details
   *   Initializes the driver on first run, services the driver at the configured
   *   task period, and starts the next alternating cook rotation when the
   *   previous one completes.
   */
  void run() override;

  /** @brief Compatibility wrapper that calls run(). */
  void update();

  /**
   * @brief Get the generic task health status.
   * @return Uninitialized before begin succeeds, Fault on failure, otherwise Running.
   */
  Status getStatus() const override;

  /**
   * @brief Get the detailed R motor state.
   * @return Current TaskRMotor::State value.
   */
  State getState() const;

  /**
   * @brief Begin alternating 360 degree cooking rotations.
   * @details
   *   The next run() updates will command a forward rotation followed by a
   *   backward rotation, repeating until stopCookingRotation() or
   *   emergencyStop() is called.
   */
  void startCookingRotation();

  /**
   * @brief Request a normal stop of the cooking rotation.
   * @details
   *   Stops after the current driver command is halted and clears the repeating
   *   rotation request. This is used for normal cook completion or user stop.
   */
  void stopCookingRotation();

  /**
   * @brief Immediately stop the R motor and place the task in Fault.
   */
  void emergencyStop();

  /**
   * @brief Clear an R motor fault for software recovery.
   * @details
   *   This method is intended for recovery after an emergency stop or task
   *   fault. It should not reset the encoder unless that behavior is explicitly
   *   desired elsewhere, because the encoder position is needed to unwind back
   *   to the known initial orientation.
   */
  void resetFault();

  /**
   * @brief Set the duty command used for cooking rotations.
   * @param duty Signed or unsigned duty command. The magnitude is used for motion.
   */
  void setCookingDuty(int16_t duty);

  /**
   * @brief Check whether the R task or driver is actively moving.
   * @return true if a move or continuous rotation is in progress.
   */
  bool isBusy() const;

  /**
   * @brief Check whether the R task or driver has faulted.
   * @return true if the task or low-level driver is faulted.
   */
  bool isFaulted() const;

private:
  /** @brief Minimum time between task state updates in milliseconds. */
  static constexpr uint32_t kUpdatePeriodMs = 10;

  /** @brief Relative rotation size for each cooking oscillation. */
  static constexpr int32_t kCookRotationDegrees = 360;

  /** @brief Default DRV8833 duty command for cooking rotations. */
  static constexpr int16_t kDefaultCookDuty = 700;

  /** @brief Maximum allowed time for one commanded R-axis move. */
  static constexpr uint32_t kMoveTimeoutMs = 8000;

  /** @brief Current state of the R motor task. */
  State state_ = State::Uninitialized;

  /** @brief Low-level R motor and encoder driver. */
  RMotorDriver r_motor_driver_;

  /** @brief Last time the task state machine was serviced. */
  uint32_t last_update_ms_ = 0;

  /** @brief True when repeated cook oscillation has been requested. */
  bool cooking_rotation_requested_ = false;

  /** @brief True when a normal stop request is pending. */
  bool stop_requested_ = false;

  /** @brief Current duty command used for cooking rotations. */
  int16_t cook_duty_ = kDefaultCookDuty;
};

#endif /* TASK_R_MOTOR_H */
