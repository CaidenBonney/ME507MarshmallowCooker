#ifndef TASK_R_MOTOR_H
#define TASK_R_MOTOR_H

/**
 * @file Task_R_Motor.h
 * @brief Rotisserie motor task for oscillating and re-zeroing the marshmallow.
 */

#include "Task.h"
#include "r_motor_driver.h"
#include <cstdint>

/** @brief UART print helper provided by main.cpp. */
extern void print_str(const char* str);

/**
 * @brief Cooperative task that controls the R-axis rotisserie motor.
 *
 * The task commands alternating +360/-360 degree moves during cooking and can
 * return the motor to encoder zero to prevent thermocouple wire wind-up.
 */
class TaskRMotor : public Task {
public:
  /** @brief R motor task state. */
  enum class State {
    Uninitialized,              ///< Driver has not been initialized.
    Idle,                       ///< Motor is stopped and ready.
    RotatingForward,            ///< Current cooking move is in the positive direction.
    RotatingBackward,           ///< Current cooking move is in the negative direction.
    ReturningToInitialRotation, ///< Motor is returning to encoder zero.
    Fault                       ///< Task has faulted and requires reset.
  };

  /** @brief Construct the R motor task. */
  TaskRMotor();

  /** @copydoc Task::run */
  void run() override;

  /** @brief Compatibility wrapper for run(). */
  void update();

  /** @copydoc Task::getStatus */
  Status getStatus() const override;

  /** @brief Get the current R motor task state. */
  State getState() const;

  /** @brief Start alternating forward/backward cooking rotation. */
  void startCookingRotation();

  /** @brief Stop cooking rotation and leave the task idle. */
  void stopCookingRotation();

  /** @brief Command the R motor back to encoder position zero. */
  void returnToInitialRotation();

  /** @brief Immediately stop the R motor and place the task in Fault. */
  void emergencyStop();

  /** @brief Clear a recoverable R motor task fault without reinitializing the driver. */
  void resetFault();

  /** @brief Set the duty command used for cooking rotation moves. */
  void setCookingDuty(int16_t duty);

  /** @brief Return true while the task or driver is actively moving. */
  bool isBusy() const;

  /** @brief Return true if the task or driver is faulted. */
  bool isFaulted() const;

private:
  static constexpr uint32_t kUpdatePeriodMs = 10;          ///< R task update period.
  static constexpr int32_t kCookRotationDegrees = 360;     ///< Oscillation size for each cooking move.
  static constexpr int16_t kDefaultCookDuty = 700;         ///< Default DRV8833 duty command.
  static constexpr uint32_t kMoveTimeoutMs = 8000;         ///< Timeout for cooking rotation moves.
  static constexpr uint32_t kReturnToInitialTimeoutMs = 8000; ///< Timeout for return-to-zero moves.

  State state_ = State::Uninitialized;

  RMotorDriver r_motor_driver_;

  uint32_t last_update_ms_ = 0;

  bool cooking_rotation_requested_ = false;
  bool stop_requested_ = false;

  int16_t cook_duty_ = kDefaultCookDuty;
};

#endif /* TASK_R_MOTOR_H */
