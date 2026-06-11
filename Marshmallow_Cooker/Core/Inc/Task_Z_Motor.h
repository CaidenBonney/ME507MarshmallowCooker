#ifndef TASK_Z_MOTOR_H
#define TASK_Z_MOTOR_H

/**
 * @file Task_Z_Motor.h
 * @brief Z-axis stepper task for homing, jogging, and flame-height PID control.
 */

// Parent class include
#include "Task.h"

// User created includes
#include "Z_Limit_Switches.h"
#include "z_motor_driver.h"

// Additional includes
#include <cstdint>

// Externs
/** @brief UART print helper provided by main.cpp. */
extern void print_str(const char* str);

/** @brief Shared formatted-print buffer provided by main.cpp. */
extern char print_buf[100];

/**
 * @brief Cooperative task that controls the Z lift mechanism.
 *
 * The Z coordinate system defines the top limit switch as zero, upward as
 * positive, and downward toward the flame as negative. The task homes Z, moves
 * to starting/removal heights, allows manual jogging, and performs PID flame
 * temperature control during cooking.
 */
class TaskZMotor : public Task {
public:
  /** @brief Z task state. */
  enum class State {
    Uninitialized, ///< Driver has not been initialized.
    Idle, ///< Z is stopped and ready.
    Homing, ///< Moving upward until the top limit is reached.
    MovingToStartPosition, ///< Moving from home to the initial cook height.
    ControllingFlameTemp, ///< PID height control is active.
    MovingToRemovalHeight, ///< Moving to the removal height after cooking.
    Fault ///< Task fault requiring reset.
  };

  /** @brief Construct the Z motor task. */
  TaskZMotor();

  /** @copydoc Task::run */
  void run() override;

  /** @brief Compatibility wrapper for run(). */
  void update();

  /** @copydoc Task::getStatus */
  Status getStatus() const override;

  /** @brief Get the current Z task state. */
  State getState() const;

  /** @brief Start homing upward toward the top limit switch. */
  void startHoming();

  /** @brief Begin the start-position move and then PID flame-height control. */
  void startTemperatureControl(int32_t target_flame_temp_fx100);

  /** @brief Update the measured flame temperature used by the PID controller. */
  void setMeasuredFlameTempFx100(int32_t measured_flame_temp_fx100);

  /** @brief Enable or disable PID debug printing. */
  void setPidDebugEnabled(bool enabled);

  /** @brief Move Z to the configured removal height. */
  void moveToRemovalHeight();

  /** @brief Move Z to the configured initial cooking position. */
  void moveToStartPosition();

  /** @brief Jog Z by a signed relative step amount. */
  void jogRelativeSteps(int32_t relative_steps);

  /** @brief Stop Z motion and leave the task idle unless faulted. */
  void stopMotion();

  /** @brief Immediately stop Z and enter Fault. */
  void emergencyStop();

  /** @brief Clear a recoverable Z fault and require homing before cooking. */
  void resetFault();

  /** @brief Set PID gains used for flame-height control. */
  void setPidGains(float kp, float ki, float kd);

  /** @brief Clear PID integrator and derivative state. */
  void resetPid();

  /** @brief Return true while Z is moving or controlling flame height. */
  bool isBusy() const;

  /** @brief Return true if the Z task or driver is faulted. */
  bool isFaulted() const;

  /** @brief Return true once the top-limit home reference has been established. */
  bool isHomed() const;

  /** @brief Return true when the top limit switch is logically pressed. */
  bool topLimitPressed() const;

  /** @brief Return true when the bottom limit switch is logically pressed. */
  bool bottomLimitPressed() const;

  /** @brief Get the current software Z position in steps. */
  int32_t getPositionSteps() const;

  /** @brief Get the current software Z target in steps. */
  int32_t getTargetSteps() const;

private:
  /**
   * @note Z coordinate convention:
   *       top limit/home is 0 steps, upward is positive, and downward toward
   *       the flame is negative.
   */
  static constexpr uint32_t kUpdatePeriodMs = 500; ///< Task state-machine update period.
  static constexpr uint32_t kHomeSpeedStepsPerSecond = 1600; ///< Homing speed in steps/s.
  static constexpr uint32_t kMoveSpeedStepsPerSecond = 2000; ///< Normal move speed in steps/s.
  static constexpr uint32_t kPidUpdatePeriodMs = 500; ///< PID update period.
  static constexpr uint32_t kMinimumAntiBindSpeedStepsPerSecond = 1600; ///< Minimum speed for anti-binding.

  static constexpr int32_t kRemovalHeightSteps = -500; ///< Z removal height.
  static constexpr int32_t kStartCookingPositionSteps = -11000; ///< Initial cooking height.
  static constexpr int32_t kPidOutputLimitSteps = 750; ///< Maximum PID step correction per update.
  static constexpr int32_t kPidDeadbandSteps = 2; ///< PID command deadband in steps.
  static constexpr int32_t kMinCookPositionSteps = -24400; ///< Software lower travel limit.
  static constexpr float kIntegralErrorLimit = 500.0f; ///< Integral wind-up limit.

  static constexpr float kDefaultKp = 1.0f; ///< Default proportional gain, steps/F.
  static constexpr float kDefaultKi = 0.02f; ///< Default integral gain, steps/(F*s).
  static constexpr float kDefaultKd = 0.10f; ///< Default derivative gain, steps/(F/s).

  State state_ = State::Uninitialized;

  ZMotorDriver z_motor_driver_;
  ZLimitSwitches z_limit_switches_;

  uint32_t last_update_ms_ = 0;
  uint32_t last_pid_update_ms_ = 0;

  bool homed_ = false;
  bool valid_flame_temp_ = false;
  bool pid_debug_enabled_ = false;

  int32_t target_flame_temp_fx100_ = 0;
  int32_t measured_flame_temp_fx100_ = 0;

  float kp_ = kDefaultKp;
  float ki_ = kDefaultKi;
  float kd_ = kDefaultKd;
  float integral_error_ = 0.0f;
  float previous_error_f_ = 0.0f;
  bool previous_error_valid_ = false;

  /** @brief Stop Z, print a fault reason, and enter Fault. */
  void enterFault(const char* reason);

  /** @brief Translate driver limit states into task-level behavior. */
  void handleDriverLimitState();

  /** @brief Run one scheduled PID update if enough time has elapsed. */
  void updatePidControl(uint32_t now_ms);

  /** @brief Clamp a floating-point PID output to the configured step limit. */
  int32_t clampPidOutputSteps(float output_steps) const;

  /**
   * @brief Clamp a requested Z-axis speed to the minimum anti-bind speed.
   * @param requested_speed_steps_per_second Requested motor speed in steps per second.
   * @return Requested speed when it is above the minimum, otherwise the anti-bind minimum speed.
   */
  uint32_t clampZMoveSpeed(uint32_t requested_speed_steps_per_second) const;
};

#endif /* TASK_Z_MOTOR_H */
