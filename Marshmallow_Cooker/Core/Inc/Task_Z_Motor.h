#ifndef TASK_Z_MOTOR_H
#define TASK_Z_MOTOR_H

/**
 * @file Task_Z_Motor.h
 * @brief Cooperative task for the vertical Z-axis stepper motor.
 * @details
 *   TaskZMotor manages homing, commanded vertical moves, and flame-temperature
 *   PID height control for the marshmallow cooker. The Z coordinate convention
 *   is defined by the top limit switch: home is 0 steps, upward motion is
 *   positive, and downward motion toward the flame is negative.
 */

#include "Task.h"
#include "Z_Limit_Switches.h"
#include "z_motor_driver.h"

#include <cstdint>

/**
 * @class TaskZMotor
 * @brief State-machine wrapper around the Z-axis stepper driver.
 * @details
 *   The task initializes the ZMotorDriver, homes toward the top limit switch,
 *   moves to cook/removal positions, and adjusts height using a conservative PID
 *   loop based on thermocouple flame temperature. TaskZMotor owns cooking-level
 *   behavior while the low-level driver owns individual step pulses and limit
 *   input interpretation.
 */
class TaskZMotor : public Task {
public:
  /**
   * @enum State
   * @brief Detailed Z-axis task state.
   */
  enum class State {
    Uninitialized,        /**< Driver has not been initialized. */
    Idle,                 /**< Axis is stopped and ready for a command. */
    Homing,               /**< Axis is moving upward toward the top limit switch. */
    MovingToStartPosition,/**< Axis is moving from home to the initial cook height. */
    ControllingFlameTemp, /**< PID height control is active. */
    MovingToRemovalHeight,/**< Axis is moving to the removal height. */
    Fault                 /**< A limit, driver, or command fault has stopped the axis. */
  };

  /** @brief Construct the Z motor task and embedded limit/driver objects. */
  TaskZMotor();

  /**
   * @brief Execute one non-blocking update of the Z motor task.
   * @details
   *   Initializes the driver, services stepper motion, handles limit states, and
   *   runs homing, commanded move, or PID-control state logic as appropriate.
   */
  void run() override;

  /** @brief Compatibility wrapper that calls run(). */
  void update();

  /**
   * @brief Get generic task health status.
   * @return Uninitialized before setup, Fault when faulted, otherwise Running.
   */
  Status getStatus() const override;

  /**
   * @brief Get the detailed Z-axis task state.
   * @return Current TaskZMotor::State value.
   */
  State getState() const;

  /**
   * @brief Begin homing upward toward the top limit switch.
   * @details
   *   A successful home sets the Z position to 0 steps and marks the axis as
   *   homed. Cooking and manual moves are rejected until homing completes.
   */
  void startHoming();

  /**
   * @brief Begin the cooking height sequence and PID control.
   * @param target_flame_temp_fx100 Flame temperature target in degrees F x100.
   */
  void startTemperatureControl(int16_t target_flame_temp_fx100);

  /**
   * @brief Provide the latest flame temperature measurement to the PID loop.
   * @param measured_flame_temp_fx100 Thermocouple hot-junction temperature in degrees F x100.
   */
  void setMeasuredFlameTempFx100(int16_t measured_flame_temp_fx100);

  /** @brief Command the axis to the configured marshmallow removal height. */
  void moveToRemovalHeight();

  /** @brief Command the axis to the configured initial cooking height. */
  void moveToStartPosition();

  /** @brief Stop Z motion and return the task to Idle unless already faulted. */
  void stopMotion();

  /** @brief Immediately stop Z motion and place the task in Fault. */
  void emergencyStop();

  /** @brief Clear a Z task fault and require homing before the next cook. */
  void resetFault();

  /**
   * @brief Set PID gains used for flame-temperature height control.
   * @param kp Proportional gain in steps per degree F.
   * @param ki Integral gain in steps per degree F-second.
   * @param kd Derivative gain in steps per degree F per second.
   */
  void setPidGains(float kp, float ki, float kd);

  /** @brief Clear PID integrator, derivative history, and update timestamp. */
  void resetPid();

  /**
   * @brief Check whether the task or driver is currently moving.
   * @return true when homing, moving, PID-controlling, or the driver is busy.
   */
  bool isBusy() const;

  /**
   * @brief Check whether the task or low-level driver is faulted.
   * @return true if either the task state or driver state indicates a fault.
   */
  bool isFaulted() const;

  /**
   * @brief Check whether the Z axis has completed top-limit homing.
   * @return true after a successful home operation.
   */
  bool isHomed() const;

  /**
   * @brief Get the current Z position estimate.
   * @return Current position in microstep units, with home equal to 0.
   */
  int32_t getPositionSteps() const;

  /**
   * @brief Get the current Z target position.
   * @return Target position in microstep units.
   */
  int32_t getTargetSteps() const;

private:
  /** @brief Minimum period for high-level Z task state logic in milliseconds. */
  static constexpr uint32_t kUpdatePeriodMs = 10;

  /** @brief Speed used while homing toward the top limit switch. */
  static constexpr uint32_t kHomeSpeedStepsPerSecond = 300;

  /** @brief Speed used for normal commanded Z moves. */
  static constexpr uint32_t kMoveSpeedStepsPerSecond = 500;

  /** @brief Time between PID height-control updates in milliseconds. */
  static constexpr uint32_t kPidUpdatePeriodMs = 500;

  /** @brief Z position used for safe marshmallow removal. */
  static constexpr int32_t kRemovalHeightSteps = -500;

  /** @brief Initial Z cooking position before PID control begins. */
  static constexpr int32_t kStartCookingPositionSteps = -1000;

  /** @brief Maximum signed PID correction per update in steps. */
  static constexpr int32_t kPidOutputLimitSteps = 50;

  /** @brief PID output magnitude below which no retargeting occurs. */
  static constexpr int32_t kPidDeadbandSteps = 2;

  /** @brief Default proportional gain for conservative bring-up testing. */
  static constexpr float kDefaultKp = 1.0f;

  /** @brief Default integral gain for conservative bring-up testing. */
  static constexpr float kDefaultKi = 0.02f;

  /** @brief Default derivative gain for conservative bring-up testing. */
  static constexpr float kDefaultKd = 0.10f;

  /** @brief Current detailed state of the Z motor task. */
  State state_ = State::Uninitialized;

  /** @brief Low-level stepper motor driver. */
  ZMotorDriver z_motor_driver_;

  /** @brief Limit switch helper object for top and bottom Z switches. */
  ZLimitSwitches z_limit_switches_;

  /** @brief HAL tick timestamp of the most recent state-machine update. */
  uint32_t last_update_ms_ = 0;

  /** @brief HAL tick timestamp of the most recent PID update. */
  uint32_t last_pid_update_ms_ = 0;

  /** @brief True after top-limit homing has completed. */
  bool homed_ = false;

  /** @brief True after a valid thermocouple flame reading has been supplied. */
  bool valid_flame_temp_ = false;

  /** @brief Flame temperature target in degrees F x100. */
  int16_t target_flame_temp_fx100_ = 35000;

  /** @brief Most recent flame temperature measurement in degrees F x100. */
  int16_t measured_flame_temp_fx100_ = 0;

  /** @brief Proportional PID gain. */
  float kp_ = kDefaultKp;

  /** @brief Integral PID gain. */
  float ki_ = kDefaultKi;

  /** @brief Derivative PID gain. */
  float kd_ = kDefaultKd;

  /** @brief Accumulated PID integral error in degree F-seconds. */
  float integral_error_ = 0.0f;

  /** @brief Previous PID error value used to compute derivative error. */
  float previous_error_f_ = 0.0f;

  /** @brief True once previous_error_f_ contains a valid prior error. */
  bool previous_error_valid_ = false;

  /**
   * @brief Enter Z fault state and print a reason.
   * @param reason Null-terminated text describing the fault source.
   */
  void enterFault(const char* reason);

  /** @brief Convert low-level driver limit states into task actions. */
  void handleDriverLimitState();

  /**
   * @brief Run one PID-control update when its update period has elapsed.
   * @param now_ms Current HAL tick time in milliseconds.
   */
  void updatePidControl(uint32_t now_ms);

  /**
   * @brief Clamp a floating-point PID output to the configured step limit.
   * @param output_steps Raw PID output in steps.
   * @return Signed integer step correction limited to kPidOutputLimitSteps.
   */
  int32_t clampPidOutputSteps(float output_steps) const;
};

#endif /* TASK_Z_MOTOR_H */
