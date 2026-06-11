#ifndef TASK_Z_MOTOR_H
#define TASK_Z_MOTOR_H

// Parent class include
#include "Task.h"

// User created includes
#include "Z_Limit_Switches.h"
#include "z_motor_driver.h"

// Additional includes
#include <cstdint>

// Externs

class TaskZMotor : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    Homing,
    MovingToStartPosition,
    ControllingFlameTemp,
    MovingToRemovalHeight,
    Fault
  };

  TaskZMotor();

  void run() override;
  void update();

  Status getStatus() const override;
  State getState() const;

  void startHoming();
  void startTemperatureControl(int16_t target_flame_temp_fx100);
  void setMeasuredFlameTempFx100(int16_t measured_flame_temp_fx100);
  void moveToRemovalHeight();
  void moveToStartPosition();
  void stopMotion();
  void emergencyStop();
  void resetFault();

  void setPidGains(float kp, float ki, float kd);
  void resetPid();

  bool isBusy() const;
  bool isFaulted() const;
  bool isHomed() const;

  int32_t getPositionSteps() const;
  int32_t getTargetSteps() const;

private:
  /*
   * Z coordinate convention:
   *   Top limit switch position is home and equals 0 steps.
   *   Upward motion is positive.
   *   Downward motion is negative.
   *   The flame is below the mechanism.
   *   Cooking positions are negative step positions.
   */
  static constexpr uint32_t kUpdatePeriodMs = 10;
  static constexpr uint32_t kHomeSpeedStepsPerSecond = 300;
  static constexpr uint32_t kMoveSpeedStepsPerSecond = 500;
  static constexpr uint32_t kPidUpdatePeriodMs = 500;

  static constexpr int32_t kRemovalHeightSteps = -500;
  static constexpr int32_t kStartCookingPositionSteps = -1000;
  static constexpr int32_t kPidOutputLimitSteps = 50;
  static constexpr int32_t kPidDeadbandSteps = 2;

  // Initial PID values are intentionally conservative placeholders.
  // Units are steps per degree F for Kp, steps per degree F second for Ki,
  // and steps per degree F per second for Kd.
  static constexpr float kDefaultKp = 1.0f;
  static constexpr float kDefaultKi = 0.02f;
  static constexpr float kDefaultKd = 0.10f;

  State state_ = State::Uninitialized;

  ZMotorDriver z_motor_driver_;
  ZLimitSwitches z_limit_switches_;

  uint32_t last_update_ms_ = 0;
  uint32_t last_pid_update_ms_ = 0;

  bool homed_ = false;
  bool valid_flame_temp_ = false;

  int16_t target_flame_temp_fx100_ = 35000;
  int16_t measured_flame_temp_fx100_ = 0;

  float kp_ = kDefaultKp;
  float ki_ = kDefaultKi;
  float kd_ = kDefaultKd;
  float integral_error_ = 0.0f;
  float previous_error_f_ = 0.0f;
  bool previous_error_valid_ = false;

  void enterFault(const char* reason);
  void handleDriverLimitState();
  void updatePidControl(uint32_t now_ms);
  int32_t clampPidOutputSteps(float output_steps) const;
};

#endif /* TASK_Z_MOTOR_H */
