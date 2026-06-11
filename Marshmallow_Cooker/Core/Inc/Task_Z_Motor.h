#ifndef TASK_Z_MOTOR_H
#define TASK_Z_MOTOR_H

// Parent class include
#include "Task.h"

// User created includes
#include "Z_Limit_Switches.h"
#include "z_motor_driver.h"

// Additional includes
#include "stm32f4xx_hal.h"

// Externs
extern void Error_Handler();
extern void print_str(const char* str);
extern char print_buf[100];

class TaskZMotor : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    FindingTopLimit,
    MovingToRemovalHeight,
    MovingToTarget,
    ControllingFlameTemp,
    Fault
  };

  TaskZMotor();

  void run() override;
  void update();

  Status getStatus() const override;
  State getState() const;

  void startHoming();
  void moveToRemovalHeight();
  void moveToTarget(int32_t target_position_steps);
  void startTemperatureControl(int16_t target_flame_temp_fx100);
  void stopMotion();
  void emergencyStop();
  void resetFault();

  void setMeasuredFlameTempFx100(int16_t flame_temp_fx100);

  bool isHomed() const;
  bool isBusy() const;
  bool isFaulted() const;

  int32_t getPositionSteps() const;

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
  static constexpr uint32_t kHomeSpeedStepsPerSecond = 250;
  static constexpr uint32_t kMoveSpeedStepsPerSecond = 500;

  // TODO: tune these after testing the actual mechanism.
  static constexpr int32_t kRemovalHeightSteps = -500;
  static constexpr int32_t kPidCorrectionStepLimit = 20;
  static constexpr int16_t kPidDeadbandFx100 = 200; // 2.00 F

  State state_ = State::Uninitialized;

  ZMotorDriver z_motor_driver_;
  ZLimitSwitches z_limit_switches_;

  uint32_t last_update_ms_ = 0;

  bool homed_ = false;
  bool home_requested_ = false;
  bool removal_height_requested_ = false;
  bool target_move_requested_ = false;
  bool temp_control_requested_ = false;
  bool stop_requested_ = false;

  int32_t requested_target_steps_ = 0;

  int16_t target_flame_temp_fx100_ = 0;
  int16_t measured_flame_temp_fx100_ = 0;
  bool has_flame_temp_ = false;

  void handleLimitSwitches();
  void updateTemperatureControl();
};

#endif /* TASK_Z_MOTOR_H */
