#ifndef Z_MOTOR_DRIVER_H
#define Z_MOTOR_DRIVER_H

#include "TMC2209.h" // PWM driver for Z motor
#include "stm32f4xx_hal.h"
#include <cstdint>

class ZMotorDriver {
public:
  enum class Direction {
    Up,
    Down
  };

  enum class State {
    Disabled,
    Idle,
    Moving,
    Fault,
    HitTopLimit,
    HitBottomLimit
  };

  ZMotorDriver();

  void begin();
  void enable();
  void disable();
  void update();

  void moveSteps(int32_t steps);
  void moveTo(int32_t target_position_steps);
  void jog(Direction direction, uint32_t speed_steps_per_second);
  void stop();

  // Home toward the selected limit switch. For this project, TaskZMotor homes Up.
  void home(Direction direction, uint32_t speed_steps_per_second);
  void homeUp(uint32_t speed_steps_per_second);
  void homeDown(uint32_t speed_steps_per_second);

  void zeroPosition();

  void setSpeed(uint32_t steps_per_second);
  void setSpeedStepsPerSecond(uint32_t steps_per_second);
  void setMicrosteps(uint16_t microsteps);
  void setLimitsActiveLow(bool active_low);
  void setDirectionInverted(bool inverted);

  int32_t getPositionSteps() const;
  int32_t getTargetSteps() const;
  State getState() const;

  bool isBusy() const;
  bool isFaulted() const;

  bool topLimitPressed() const;
  bool bottomLimitPressed() const;
  bool driverFaultActive() const;

private:
  static constexpr uint32_t kDefaultSpeedStepsPerSecond = 250;
  static constexpr uint16_t kDefaultMicrosteps = 8;
  static constexpr int32_t kFullStepsPerRev = 200;

  void setDirection(Direction direction);
  bool tryStep(Direction direction);
  bool limitBlocksDirection(Direction direction) const;

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
