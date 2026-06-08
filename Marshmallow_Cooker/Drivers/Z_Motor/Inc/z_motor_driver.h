#ifndef Z_MOTOR_DRIVER_H
#define Z_MOTOR_DRIVER_H

#include "main.h"
#include "stm32f4xx_hal.h"

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

  void homeDown(uint32_t speed_steps_per_second);
  void zeroPosition();

  void setSpeed(uint32_t steps_per_second);
  void setMicrosteps(uint16_t microsteps);
  void setLimitsActiveLow(bool active_low);

  int32_t getPositionSteps() const;
  int32_t getTargetSteps() const;
  State getState() const;

  bool topLimitPressed() const;
  bool bottomLimitPressed() const;
  bool driverFaultActive() const;

private:
  static constexpr uint32_t kMinStepPulseUs = 3;
  static constexpr uint32_t kDefaultSpeedStepsPerSecond = 400;
  static constexpr uint16_t kDefaultMicrosteps = 16;
  static constexpr int32_t kFullStepsPerRev = 200;

  void configureGpioPins();
  void setDirection(Direction direction);
  void stepOnce();
  bool limitBlocksDirection(Direction direction) const;
  uint32_t micros() const;

  int32_t position_steps_;
  int32_t target_steps_;

  uint32_t speed_steps_per_second_;
  uint32_t step_interval_us_;
  uint32_t last_step_time_us_;

  uint16_t microsteps_;
  bool enabled_;
  bool homing_down_;
  bool limits_active_low_;

  Direction current_direction_;
  State state_;
};

#endif