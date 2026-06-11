#ifndef R_MOTOR_DRIVER_H
#define R_MOTOR_DRIVER_H

#include "DRV8833.h"
#include "r_encoder_driver.h"

class RMotorDriver {
public:
  enum class State {
    Disabled,
    Idle,
    MovingToPosition,
    RotatingContinuous,
    Fault
  };

  RMotorDriver();

  HAL_StatusTypeDef begin();

  void update();

  void setDuty(int16_t duty);
  void brake();
  void coast();
  void stop();

  void moveDegrees(int32_t degrees, int16_t duty, uint32_t timeout_ms);
  void moveToDegrees(int32_t target_degrees, int16_t duty, uint32_t timeout_ms);
  void rotateContinuous(int16_t duty);

  int16_t getVelocity() const;
  int32_t getPosition() const;
  int32_t getPositionDegrees() const;

  void resetEncoder();

  bool isBusy() const;
  bool isFaulted() const;
  State getState() const;

  // Keep temporarily for testing, but remove from tasks later.
  void moveDegreesBlocking(int32_t degrees, int16_t duty, uint32_t timeout_ms);

private:
  static constexpr int32_t kCountsPerOutputRev = 3626;

  static constexpr int16_t kMinMovingPower = 350;
  static constexpr int16_t kMaxPower = 1000;

  static constexpr int32_t kPositionToleranceCounts = 8;

  int32_t degreesToCounts(int32_t degrees) const;
  int32_t countsToDegrees(int32_t counts) const;
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