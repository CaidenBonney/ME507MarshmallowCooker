#ifndef R_MOTOR_DRIVER_H
#define R_MOTOR_DRIVER_H

#include "DRV8833.h"
#include "r_encoder_driver.h"

class RMotorDriver {
public:
  RMotorDriver(DRV8833* driver, REncoder* encoder);

  HAL_StatusTypeDef begin();

  void update();

  static constexpr int16_t kMaxDuty = 1000;
  static constexpr int16_t kMinMovingDuty = 750;
  static constexpr int16_t kFullDuty = 1000;

  void setDuty(int16_t duty); // -1000 to +1000

  void brake();
  void coast();

  int16_t getVelocity() const;
  int32_t getPosition() const;
  int32_t getPositionDegrees() const;

  void resetEncoder();

  void moveDegreesBlocking(int32_t degrees, int16_t duty, uint32_t timeout_ms);

private:
  static constexpr int32_t kCountsPerOutputRev = 3626;

  static constexpr int16_t kMinMovingPower = 350; // 35%
  static constexpr int16_t kMaxPower = 1000; // 100%

  static constexpr int32_t kPositionKp = 12;
  static constexpr int32_t kPositionKpScale = 1000;

  static constexpr uint32_t kControlPeriodMs = 10;

  int32_t degreesToCounts(int32_t degrees) const;
  int32_t countsToDegrees(int32_t counts) const;

  int16_t clampPower(int32_t power) const;

  DRV8833* driver_;
  REncoder* encoder_;
};

#endif /* R_MOTOR_DRIVER_H */