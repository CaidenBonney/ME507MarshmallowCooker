#ifndef R_MOTOR_DRIVER_H
#define R_MOTOR_DRIVER_H

#include "DRV8833.h"
#include "r_encoder_driver.h"

class RMotorDriver {
public:
  RMotorDriver(DRV8833* driver, REncoder* encoder);

  HAL_StatusTypeDef begin();

  void update();

  void setPower(float power);
  void setDuty(int16_t duty);

  void brake();
  void coast();

  int16_t getVelocity() const;
  int32_t getPosition() const;

  void resetEncoder();

private:
  DRV8833* driver_;
  REncoder* encoder_;
};

#endif /* R_MOTOR_DRIVER_H */