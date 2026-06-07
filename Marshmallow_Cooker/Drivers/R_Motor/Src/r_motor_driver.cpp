#include "r_motor_driver.h"

RMotorDriver::RMotorDriver(DRV8833* driver,
                           REncoder* encoder)
    : driver_(driver),
      encoder_(encoder) {
}

HAL_StatusTypeDef RMotorDriver::begin() {
  encoder_->reset();
  return driver_->begin();
}

void RMotorDriver::update() {
  encoder_->update();
}

void RMotorDriver::setPower(float power) {
  driver_->setPower(power);
}

void RMotorDriver::setDuty(int16_t duty) {
  driver_->setDuty(duty);
}

void RMotorDriver::brake() {
  driver_->brake();
}

void RMotorDriver::coast() {
  driver_->coast();
}

int16_t RMotorDriver::getVelocity() const {
  return encoder_->getVelocity();
}

int32_t RMotorDriver::getPosition() const {
  return encoder_->getPosition();
}

void RMotorDriver::resetEncoder() {
  encoder_->reset();
}