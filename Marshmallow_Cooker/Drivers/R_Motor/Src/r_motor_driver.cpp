#include "r_motor_driver.h"

RMotorDriver::RMotorDriver(DRV8833* driver, REncoderDriver* encoder)
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

void RMotorDriver::setDuty(int16_t duty) {
  driver_->setDuty(clampPower(duty));
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

int32_t RMotorDriver::getPositionDegrees() const {
  return countsToDegrees(encoder_->getPosition());
}

void RMotorDriver::resetEncoder() {
  encoder_->reset();
}

int32_t RMotorDriver::degreesToCounts(int32_t degrees) const {
  return (degrees * kCountsPerOutputRev) / 360;
}

int32_t RMotorDriver::countsToDegrees(int32_t counts) const {
  return (counts * 360) / kCountsPerOutputRev;
}

int16_t RMotorDriver::clampPower(int32_t power) const {
  if (power > kMaxPower) {
    return kMaxPower;
  }

  if (power < -kMaxPower) {
    return -kMaxPower;
  }

  return static_cast<int16_t>(power);
}

void RMotorDriver::moveDegreesBlocking(int32_t degrees, int16_t duty, uint32_t timeout_ms) {

  if (degrees == 0 || duty == 0) {
    brake();
    return;
  }

  resetEncoder();

  const int32_t target_counts = degreesToCounts(degrees);

  const int32_t direction = target_counts >= 0 ? 1 : -1;

  const int32_t abs_target_counts = target_counts >= 0 ? target_counts : -target_counts;

  duty = duty >= 0 ? duty : -duty;

  setDuty(duty * direction);

  const uint32_t start_ms = HAL_GetTick();

  while ((HAL_GetTick() - start_ms) < timeout_ms) {

    update();

    int32_t position = getPosition() * direction;

    if (position >= abs_target_counts) {
      break;
    }

    HAL_Delay(1);
  }

  brake();
}