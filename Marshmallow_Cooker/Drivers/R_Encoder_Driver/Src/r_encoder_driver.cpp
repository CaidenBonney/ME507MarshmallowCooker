#include "r_encoder_driver.h"

REncoderDriver::REncoderDriver(TIM_HandleTypeDef* htim)
    : htim_(htim),
      velocity_(0),
      position_(0),
      last_counter_value_(0) {
}

void REncoderDriver::update() {
  const uint32_t counter_value = __HAL_TIM_GET_COUNTER(htim_);

  int32_t delta = static_cast<int32_t>(counter_value) - static_cast<int32_t>(last_counter_value_);

  if (delta > kHalfCounterRange) {
    delta -= kCounterRange;
  } else if (delta < -kHalfCounterRange) {
    delta += kCounterRange;
  }

  last_counter_value_ = counter_value;
  velocity_ = static_cast<int16_t>(-delta);
  position_ -= delta;
}

void REncoderDriver::reset() {
  velocity_ = 0;
  position_ = 0;
  last_counter_value_ = __HAL_TIM_GET_COUNTER(htim_);
}

int16_t REncoderDriver::getVelocity() const {
  return velocity_;
}

int32_t REncoderDriver::getPosition() const {
  return position_;
}
