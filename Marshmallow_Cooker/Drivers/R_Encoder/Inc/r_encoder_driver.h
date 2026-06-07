#ifndef R_ENCODER_DRIVER_H
#define R_ENCODER_DRIVER_H

#include "stm32f4xx_hal.h"

class REncoder {
public:
  REncoder(TIM_HandleTypeDef* htim);

  void update();
  void reset();

  int16_t getVelocity() const;
  int32_t getPosition() const;

private:
  static constexpr int32_t kCounterRange = 65536;
  static constexpr int32_t kHalfCounterRange = kCounterRange / 2;

  TIM_HandleTypeDef* htim_;
  int16_t velocity_;
  int32_t position_;
  uint32_t last_counter_value_;
};

#endif /* R_ENCODER_DRIVER_H */