#include "DRV8833.h"

DRV8833::DRV8833(TIM_HandleTypeDef* htim,
                 uint32_t in1_channel,
                 uint32_t in2_channel)
    : htim_(htim),
      in1_channel_(in1_channel),
      in2_channel_(in2_channel) {
}

HAL_StatusTypeDef DRV8833::begin() {
  write(0, 0);

  if (HAL_TIM_PWM_Start(htim_, in1_channel_) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(htim_, in2_channel_) != HAL_OK) {
    return HAL_ERROR;
  }

  coast();
  return HAL_OK;
}

void DRV8833::setPower(float power) {
  if (power > 1.0f) {
    power = 1.0f;
  } else if (power < -1.0f) {
    power = -1.0f;
  }

  setDuty(static_cast<int16_t>(power * 1000.0f));
}

void DRV8833::setDuty(int16_t duty) {
  if (duty > 1000) {
    duty = 1000;
  } else if (duty < -1000) {
    duty = -1000;
  }

  uint16_t pwm = static_cast<uint16_t>(
      (static_cast<int32_t>(maxDuty()) * std::abs(duty)) / 1000);

  if (duty > 0) {
    write(pwm, 0);      // Forward: AIN1 = PWM, AIN2 = 0
  } else if (duty < 0) {
    write(0, pwm);      // Reverse: AIN1 = 0, AIN2 = PWM
  } else {
    coast();            // AIN1 = 0, AIN2 = 0
  }
}

void DRV8833::brake() {
  write(maxDuty(), maxDuty());  // AIN1 = 1, AIN2 = 1
}

void DRV8833::coast() {
  write(0, 0);                  // AIN1 = 0, AIN2 = 0
}

uint16_t DRV8833::maxDuty() const {
  return static_cast<uint16_t>(__HAL_TIM_GET_AUTORELOAD(htim_));
}

void DRV8833::write(uint16_t in1, uint16_t in2) {
  __HAL_TIM_SET_COMPARE(htim_, in1_channel_, in1);
  __HAL_TIM_SET_COMPARE(htim_, in2_channel_, in2);
}