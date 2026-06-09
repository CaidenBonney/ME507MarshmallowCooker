#include "TMC2209.h"

TMC2209::TMC2209(TIM_HandleTypeDef* htim,
                 uint32_t in1_channel,
                 uint32_t in2_channel)
    : htim_(htim),
      in1_channel_(in1_channel),
      in2_channel_(in2_channel) {
}

HAL_StatusTypeDef TMC2209::begin() {
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

void TMC2209::setDuty(int16_t duty) {
  if (duty > kMaxDuty) {
    duty = kMaxDuty;
  } else if (duty < -kMaxDuty) {
    duty = -kMaxDuty;
  }

  int16_t abs_duty = duty >= 0 ? duty : -duty;

  uint16_t pwm = static_cast<uint16_t>(
      (static_cast<int32_t>(getTimerPeriod()) * abs_duty) / kMaxDuty);

  if (duty > 0) {
    write(pwm, 0);      // Forward: AIN1 = PWM, AIN2 = 0
  } else if (duty < 0) {
    write(0, pwm);      // Reverse: AIN1 = 0, AIN2 = PWM
  } else {
    coast();            // AIN1 = 0, AIN2 = 0
  }
}

void TMC2209::brake() {
  write(getTimerPeriod(), getTimerPeriod());  // AIN1 = 1, AIN2 = 1
}

void TMC2209::coast() {
  write(0, 0);                  // AIN1 = 0, AIN2 = 0
}

uint16_t TMC2209::getTimerPeriod() const {
  return static_cast<uint16_t>(__HAL_TIM_GET_AUTORELOAD(htim_));
}

void TMC2209::write(uint16_t in1, uint16_t in2) {
  __HAL_TIM_SET_COMPARE(htim_, in1_channel_, in1);
  __HAL_TIM_SET_COMPARE(htim_, in2_channel_, in2);
}