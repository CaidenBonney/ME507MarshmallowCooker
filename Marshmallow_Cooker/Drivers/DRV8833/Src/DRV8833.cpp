/**
 * @file DRV8833.cpp
 * @brief Implementation of the DRV8833 brushed DC motor driver wrapper.
 */

#include "DRV8833.h"

/**
 * @brief Construct a DRV8833 PWM driver.
 * @param htim Timer handle that owns the two PWM output channels.
 * @param in1_channel Timer channel connected to DRV8833 IN1.
 * @param in2_channel Timer channel connected to DRV8833 IN2.
 */
DRV8833::DRV8833(TIM_HandleTypeDef* htim, uint32_t in1_channel, uint32_t in2_channel)
    : htim_(htim),
      in1_channel_(in1_channel),
      in2_channel_(in2_channel) {
}

/**
 * @brief Start PWM outputs and place the bridge in coast mode.
 * @return HAL_OK if both PWM channels start successfully, otherwise HAL_ERROR.
 */
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

/**
 * @brief Command signed motor duty.
 * @param duty Signed duty command from -1000 to +1000.
 */
void DRV8833::setDuty(int16_t duty) {
  if (duty > kMaxDuty) {
    duty = kMaxDuty;
  } else if (duty < -kMaxDuty) {
    duty = -kMaxDuty;
  }

  int16_t abs_duty = duty >= 0 ? duty : -duty;

  uint16_t pwm = static_cast<uint16_t>((static_cast<int32_t>(getTimerPeriod()) * abs_duty) / kMaxDuty);

  if (duty > 0) {
    write(pwm, 0); // Forward: AIN1 = PWM, AIN2 = 0
  } else if (duty < 0) {
    write(0, pwm); // Reverse: AIN1 = 0, AIN2 = PWM
  } else {
    coast(); // AIN1 = 0, AIN2 = 0
  }
}

/**
 * @brief Actively brake the motor by driving both bridge inputs high.
 */
void DRV8833::brake() {
  write(getTimerPeriod(), getTimerPeriod()); // AIN1 = 1, AIN2 = 1
}

/**
 * @brief Coast the motor by driving both bridge inputs low.
 */
void DRV8833::coast() {
  write(0, 0); // AIN1 = 0, AIN2 = 0
}

/**
 * @brief Get the PWM timer auto-reload value.
 * @return Timer period used to scale duty commands into compare values.
 */
uint16_t DRV8833::getTimerPeriod() const {
  return static_cast<uint16_t>(__HAL_TIM_GET_AUTORELOAD(htim_));
}

/**
 * @brief Write raw compare values to the two PWM channels.
 * @param in1 Compare value for IN1.
 * @param in2 Compare value for IN2.
 */
void DRV8833::write(uint16_t in1, uint16_t in2) {
  __HAL_TIM_SET_COMPARE(htim_, in1_channel_, in1);
  __HAL_TIM_SET_COMPARE(htim_, in2_channel_, in2);
}
