#ifndef DRV8833_H
#define DRV8833_H

/**
 * @file DRV8833.h
 * @brief PWM driver interface for the DRV8833 brushed DC motor driver.
 */

#include "stm32f4xx_hal.h"

/**
 * @class DRV8833
 * @brief Controls a DRV8833 H-bridge using two timer PWM channels.
 *
 * Duty commands use a signed fixed range where +1000 is full forward, -1000 is
 * full reverse, and 0 coasts the motor. The driver also supports active braking.
 */
class DRV8833 {
public:
  /**
   * @brief Construct a DRV8833 PWM driver.
   * @param htim Timer handle that owns the two PWM output channels.
   * @param in1_channel Timer channel connected to DRV8833 IN1.
   * @param in2_channel Timer channel connected to DRV8833 IN2.
   */
  DRV8833(TIM_HandleTypeDef* htim, uint32_t in1_channel, uint32_t in2_channel);

  /**
   * @brief Start PWM outputs and place the bridge in coast mode.
   * @return HAL_OK if both PWM channels start successfully, otherwise HAL_ERROR.
   */
  HAL_StatusTypeDef begin();

  /** @brief Maximum signed duty command. A value of 1000 represents 100 percent duty. */
  static constexpr int16_t kMaxDuty = 1000;

  /**
   * @brief Command signed motor duty.
   * @param duty Signed duty command from -1000 to +1000.
   */
  void setDuty(int16_t duty);

  /** @brief Actively brake the motor by driving both bridge inputs high. */
  void brake();

  /** @brief Coast the motor by driving both bridge inputs low. */
  void coast();

private:
  /**
   * @brief Write raw compare values to the two PWM channels.
   * @param in1 Compare value for IN1.
   * @param in2 Compare value for IN2.
   */
  void write(uint16_t in1, uint16_t in2);

  /**
   * @brief Get the PWM timer auto-reload value.
   * @return Timer period used to scale duty commands into compare values.
   */
  uint16_t getTimerPeriod() const;

  /** @brief Timer handle that owns both PWM channels. */
  TIM_HandleTypeDef* htim_;

  /** @brief Timer channel connected to DRV8833 IN1. */
  uint32_t in1_channel_;

  /** @brief Timer channel connected to DRV8833 IN2. */
  uint32_t in2_channel_;
};

#endif /* DRV8833_H */
