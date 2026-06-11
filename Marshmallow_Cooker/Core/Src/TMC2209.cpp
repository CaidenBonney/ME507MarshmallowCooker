/**
 * @file TMC2209.cpp
 * @brief Implementation of the TMC2209 step/direction GPIO helper.
 */

#include "TMC2209.h"

#include <cstdio>

#ifndef TMC2209_ENABLE_PRINTF_DIAG
#define TMC2209_ENABLE_PRINTF_DIAG 1
#endif

TMC2209::TMC2209(GPIO_TypeDef* step_port,
                 uint16_t step_pin,
                 GPIO_TypeDef* dir_port,
                 uint16_t dir_pin,
                 GPIO_TypeDef* enn_port,
                 uint16_t enn_pin,
                 GPIO_TypeDef* diag_port,
                 uint16_t diag_pin)
    : step_port_(step_port),
      step_pin_(step_pin),
      dir_port_(dir_port),
      dir_pin_(dir_pin),
      enn_port_(enn_port),
      enn_pin_(enn_pin),
      diag_port_(diag_port),
      diag_pin_(diag_pin),
      enabled_(false),
      direction_inverted_(false),
      last_diag_state_(false),
      direction_(Direction::Forward),
      step_rate_steps_per_second_(kDefaultStepRateStepsPerSecond),
      step_interval_us_(1000000UL / kDefaultStepRateStepsPerSecond),
      last_step_time_us_(0) {
}

void TMC2209::begin() {
  configureGpioPins();
  enableCycleCounter();

  HAL_GPIO_WritePin(step_port_, step_pin_, GPIO_PIN_RESET);
  writeDirectionPin(direction_);

  disable();
  last_diag_state_ = diagActive();
  last_step_time_us_ = micros();
}

void TMC2209::configureGpioPins() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = step_pin_;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(step_port_, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = dir_pin_;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(dir_port_, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = enn_pin_;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(enn_port_, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = diag_pin_;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(diag_port_, &GPIO_InitStruct);
}

void TMC2209::enableCycleCounter() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void TMC2209::enable() {
  // TMC2209 ENN is active-low: LOW enables the driver power stage.
  HAL_GPIO_WritePin(enn_port_, enn_pin_, GPIO_PIN_RESET);
  enabled_ = true;
  last_step_time_us_ = micros();
}

void TMC2209::disable() {
  // TMC2209 ENN is active-low: HIGH disables the driver power stage.
  HAL_GPIO_WritePin(enn_port_, enn_pin_, GPIO_PIN_SET);
  enabled_ = false;
}

bool TMC2209::isEnabled() const {
  return enabled_;
}

void TMC2209::setDirection(Direction direction) {
  direction_ = direction;
  writeDirectionPin(direction_);
}

TMC2209::Direction TMC2209::getDirection() const {
  return direction_;
}

void TMC2209::setDirectionInverted(bool inverted) {
  direction_inverted_ = inverted;
  writeDirectionPin(direction_);
}

bool TMC2209::directionInverted() const {
  return direction_inverted_;
}

void TMC2209::setStepRate(uint32_t steps_per_second) {
  if (steps_per_second == 0) {
    steps_per_second = 1;
  }

  step_rate_steps_per_second_ = steps_per_second;
  step_interval_us_ = 1000000UL / step_rate_steps_per_second_;

  const uint32_t minimum_interval_us = 2UL * kMinStepPulseUs;
  if (step_interval_us_ < minimum_interval_us) {
    step_interval_us_ = minimum_interval_us;
  }
}

uint32_t TMC2209::getStepRate() const {
  return step_rate_steps_per_second_;
}

uint32_t TMC2209::getStepIntervalUs() const {
  return step_interval_us_;
}

void TMC2209::stepNow() {
  if (!enabled_) {
    return;
  }

  HAL_GPIO_WritePin(step_port_, step_pin_, GPIO_PIN_SET);
  delayUs(kMinStepPulseUs);
  HAL_GPIO_WritePin(step_port_, step_pin_, GPIO_PIN_RESET);

  last_step_time_us_ = micros();
}

bool TMC2209::stepIfDue() {
  if (!enabled_) {
    return false;
  }

  updateDiagLog();

  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_step_time_us_) < step_interval_us_) {
    return false;
  }

  stepNow();
  return true;
}

bool TMC2209::diagActive() const {
  return HAL_GPIO_ReadPin(diag_port_, diag_pin_) == GPIO_PIN_SET;
}

bool TMC2209::updateDiagLog() {
  const bool active = diagActive();

#if TMC2209_ENABLE_PRINTF_DIAG
  if (active && !last_diag_state_) {
    std::printf("TMC2209 Z DIAG asserted\r\n");
  }
#endif

  last_diag_state_ = active;
  return active;
}

void TMC2209::resetDiagLatch() {
  // Many latched driver errors are cleared by disabling and re-enabling ENN.
  const bool was_enabled = enabled_;
  disable();
  HAL_Delay(1);
  last_diag_state_ = diagActive();

  if (was_enabled) {
    enable();
  }
}

uint32_t TMC2209::micros() const {
  return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000UL);
}

void TMC2209::delayUs(uint32_t delay_us) const {
  const uint32_t start_us = micros();
  while ((uint32_t)(micros() - start_us) < delay_us) {
  }
}

void TMC2209::writeDirectionPin(Direction direction) {
  bool forward_high = (direction == Direction::Forward);

  if (direction_inverted_) {
    forward_high = !forward_high;
  }

  HAL_GPIO_WritePin(dir_port_, dir_pin_, forward_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
