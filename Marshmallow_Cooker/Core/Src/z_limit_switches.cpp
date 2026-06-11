/**
 * @file z_limit_switches.cpp
 * @brief Implementation of the Z-axis limit switch GPIO helper.
 */

#include "z_limit_switches.h"

ZLimitSwitches::ZLimitSwitches(GPIO_TypeDef* top_port, uint16_t top_pin, GPIO_TypeDef* bottom_port, uint16_t bottom_pin)
    : top_port_(top_port),
      top_pin_(top_pin),
      bottom_port_(bottom_port),
      bottom_pin_(bottom_pin) {
}

bool ZLimitSwitches::isTopTriggered() const {
  return HAL_GPIO_ReadPin(top_port_, top_pin_) == GPIO_PIN_SET;
}

bool ZLimitSwitches::isBottomTriggered() const {
  return HAL_GPIO_ReadPin(bottom_port_, bottom_pin_) == GPIO_PIN_SET;
}

bool ZLimitSwitches::isAnyTriggered() const {
  return isTopTriggered() || isBottomTriggered();
}
