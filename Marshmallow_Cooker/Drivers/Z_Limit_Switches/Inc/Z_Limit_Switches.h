#ifndef Z_LIMIT_SWITCHES_H
#define Z_LIMIT_SWITCHES_H

/**
 * @file z_limit_switches.h
 * @brief Helper class for reading Z-axis top and bottom limit switches.
 * @details
 *   The limit switch helper stores the GPIO ports and pins for the two vertical
 *   travel limit switches. It provides compact query methods used by higher
 *   level Z-axis control code and keeps GPIO access in one small class.
 */

#include "stm32f4xx_hal.h"

/**
 * @class ZLimitSwitches
 * @brief Reads the top and bottom Z travel limit switches.
 */
class ZLimitSwitches {
public:
  /**
   * @brief Construct a Z limit switch reader.
   * @param top_port GPIO port for the top limit switch.
   * @param top_pin GPIO pin mask for the top limit switch.
   * @param bottom_port GPIO port for the bottom limit switch.
   * @param bottom_pin GPIO pin mask for the bottom limit switch.
   */
  ZLimitSwitches(GPIO_TypeDef* top_port, uint16_t top_pin, GPIO_TypeDef* bottom_port, uint16_t bottom_pin);

  /**
   * @brief Check whether the top limit switch is triggered.
   * @return true when the top limit input is active.
   */
  bool isTopTriggered() const;

  /**
   * @brief Check whether the bottom limit switch is triggered.
   * @return true when the bottom limit input is active.
   */
  bool isBottomTriggered() const;

  /**
   * @brief Check whether either Z limit switch is triggered.
   * @return true when the top or bottom limit input is active.
   */
  bool isAnyTriggered() const;

private:
  /** @brief GPIO port for the top limit switch. */
  GPIO_TypeDef* top_port_;

  /** @brief GPIO pin mask for the top limit switch. */
  uint16_t top_pin_;

  /** @brief GPIO port for the bottom limit switch. */
  GPIO_TypeDef* bottom_port_;

  /** @brief GPIO pin mask for the bottom limit switch. */
  uint16_t bottom_pin_;
};

#endif /* Z_LIMIT_SWITCHES_H */
