#ifndef Z_LIMIT_SWITCHES_H
#define Z_LIMIT_SWITCHES_H

/**
 * @file z_limit_switches.h
 * @brief Simple GPIO wrapper for Z-axis top and bottom limit switches.
 */

#include "stm32f4xx_hal.h"

/** @brief Reads the Z top and bottom limit switch GPIO pins. */
class ZLimitSwitches {
public:
  /**
   * @brief Construct a Z limit switch reader.
   * @param top_port GPIO port for the top switch.
   * @param top_pin GPIO pin for the top switch.
   * @param bottom_port GPIO port for the bottom switch.
   * @param bottom_pin GPIO pin for the bottom switch.
   */
  ZLimitSwitches(GPIO_TypeDef* top_port, uint16_t top_pin, GPIO_TypeDef* bottom_port, uint16_t bottom_pin);

  /** @brief Return true when the top switch input is triggered. */
  bool isTopTriggered() const;

  /** @brief Return true when the bottom switch input is triggered. */
  bool isBottomTriggered() const;

  /** @brief Return true when either switch input is triggered. */
  bool isAnyTriggered() const;

private:
  GPIO_TypeDef* top_port_;
  uint16_t top_pin_;

  GPIO_TypeDef* bottom_port_;
  uint16_t bottom_pin_;
};

#endif /* Z_LIMIT_SWITCHES_H */
