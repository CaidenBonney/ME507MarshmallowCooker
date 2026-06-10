#ifndef Z_LIMIT_SWITCHES_H
#define Z_LIMIT_SWITCHES_H

#include "stm32f4xx_hal.h"

class ZLimitSwitches {
public:
  ZLimitSwitches(GPIO_TypeDef* top_port,
                 uint16_t top_pin,
                 GPIO_TypeDef* bottom_port,
                 uint16_t bottom_pin);

  bool isTopTriggered() const;
  bool isBottomTriggered() const;
  bool isAnyTriggered() const;

private:
  GPIO_TypeDef* top_port_;
  uint16_t top_pin_;

  GPIO_TypeDef* bottom_port_;
  uint16_t bottom_pin_;
};

#endif /* Z_LIMIT_SWITCHES_H */