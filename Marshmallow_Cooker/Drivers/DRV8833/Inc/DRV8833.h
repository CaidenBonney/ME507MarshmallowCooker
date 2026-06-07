#ifndef DRV8833_H
#define DRV8833_H

#include "stm32f4xx_hal.h"
#include <cstdlib>

class DRV8833 {
public:
  DRV8833(TIM_HandleTypeDef* htim,
          uint32_t in1_channel,
          uint32_t in2_channel);

  HAL_StatusTypeDef begin();

  void setPower(float power);   // -1.0 to +1.0
  void setDuty(int16_t duty);   // -1000 to +1000

  void brake();
  void coast();

private:
  void write(uint16_t in1, uint16_t in2);
  uint16_t maxDuty() const;

  TIM_HandleTypeDef* htim_;
  uint32_t in1_channel_;
  uint32_t in2_channel_;
};

#endif /* DRV8833_H */