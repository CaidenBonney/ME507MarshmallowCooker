#ifndef TMC2209_H
#define TMC2209_H

#include "stm32f4xx_hal.h"

class TMC2209 {
public:
  TMC2209(TIM_HandleTypeDef* htim,
          uint32_t in1_channel,
          uint32_t in2_channel);

  HAL_StatusTypeDef begin();

  static constexpr int16_t kMaxDuty = 1000; // 100%

  void setDuty(int16_t duty); // -1000 to +1000

  void brake();
  void coast();

private:
  void write(uint16_t in1, uint16_t in2);
  uint16_t getTimerPeriod() const;

  TIM_HandleTypeDef* htim_;
  uint32_t in1_channel_;
  uint32_t in2_channel_;
};

#endif /* TMC2209_H */