#ifndef TMC2209_H
#define TMC2209_H

#include "main.h"
#include "stm32f4xx_hal.h"
#include <cstdint>

class TMC2209 {
public:
  enum class Direction {
    Forward,
    Reverse
  };

  TMC2209(GPIO_TypeDef* step_port,
          uint16_t step_pin,
          GPIO_TypeDef* dir_port,
          uint16_t dir_pin,
          GPIO_TypeDef* enn_port,
          uint16_t enn_pin,
          GPIO_TypeDef* diag_port,
          uint16_t diag_pin);

  void begin();

  void enable();
  void disable();
  bool isEnabled() const;

  void setDirection(Direction direction);
  Direction getDirection() const;
  void setDirectionInverted(bool inverted);
  bool directionInverted() const;

  void setStepRate(uint32_t steps_per_second);
  uint32_t getStepRate() const;
  uint32_t getStepIntervalUs() const;

  void stepNow();
  bool stepIfDue();

  bool diagActive() const;
  bool updateDiagLog();
  void resetDiagLatch();

  uint32_t micros() const;

private:
  static constexpr uint32_t kMinStepPulseUs = 3;
  static constexpr uint32_t kDefaultStepRateStepsPerSecond = 250;

  void configureGpioPins();
  void enableCycleCounter();
  void delayUs(uint32_t delay_us) const;
  void writeDirectionPin(Direction direction);

  GPIO_TypeDef* step_port_;
  uint16_t step_pin_;
  GPIO_TypeDef* dir_port_;
  uint16_t dir_pin_;
  GPIO_TypeDef* enn_port_;
  uint16_t enn_pin_;
  GPIO_TypeDef* diag_port_;
  uint16_t diag_pin_;

  bool enabled_;
  bool direction_inverted_;
  bool last_diag_state_;
  Direction direction_;

  uint32_t step_rate_steps_per_second_;
  uint32_t step_interval_us_;
  uint32_t last_step_time_us_;
};

#endif
