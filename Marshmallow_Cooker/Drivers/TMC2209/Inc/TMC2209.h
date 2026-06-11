#ifndef TMC2209_H
#define TMC2209_H

/**
 * @file TMC2209.h
 * @brief GPIO step/direction helper for a TMC2209 stepper driver.
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <cstdint>

/**
 * @brief Minimal TMC2209 step/dir interface.
 *
 * This class controls STEP, DIR, ENN, and DIAG pins directly. It does not use
 * UART configuration of the TMC2209.
 */
class TMC2209 {
public:
  /** @brief Physical driver direction before optional inversion. */
  enum class Direction {
    Forward, ///< Forward DIR pin state.
    Reverse ///< Reverse DIR pin state.
  };

  /**
   * @brief Construct a TMC2209 GPIO interface.
   */
  TMC2209(GPIO_TypeDef* step_port,
          uint16_t step_pin,
          GPIO_TypeDef* dir_port,
          uint16_t dir_pin,
          GPIO_TypeDef* enn_port,
          uint16_t enn_pin,
          GPIO_TypeDef* diag_port,
          uint16_t diag_pin);

  /** @brief Initialize pins and timing defaults. */
  void begin();

  /** @brief Enable motor outputs. */
  void enable();

  /** @brief Disable motor outputs. */
  void disable();

  /** @brief Return true when outputs are enabled. */
  bool isEnabled() const;

  /** @brief Set the logical motor direction. */
  void setDirection(Direction direction);

  /** @brief Get the current logical motor direction. */
  Direction getDirection() const;

  /** @brief Set whether logical direction is inverted at the DIR pin. */
  void setDirectionInverted(bool inverted);

  /** @brief Return true when direction inversion is enabled. */
  bool directionInverted() const;

  /** @brief Set the target step rate in steps per second. */
  void setStepRate(uint32_t steps_per_second);

  /** @brief Get the target step rate in steps per second. */
  uint32_t getStepRate() const;

  /** @brief Get the computed interval between step pulses in microseconds. */
  uint32_t getStepIntervalUs() const;

  /** @brief Emit one STEP pulse immediately. */
  void stepNow();

  /** @brief Emit one STEP pulse if the configured interval has elapsed. */
  bool stepIfDue();

  /** @brief Return true when the DIAG input is active. */
  bool diagActive() const;

  /** @brief Update and return the latched diagnostic state. */
  bool updateDiagLog();

  /** @brief Clear the latched diagnostic state. */
  void resetDiagLatch();

  /** @brief Return a microsecond timestamp from the DWT cycle counter. */
  uint32_t micros() const;

private:
  static constexpr uint32_t kMinStepPulseUs = 3; ///< Minimum STEP high pulse width.
  static constexpr uint32_t kDefaultStepRateStepsPerSecond = 250; ///< Default step rate.

  /** @brief Configure GPIO output/input defaults. */
  void configureGpioPins();

  /** @brief Enable the Cortex-M DWT cycle counter for microsecond timing. */
  void enableCycleCounter();

  /** @brief Busy-wait for a specified number of microseconds. */
  void delayUs(uint32_t delay_us) const;

  /** @brief Write the DIR pin for the given logical direction. */
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

#endif /* TMC2209_H */
