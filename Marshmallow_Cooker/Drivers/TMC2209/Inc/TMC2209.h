#ifndef TMC2209_H
#define TMC2209_H

/**
 * @file TMC2209.h
 * @brief Minimal step/direction wrapper for a TMC2209 stepper driver.
 * @details
 *   This class owns the STM32 GPIO pins connected to a TMC2209 module and
 *   provides enable, direction, step timing, and diagnostic-pin helpers. It is
 *   intentionally lightweight; position control and limit handling are managed
 *   by ZMotorDriver.
 */

#include "main.h"
#include "stm32f4xx_hal.h"
#include <cstdint>

/**
 * @class TMC2209
 * @brief GPIO-level interface for step, direction, enable, and diagnostic pins.
 * @details
 *   The class uses the Cortex-M cycle counter to time microsecond step pulses
 *   and step intervals. It does not use UART configuration of the TMC2209; it
 *   assumes standalone driver configuration through board wiring or defaults.
 */
class TMC2209 {
public:
  /**
   * @enum Direction
   * @brief Electrical direction values applied to the DIR pin.
   */
  enum class Direction {
    Forward, /**< Forward electrical direction. */
    Reverse /**< Reverse electrical direction. */
  };

  /**
   * @brief Construct a TMC2209 pin wrapper.
   * @param step_port GPIO port connected to STEP.
   * @param step_pin GPIO pin mask connected to STEP.
   * @param dir_port GPIO port connected to DIR.
   * @param dir_pin GPIO pin mask connected to DIR.
   * @param enn_port GPIO port connected to enable-not ENN.
   * @param enn_pin GPIO pin mask connected to enable-not ENN.
   * @param diag_port GPIO port connected to DIAG.
   * @param diag_pin GPIO pin mask connected to DIAG.
   */
  TMC2209(GPIO_TypeDef* step_port,
          uint16_t step_pin,
          GPIO_TypeDef* dir_port,
          uint16_t dir_pin,
          GPIO_TypeDef* enn_port,
          uint16_t enn_pin,
          GPIO_TypeDef* diag_port,
          uint16_t diag_pin);

  /** @brief Configure GPIO pins and initialize timing state. */
  void begin();

  /** @brief Enable motor outputs by driving ENN active. */
  void enable();

  /** @brief Disable motor outputs by driving ENN inactive. */
  void disable();

  /**
   * @brief Check whether the driver outputs are enabled.
   * @return true when enable() has placed the driver in the enabled state.
   */
  bool isEnabled() const;

  /**
   * @brief Set the electrical direction output.
   * @param direction Direction value to apply, after direction inversion if enabled.
   */
  void setDirection(Direction direction);

  /**
   * @brief Get the last logical direction command.
   * @return Last Direction passed to setDirection().
   */
  Direction getDirection() const;

  /**
   * @brief Configure direction inversion.
   * @param inverted true to swap Forward and Reverse at the DIR pin.
   */
  void setDirectionInverted(bool inverted);

  /**
   * @brief Check whether direction inversion is enabled.
   * @return true if direction commands are inverted before writing DIR.
   */
  bool directionInverted() const;

  /**
   * @brief Set the desired step pulse rate.
   * @param steps_per_second Step frequency in steps per second.
   */
  void setStepRate(uint32_t steps_per_second);

  /**
   * @brief Get the configured step rate.
   * @return Step frequency in steps per second.
   */
  uint32_t getStepRate() const;

  /**
   * @brief Get the computed interval between steps.
   * @return Step interval in microseconds.
   */
  uint32_t getStepIntervalUs() const;

  /** @brief Immediately emit one STEP pulse if the driver is enabled. */
  void stepNow();

  /**
   * @brief Emit a STEP pulse only if the configured interval has elapsed.
   * @return true if a step was emitted.
   */
  bool stepIfDue();

  /**
   * @brief Read the diagnostic pin.
   * @return true when the DIAG input is active.
   */
  bool diagActive() const;

  /**
   * @brief Update the stored diagnostic state and report changes.
   * @return true when a diagnostic transition or active fault is detected.
   */
  bool updateDiagLog();

  /** @brief Clear the stored diagnostic latch state. */
  void resetDiagLatch();

  /**
   * @brief Get a microsecond timestamp from the cycle counter.
   * @return Approximate microseconds since the cycle counter was enabled.
   */
  uint32_t micros() const;

private:
  /** @brief Minimum STEP high pulse width in microseconds. */
  static constexpr uint32_t kMinStepPulseUs = 3;

  /** @brief Default step frequency used at construction. */
  static constexpr uint32_t kDefaultStepRateStepsPerSecond = 250;

  /** @brief Configure STEP, DIR, ENN, and DIAG GPIO directions. */
  void configureGpioPins();

  /** @brief Enable the Cortex-M DWT cycle counter used for microsecond timing. */
  void enableCycleCounter();

  /**
   * @brief Busy-wait for a short microsecond delay.
   * @param delay_us Delay time in microseconds.
   */
  void delayUs(uint32_t delay_us) const;

  /**
   * @brief Write the physical DIR pin for a logical direction.
   * @param direction Logical direction before optional inversion.
   */
  void writeDirectionPin(Direction direction);

  /** @brief GPIO port connected to STEP. */
  GPIO_TypeDef* step_port_;

  /** @brief GPIO pin mask connected to STEP. */
  uint16_t step_pin_;

  /** @brief GPIO port connected to DIR. */
  GPIO_TypeDef* dir_port_;

  /** @brief GPIO pin mask connected to DIR. */
  uint16_t dir_pin_;

  /** @brief GPIO port connected to ENN. */
  GPIO_TypeDef* enn_port_;

  /** @brief GPIO pin mask connected to ENN. */
  uint16_t enn_pin_;

  /** @brief GPIO port connected to DIAG. */
  GPIO_TypeDef* diag_port_;

  /** @brief GPIO pin mask connected to DIAG. */
  uint16_t diag_pin_;

  /** @brief true when the driver has been enabled. */
  bool enabled_;

  /** @brief true when direction commands are inverted before writing DIR. */
  bool direction_inverted_;

  /** @brief Last sampled diagnostic state. */
  bool last_diag_state_;

  /** @brief Last logical direction command. */
  Direction direction_;

  /** @brief Requested step frequency. */
  uint32_t step_rate_steps_per_second_;

  /** @brief Computed time between steps in microseconds. */
  uint32_t step_interval_us_;

  /** @brief Microsecond timestamp of the previous emitted step. */
  uint32_t last_step_time_us_;
};

#endif
