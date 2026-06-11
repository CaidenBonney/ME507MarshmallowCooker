#ifndef R_ENCODER_DRIVER_H
#define R_ENCODER_DRIVER_H

/**
 * @file r_encoder_driver.h
 * @brief Quadrature encoder interface for the marshmallow rotisserie axis.
 * @details
 * This file declares the REncoderDriver class, which wraps an STM32 hardware
 * timer configured in encoder mode. The driver converts the timer counter into
 * a signed accumulated position and a per-update velocity estimate for the
 * rotisserie motor. The accumulated position is used by the R motor controller
 * to move the marshmallow holder through commanded angular displacements and to
 * return the rotisserie to its initial orientation.
 *
 * The STM32 encoder timer counter is finite width, so update() handles counter
 * wraparound before adding the measured delta into the signed software position.
 */

#include "stm32f4xx_hal.h"

/**
 * @class REncoderDriver
 * @brief Tracks rotisserie encoder position and incremental velocity.
 * @details
 * REncoderDriver reads the count value from a hardware timer configured for
 * quadrature encoder mode. Each call to update() computes the signed count
 * change since the previous update, corrects for 16-bit counter rollover, and
 * accumulates that change into a long-term software position.
 *
 * The sign convention is chosen to match the RMotorDriver control direction.
 * The class does not configure or start the timer; CubeMX/HAL initialization is
 * expected to configure the encoder timer before this driver is used.
 */
class REncoderDriver {
public:
  /**
   * @brief Construct an encoder driver around an STM32 timer handle.
   * @param htim Pointer to a timer configured in encoder mode.
   * @details
   * The constructor stores the timer handle and initializes the software
   * position and velocity state. The timer counter baseline is finalized by
   * reset(), which is called by the R motor driver during initialization.
   */
  REncoderDriver(TIM_HandleTypeDef* htim);

  /**
   * @brief Sample the hardware encoder counter and update position state.
   * @details
   * Reads the current timer counter, calculates the signed delta from the last
   * sample, corrects for counter rollover, and updates both velocity_ and
   * position_. This function should be called regularly by the R motor driver
   * before using getPosition() or getVelocity().
   */
  void update();

  /**
   * @brief Clear the accumulated encoder position and velocity.
   * @details
   * Sets the software position and velocity to zero and records the current
   * hardware timer count as the new baseline. This should be used when the
   * current physical rotisserie orientation should be treated as zero.
   */
  void reset();

  /**
   * @brief Get the most recent incremental encoder velocity.
   * @return Signed encoder count delta measured during the last update() call.
   * @details
   * This is not scaled to counts per second. It is the signed count change per
   * update period, so its physical units depend on how often update() is called.
   */
  int16_t getVelocity() const;

  /**
   * @brief Get the accumulated encoder position.
   * @return Signed software encoder position in counts since the last reset().
   * @details
   * The returned value can grow beyond the hardware timer range because the
   * driver accumulates wrapped timer deltas into a 32-bit signed position.
   */
  int32_t getPosition() const;

private:
  /** @brief Total count range of the 16-bit encoder timer. */
  static constexpr int32_t kCounterRange = 65536;

  /** @brief Half of the counter range, used to detect timer rollover. */
  static constexpr int32_t kHalfCounterRange = kCounterRange / 2;

  /** @brief STM32 HAL timer handle configured in encoder mode. */
  TIM_HandleTypeDef* htim_;

  /** @brief Signed encoder count delta from the most recent update(). */
  int16_t velocity_;

  /** @brief Accumulated signed encoder position in counts. */
  int32_t position_;

  /** @brief Raw timer counter value recorded during the previous update(). */
  uint32_t last_counter_value_;
};

#endif /* R_ENCODER_DRIVER_H */
