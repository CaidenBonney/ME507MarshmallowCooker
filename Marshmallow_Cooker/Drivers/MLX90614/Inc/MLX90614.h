#ifndef MLX90614_H
#define MLX90614_H

/**
 * @file MLX90614.h
 * @brief Driver interface for the MLX90614 infrared temperature sensor.
 *
 * This driver reads ambient and object temperatures from an MLX90614 over I2C
 * and stores the most recent successful readings in fixed-point degrees C x100.
 */

#include "stm32f4xx_hal.h"
#include <stdint.h>

/**
 * @class MLX90614
 * @brief I2C driver for the MLX90614 non-contact infrared temperature sensor.
 *
 * The class supports connection checks, one-shot temperature reads, periodic
 * updates, cached ambient/object readings, and fixed-point Fahrenheit
 * conversion. Temperatures are represented as degrees C or F multiplied by 100.
 */
class MLX90614 {
public:
  /** @brief Default 7-bit I2C address for the MLX90614. */
  static constexpr uint8_t DEFAULT_ADDRESS = 0x5A;

  /**
   * @brief Construct an MLX90614 driver instance.
   * @param hi2c Pointer to the STM32 HAL I2C handle used for communication.
   * @param address 7-bit I2C address of the MLX90614.
   */
  explicit MLX90614(I2C_HandleTypeDef* hi2c, uint8_t address = DEFAULT_ADDRESS);

  /**
   * @brief Check whether the MLX90614 responds on the I2C bus.
   * @return HAL_OK if the device acknowledges, otherwise the HAL error status.
   */
  HAL_StatusTypeDef isConnected();

  /**
   * @brief Read and cache the ambient and object temperatures.
   * @return HAL_OK if both reads succeed, otherwise HAL_ERROR.
   */
  HAL_StatusTypeDef update();

  /**
   * @brief Read the ambient temperature in degrees C x100.
   * @param temperatureC_x100 Reference that receives the ambient temperature.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readAmbientCx100(int16_t& temperatureC_x100);

  /**
   * @brief Read the ambient temperature in degrees F x100.
   * @param temperatureF_x100 Reference that receives the ambient temperature.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readAmbientFx100(int16_t& temperatureF_x100);

  /**
   * @brief Read the object temperature in degrees C x100.
   * @param temperatureC_x100 Reference that receives the object temperature.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readObjectCx100(int16_t& temperatureC_x100);

  /**
   * @brief Read the object temperature in degrees F x100.
   * @param temperatureF_x100 Reference that receives the object temperature.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readObjectFx100(int16_t& temperatureF_x100);

  /**
   * @brief Get the most recently cached ambient temperature in degrees C x100.
   * @return Cached ambient temperature in degrees C x100.
   */
  int16_t getAmbientCx100() const;

  /**
   * @brief Get the most recently cached ambient temperature in degrees F x100.
   * @return Cached ambient temperature in degrees F x100.
   */
  int16_t getAmbientFx100() const;

  /**
   * @brief Get the most recently cached object temperature in degrees C x100.
   * @return Cached object temperature in degrees C x100.
   */
  int16_t getObjectCx100() const;

  /**
   * @brief Get the most recently cached object temperature in degrees F x100.
   * @return Cached object temperature in degrees F x100.
   */
  int16_t getObjectFx100() const;

  /**
   * @brief Get the status from the most recent update operation.
   * @return Last cached HAL status.
   */
  HAL_StatusTypeDef getLastStatus() const;

private:
  /** @brief MLX90614 RAM register address for ambient temperature. */
  static constexpr uint8_t REG_AMBIENT_TEMP = 0x06;

  /** @brief MLX90614 RAM register address for object temperature. */
  static constexpr uint8_t REG_OBJECT_TEMP = 0x07;

  /** @brief STM32 HAL I2C handle used by the driver. */
  I2C_HandleTypeDef* i2c_;

  /** @brief Left-shifted 8-bit I2C address used by STM32 HAL calls. */
  uint16_t address_;

  /** @brief Most recent ambient temperature in degrees C x100. */
  int16_t ambient_temperature_c_x100_ = 0;

  /** @brief Most recent object temperature in degrees C x100. */
  int16_t object_temperature_c_x100_ = 0;

  /** @brief Status from the most recent update/read operation. */
  HAL_StatusTypeDef last_status_ = HAL_ERROR;

  /**
   * @brief Read and convert an MLX90614 temperature register.
   * @param reg Register address to read.
   * @param temperatureC_x100 Reference that receives the converted temperature in degrees C x100.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readTemperatureRegister(uint8_t reg, int16_t& temperatureC_x100);

  /**
   * @brief Convert fixed-point Celsius to fixed-point Fahrenheit.
   * @param celsius_x100 Temperature in degrees C x100.
   * @return Temperature in degrees F x100.
   */
  static int16_t celsiusToFahrenheitX100(int16_t celsius_x100);
};

#endif /* MLX90614_H */
