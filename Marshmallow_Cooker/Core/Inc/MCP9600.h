#ifndef MCP9600_H
#define MCP9600_H

/**
 * @file MCP9600.h
 * @brief Driver interface for the MCP9600 thermocouple amplifier.
 *
 * This driver configures and reads an MCP9600 over I2C. Temperatures are stored
 * as signed fixed-point values in degrees C x100 or degrees F x100.
 */

#include "stm32f4xx_hal.h"

/**
 * @class MCP9600
 * @brief I2C driver for the MCP9600 thermocouple-to-digital converter.
 *
 * The class initializes the MCP9600, configures thermocouple type, ADC
 * resolution, filter coefficient, and caches hot/cold/delta junction readings.
 */
class MCP9600 {
public:
  /**
   * @enum ThermocoupleType
   * @brief Supported MCP9600 thermocouple type selections.
   */
  enum class ThermocoupleType : uint8_t {
    TYPE_K = 0, /**< Type K thermocouple. */
    TYPE_J = 1, /**< Type J thermocouple. */
    TYPE_T = 2, /**< Type T thermocouple. */
    TYPE_N = 3, /**< Type N thermocouple. */
    TYPE_S = 4, /**< Type S thermocouple. */
    TYPE_E = 5, /**< Type E thermocouple. */
    TYPE_B = 6, /**< Type B thermocouple. */
    TYPE_R = 7, /**< Type R thermocouple. */
  };

  /**
   * @enum AdcResolution
   * @brief MCP9600 ADC resolution settings.
   */
  enum class AdcResolution : uint8_t {
    BITS_18 = 0, /**< 18-bit ADC resolution. */
    BITS_16 = 1, /**< 16-bit ADC resolution. */
    BITS_14 = 2, /**< 14-bit ADC resolution. */
    BITS_12 = 3, /**< 12-bit ADC resolution. */
  };

  /** @brief Default 7-bit I2C address for the MCP9600. */
  static constexpr uint8_t DEFAULT_ADDRESS = 0x60;

  /**
   * @brief Construct an MCP9600 driver instance.
   * @param hi2c Pointer to the STM32 HAL I2C handle used for communication.
   * @param type Thermocouple type used during initialization.
   * @param address 7-bit I2C address of the MCP9600.
   */
  MCP9600(I2C_HandleTypeDef* hi2c, ThermocoupleType type = ThermocoupleType::TYPE_T, uint8_t address = DEFAULT_ADDRESS);

  /**
   * @brief Initialize and configure the MCP9600.
   * @return HAL_OK when the device ID and configuration steps succeed.
   */
  HAL_StatusTypeDef begin();

  /**
   * @brief Read status and update cached hot, cold, and delta temperatures.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef update();

  /** @brief Get cached hot junction temperature in degrees C x100. */
  int16_t getHotCx100() const;

  /** @brief Get cached cold junction temperature in degrees C x100. */
  int16_t getColdCx100() const;

  /** @brief Get cached delta junction temperature in degrees C x100. */
  int16_t getDeltaCx100() const;

  /** @brief Get cached hot junction temperature in degrees F x100. */
  int32_t getHotFx100() const;

  /** @brief Get cached cold junction temperature in degrees F x100. */
  int32_t getColdFx100() const;

  /** @brief Get the most recently cached MCP9600 status register value. */
  uint8_t getStatus() const;

  /** @brief Get the most recently read MCP9600 device ID. */
  uint16_t getDeviceId() const;

  /** @brief Get the most recent HAL status returned by this driver. */
  HAL_StatusTypeDef getLastStatus() const;

  /**
   * @brief Set the thermocouple type in the sensor configuration register.
   * @param type Thermocouple type to configure.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef setThermocoupleType(ThermocoupleType type);

  /**
   * @brief Set the ADC resolution in the device configuration register.
   * @param resolution ADC resolution selection.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef setAdcResolution(AdcResolution resolution);

  /**
   * @brief Set the digital filter coefficient.
   * @param coefficient Filter coefficient from 0 to 7. Values above 7 are clamped.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef setFilterCoefficient(uint8_t coefficient);

  /**
   * @brief Put the MCP9600 into normal conversion mode.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef setNormalMode();

  /**
   * @brief Put the MCP9600 into shutdown mode.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef setShutdownMode();

  /**
   * @brief Read the raw hot junction register.
   * @param raw Pointer that receives the raw signed register value.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readRawHot(int16_t* raw);

  /**
   * @brief Read the raw cold junction register.
   * @param raw Pointer that receives the raw signed register value.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readRawCold(int16_t* raw);

  /**
   * @brief Read the sensor configuration register.
   * @param config Pointer that receives the register value.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef getSensorConfig(uint8_t* config);

  /**
   * @brief Read the signed 24-bit raw ADC value.
   * @param raw_adc Pointer that receives the sign-extended 32-bit raw ADC value.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readRawAdc(int32_t* raw_adc);

private:
  /** @brief Hot junction temperature register address. */
  static constexpr uint8_t kRegHotJunction = 0x00;

  /** @brief Delta junction temperature register address. */
  static constexpr uint8_t kRegDeltaJunction = 0x01;

  /** @brief Cold junction temperature register address. */
  static constexpr uint8_t kRegColdJunction = 0x02;

  /** @brief Raw ADC register address. */
  static constexpr uint8_t kRegRawAdc = 0x03;

  /** @brief Status register address. */
  static constexpr uint8_t kRegStatus = 0x04;

  /** @brief Sensor configuration register address. */
  static constexpr uint8_t kRegSensorConfig = 0x05;

  /** @brief Device configuration register address. */
  static constexpr uint8_t kRegDeviceConfig = 0x06;

  /** @brief Device ID register address. */
  static constexpr uint8_t kRegDeviceId = 0x20;

  /** @brief Expected upper byte of the MCP9600 device ID. */
  static constexpr uint8_t kDeviceIdUpperExpected = 0x40;

  /** @brief I2C transaction timeout in milliseconds. */
  static constexpr uint32_t kI2cTimeoutMs = 100;

  /**
   * @brief Read an 8-bit register.
   * @param reg Register address.
   * @param value Pointer that receives the register value.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef read8(uint8_t reg, uint8_t* value);

  /**
   * @brief Read a signed 16-bit register.
   * @param reg Register address.
   * @param value Pointer that receives the signed register value.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef read16(uint8_t reg, int16_t* value);

  /**
   * @brief Read an unsigned 16-bit register.
   * @param reg Register address.
   * @param value Pointer that receives the unsigned register value.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef readU16(uint8_t reg, uint16_t* value);

  /**
   * @brief Write an 8-bit register.
   * @param reg Register address.
   * @param value Value to write.
   * @return HAL_OK on success, otherwise the HAL I2C error status.
   */
  HAL_StatusTypeDef write8(uint8_t reg, uint8_t value);

  /**
   * @brief Convert MCP9600 raw temperature register data to degrees C x100.
   * @param raw Raw signed temperature register value.
   * @return Converted temperature in degrees C x100.
   */
  int16_t rawTempToCx100(int16_t raw) const;

  /**
   * @brief Convert fixed-point Celsius to fixed-point Fahrenheit.
   * @param cx100 Temperature in degrees C x100.
   * @return Temperature in degrees F x100.
   */
  int32_t cToFx100(int16_t cx100) const;

  /** @brief STM32 HAL I2C handle used by the driver. */
  I2C_HandleTypeDef* hi2c_;

  /** @brief 7-bit I2C address of the MCP9600. */
  uint8_t address_;

  /** @brief Thermocouple type configured for the sensor. */
  ThermocoupleType type_;

  /** @brief Most recent HAL status returned by this driver. */
  HAL_StatusTypeDef last_status_;

  /** @brief Cached MCP9600 status register. */
  uint8_t status_;

  /** @brief Cached MCP9600 device ID. */
  uint16_t device_id_;

  /** @brief Cached hot junction temperature in degrees C x100. */
  int16_t hot_cx100_;

  /** @brief Cached cold junction temperature in degrees C x100. */
  int16_t cold_cx100_;

  /** @brief Cached delta junction temperature in degrees C x100. */
  int16_t delta_cx100_;
};

#endif /* MCP9600_H */
