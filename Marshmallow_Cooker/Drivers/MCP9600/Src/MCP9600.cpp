/**
 * @file MCP9600.cpp
 * @brief Implementation of the MCP9600 thermocouple amplifier driver.
 */

#include "MCP9600.h"

/**
 * @brief Construct an MCP9600 driver instance.
 * @param hi2c Pointer to the STM32 HAL I2C handle used for communication.
 * @param type Thermocouple type used during initialization.
 * @param address 7-bit I2C address of the MCP9600.
 */
MCP9600::MCP9600(I2C_HandleTypeDef* hi2c, ThermocoupleType type, uint8_t address)
    : hi2c_(hi2c),
      address_(address),
      type_(type),
      last_status_(HAL_ERROR),
      status_(0),
      device_id_(0),
      hot_cx100_(0),
      cold_cx100_(0),
      delta_cx100_(0) {
}

/**
 * @brief Initialize and configure the MCP9600.
 * @return HAL_OK when the device ID and configuration steps succeed.
 */
HAL_StatusTypeDef MCP9600::begin() {
  last_status_ = readU16(kRegDeviceId, &device_id_);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  const uint8_t device_id_upper = static_cast<uint8_t>((device_id_ >> 8) & 0xFF);

  if (device_id_upper != kDeviceIdUpperExpected) {
    last_status_ = HAL_ERROR;
    return last_status_;
  }

  last_status_ = setNormalMode();

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  last_status_ = setThermocoupleType(type_);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  last_status_ = setAdcResolution(AdcResolution::BITS_18);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  last_status_ = setFilterCoefficient(2);

  return last_status_;
}

/**
 * @brief Read status and update cached hot, cold, and delta temperatures.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::update() {
  int16_t raw_hot = 0;
  int16_t raw_cold = 0;
  int16_t raw_delta = 0;

  last_status_ = read8(kRegStatus, &status_);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  last_status_ = read16(kRegHotJunction, &raw_hot);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  last_status_ = read16(kRegColdJunction, &raw_cold);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  last_status_ = read16(kRegDeltaJunction, &raw_delta);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  hot_cx100_ = rawTempToCx100(raw_hot);
  cold_cx100_ = rawTempToCx100(raw_cold);
  delta_cx100_ = rawTempToCx100(raw_delta);

  return HAL_OK;
}

/**
 * @brief Get cached hot junction temperature in degrees C x100.
 * @return Hot junction temperature in degrees C x100.
 */
int16_t MCP9600::getHotCx100() const {
  return hot_cx100_;
}

/**
 * @brief Get cached cold junction temperature in degrees C x100.
 * @return Cold junction temperature in degrees C x100.
 */
int16_t MCP9600::getColdCx100() const {
  return cold_cx100_;
}

/**
 * @brief Get cached delta junction temperature in degrees C x100.
 * @return Delta junction temperature in degrees C x100.
 */
int16_t MCP9600::getDeltaCx100() const {
  return delta_cx100_;
}

/**
 * @brief Get cached hot junction temperature in degrees F x100.
 * @return Hot junction temperature in degrees F x100.
 */
int16_t MCP9600::getHotFx100() const {
  return cToFx100(hot_cx100_);
}

/**
 * @brief Get cached cold junction temperature in degrees F x100.
 * @return Cold junction temperature in degrees F x100.
 */
int16_t MCP9600::getColdFx100() const {
  return cToFx100(cold_cx100_);
}

/**
 * @brief Get the most recently cached MCP9600 status register value.
 * @return Cached status register value.
 */
uint8_t MCP9600::getStatus() const {
  return status_;
}

/**
 * @brief Get the most recently read MCP9600 device ID.
 * @return Cached device ID register value.
 */
uint16_t MCP9600::getDeviceId() const {
  return device_id_;
}

/**
 * @brief Get the most recent HAL status returned by this driver.
 * @return Last HAL status.
 */
HAL_StatusTypeDef MCP9600::getLastStatus() const {
  return last_status_;
}

/**
 * @brief Set the thermocouple type in the sensor configuration register.
 * @param type Thermocouple type to configure.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::setThermocoupleType(ThermocoupleType type) {
  uint8_t config = 0;

  last_status_ = read8(kRegSensorConfig, &config);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  config &= static_cast<uint8_t>(~0x70);
  config |= static_cast<uint8_t>((static_cast<uint8_t>(type) & 0x07) << 4);

  last_status_ = write8(kRegSensorConfig, config);
  return last_status_;
}

/**
 * @brief Set the ADC resolution in the device configuration register.
 * @param resolution ADC resolution selection.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::setAdcResolution(AdcResolution resolution) {
  uint8_t config = 0;

  last_status_ = read8(kRegDeviceConfig, &config);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  config &= static_cast<uint8_t>(~0x60);
  config |= static_cast<uint8_t>((static_cast<uint8_t>(resolution) & 0x03) << 5);

  last_status_ = write8(kRegDeviceConfig, config);
  return last_status_;
}

/**
 * @brief Set the digital filter coefficient.
 * @param coefficient Filter coefficient from 0 to 7. Values above 7 are clamped.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::setFilterCoefficient(uint8_t coefficient) {
  if (coefficient > 7) {
    coefficient = 7;
  }

  uint8_t config = 0;

  last_status_ = read8(kRegSensorConfig, &config);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  config &= static_cast<uint8_t>(~0x07);
  config |= static_cast<uint8_t>(coefficient & 0x07);

  last_status_ = write8(kRegSensorConfig, config);
  return last_status_;
}

/**
 * @brief Put the MCP9600 into normal conversion mode.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::setNormalMode() {
  uint8_t config = 0;

  last_status_ = read8(kRegDeviceConfig, &config);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  config &= static_cast<uint8_t>(~0x03);

  last_status_ = write8(kRegDeviceConfig, config);
  return last_status_;
}

/**
 * @brief Put the MCP9600 into shutdown mode.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::setShutdownMode() {
  uint8_t config = 0;

  last_status_ = read8(kRegDeviceConfig, &config);

  if (last_status_ != HAL_OK) {
    return last_status_;
  }

  config &= static_cast<uint8_t>(~0x03);
  config |= 0x01;

  last_status_ = write8(kRegDeviceConfig, config);
  return last_status_;
}

/**
 * @brief Read the raw hot junction register.
 * @param raw Pointer that receives the raw signed register value.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::readRawHot(int16_t* raw) {
  return read16(kRegHotJunction, raw);
}

/**
 * @brief Read the raw cold junction register.
 * @param raw Pointer that receives the raw signed register value.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::readRawCold(int16_t* raw) {
  return read16(kRegColdJunction, raw);
}

/**
 * @brief Read the sensor configuration register.
 * @param config Pointer that receives the register value.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::getSensorConfig(uint8_t* config) {
  return read8(kRegSensorConfig, config);
}

/**
 * @brief Read the signed 24-bit raw ADC value.
 * @param raw_adc Pointer that receives the sign-extended 32-bit raw ADC value.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::readRawAdc(int32_t* raw_adc) {
  uint8_t buffer[3] = {0, 0, 0};

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c_,
                                              static_cast<uint16_t>(address_ << 1),
                                              kRegRawAdc,
                                              I2C_MEMADD_SIZE_8BIT,
                                              buffer,
                                              3,
                                              kI2cTimeoutMs);

  if (status != HAL_OK) {
    return status;
  }

  int32_t value = (static_cast<int32_t>(buffer[0]) << 16) | (static_cast<int32_t>(buffer[1]) << 8) |
                  static_cast<int32_t>(buffer[2]);

  if (value & 0x800000) {
    value |= 0xFF000000;
  }

  *raw_adc = value;
  return HAL_OK;
}

/**
 * @brief Read an 8-bit register.
 * @param reg Register address.
 * @param value Pointer that receives the register value.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::read8(uint8_t reg, uint8_t* value) {
  return HAL_I2C_Mem_Read(hi2c_,
                          static_cast<uint16_t>(address_ << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          value,
                          1,
                          kI2cTimeoutMs);
}

/**
 * @brief Read a signed 16-bit register.
 * @param reg Register address.
 * @param value Pointer that receives the signed register value.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::read16(uint8_t reg, int16_t* value) {
  uint8_t buffer[2] = {0, 0};

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c_,
                                              static_cast<uint16_t>(address_ << 1),
                                              reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              buffer,
                                              2,
                                              kI2cTimeoutMs);

  if (status != HAL_OK) {
    return status;
  }

  *value = static_cast<int16_t>(static_cast<uint16_t>(buffer[0] << 8) | static_cast<uint16_t>(buffer[1]));

  return HAL_OK;
}

/**
 * @brief Read an unsigned 16-bit register.
 * @param reg Register address.
 * @param value Pointer that receives the unsigned register value.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::readU16(uint8_t reg, uint16_t* value) {
  uint8_t buffer[2] = {0, 0};

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c_,
                                              static_cast<uint16_t>(address_ << 1),
                                              reg,
                                              I2C_MEMADD_SIZE_8BIT,
                                              buffer,
                                              2,
                                              kI2cTimeoutMs);

  if (status != HAL_OK) {
    return status;
  }

  *value = static_cast<uint16_t>(static_cast<uint16_t>(buffer[0] << 8) | static_cast<uint16_t>(buffer[1]));

  return HAL_OK;
}

/**
 * @brief Write an 8-bit register.
 * @param reg Register address.
 * @param value Value to write.
 * @return HAL_OK on success, otherwise the HAL I2C error status.
 */
HAL_StatusTypeDef MCP9600::write8(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(hi2c_,
                           static_cast<uint16_t>(address_ << 1),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1,
                           kI2cTimeoutMs);
}

/**
 * @brief Convert MCP9600 raw temperature register data to degrees C x100.
 * @param raw Raw signed temperature register value.
 * @return Converted temperature in degrees C x100.
 */
int16_t MCP9600::rawTempToCx100(int16_t raw) const {
  // MCP9600 temperature registers are signed 16-bit values,
  // 0.0625 C per LSB.
  // C x100 = raw * 6.25 = raw * 625 / 100.
  return static_cast<int16_t>((static_cast<int32_t>(raw) * 625) / 100);
}

/**
 * @brief Convert fixed-point Celsius to fixed-point Fahrenheit.
 * @param cx100 Temperature in degrees C x100.
 * @return Temperature in degrees F x100.
 */
int16_t MCP9600::cToFx100(int16_t cx100) const {
  // F = C * 9/5 + 32
  return static_cast<int16_t>(((static_cast<int32_t>(cx100) * 9) / 5) + 3200);
}
