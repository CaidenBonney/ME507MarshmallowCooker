/**
 * @file MCP9600.cpp
 * @brief Implementation of the MCP9600 thermocouple amplifier driver.
 */

#include "MCP9600.h"

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

int16_t MCP9600::getHotCx100() const {
  return hot_cx100_;
}

int16_t MCP9600::getColdCx100() const {
  return cold_cx100_;
}

int16_t MCP9600::getDeltaCx100() const {
  return delta_cx100_;
}

int32_t MCP9600::getHotFx100() const {
  return cToFx100(hot_cx100_);
}

int32_t MCP9600::getColdFx100() const {
  return cToFx100(cold_cx100_);
}

uint8_t MCP9600::getStatus() const {
  return status_;
}

uint16_t MCP9600::getDeviceId() const {
  return device_id_;
}

HAL_StatusTypeDef MCP9600::getLastStatus() const {
  return last_status_;
}

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

HAL_StatusTypeDef MCP9600::readRawHot(int16_t* raw) {
  return read16(kRegHotJunction, raw);
}

HAL_StatusTypeDef MCP9600::readRawCold(int16_t* raw) {
  return read16(kRegColdJunction, raw);
}

HAL_StatusTypeDef MCP9600::getSensorConfig(uint8_t* config) {
  return read8(kRegSensorConfig, config);
}

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

HAL_StatusTypeDef MCP9600::read8(uint8_t reg, uint8_t* value) {
  return HAL_I2C_Mem_Read(hi2c_,
                          static_cast<uint16_t>(address_ << 1),
                          reg,
                          I2C_MEMADD_SIZE_8BIT,
                          value,
                          1,
                          kI2cTimeoutMs);
}

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

HAL_StatusTypeDef MCP9600::write8(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(hi2c_,
                           static_cast<uint16_t>(address_ << 1),
                           reg,
                           I2C_MEMADD_SIZE_8BIT,
                           &value,
                           1,
                           kI2cTimeoutMs);
}

int16_t MCP9600::rawTempToCx100(int16_t raw) const {
  // MCP9600 temperature registers are signed 16-bit values,
  // 0.0625 C per LSB.
  // C x100 = raw * 6.25 = raw * 625 / 100.
  return static_cast<int16_t>((static_cast<int32_t>(raw) * 625) / 100);
}

int32_t MCP9600::cToFx100(int16_t cx100) const {
  // F = C * 9/5 + 32
  return ((static_cast<int32_t>(cx100) * 9) / 5) + 3200;
}
