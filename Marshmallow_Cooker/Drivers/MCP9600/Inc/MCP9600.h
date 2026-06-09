#ifndef MCP9600_H
#define MCP9600_H

#include "stm32f4xx_hal.h"

class MCP9600 {
public:

  enum class ThermocoupleType : uint8_t {
    TYPE_K = 0,
    TYPE_J = 1,
    TYPE_T = 2,
    TYPE_N = 3,
    TYPE_S = 4,
    TYPE_E = 5,
    TYPE_B = 6,
    TYPE_R = 7,
  };

  enum class AdcResolution : uint8_t {
    BITS_18 = 0,
    BITS_16 = 1,
    BITS_14 = 2,
    BITS_12 = 3,
  };

  static constexpr uint8_t DEFAULT_ADDRESS = 0x60;

  MCP9600(I2C_HandleTypeDef* hi2c,
          ThermocoupleType type = ThermocoupleType::TYPE_T,
          uint8_t address = DEFAULT_ADDRESS);

  HAL_StatusTypeDef begin();

  HAL_StatusTypeDef update();

  int16_t getHotCx100() const;
  int16_t getColdCx100() const;
  int16_t getDeltaCx100() const;

  int16_t getHotFx100() const;
  int16_t getColdFx100() const;

  uint8_t getStatus() const;
  uint16_t getDeviceId() const;
  HAL_StatusTypeDef getLastStatus() const;

  HAL_StatusTypeDef setThermocoupleType(ThermocoupleType type);
  HAL_StatusTypeDef setAdcResolution(AdcResolution resolution);
  HAL_StatusTypeDef setFilterCoefficient(uint8_t coefficient);
  HAL_StatusTypeDef setNormalMode();
  HAL_StatusTypeDef setShutdownMode();

private:
  static constexpr uint8_t kRegHotJunction = 0x00;
  static constexpr uint8_t kRegDeltaJunction = 0x01;
  static constexpr uint8_t kRegColdJunction = 0x02;
  static constexpr uint8_t kRegRawAdc = 0x03;
  static constexpr uint8_t kRegStatus = 0x04;
  static constexpr uint8_t kRegSensorConfig = 0x05;
  static constexpr uint8_t kRegDeviceConfig = 0x06;
  static constexpr uint8_t kRegDeviceId = 0x20;

  static constexpr uint8_t kDeviceIdUpperExpected = 0x40;

  static constexpr uint32_t kI2cTimeoutMs = 100;

  HAL_StatusTypeDef read8(uint8_t reg, uint8_t* value);
  HAL_StatusTypeDef read16(uint8_t reg, int16_t* value);
  HAL_StatusTypeDef readU16(uint8_t reg, uint16_t* value);
  HAL_StatusTypeDef write8(uint8_t reg, uint8_t value);

  int16_t rawTempToCx100(int16_t raw) const;
  int16_t cToFx100(int16_t cx100) const;

  I2C_HandleTypeDef* hi2c_;
  uint8_t address_;
  ThermocoupleType type_;
  HAL_StatusTypeDef last_status_;

  uint8_t status_;
  uint16_t device_id_;

  int16_t hot_cx100_;
  int16_t cold_cx100_;
  int16_t delta_cx100_;
};

#endif /* MCP9600_H */