#ifndef MLX90614_H
#define MLX90614_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

class MLX90614 {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x5A;

  explicit MLX90614(I2C_HandleTypeDef* hi2c, uint8_t address = DEFAULT_ADDRESS);

  HAL_StatusTypeDef isConnected();

  HAL_StatusTypeDef update();

  HAL_StatusTypeDef readAmbientCx100(int16_t& temperatureC_x100);
  HAL_StatusTypeDef readAmbientFx100(int16_t& temperatureF_x100);

  HAL_StatusTypeDef readObjectCx100(int16_t& temperatureC_x100);
  HAL_StatusTypeDef readObjectFx100(int16_t& temperatureF_x100);

  int16_t getAmbientCx100() const;
  int16_t getAmbientFx100() const;

  int16_t getObjectCx100() const;
  int16_t getObjectFx100() const;

  HAL_StatusTypeDef getLastStatus() const;

private:
  static constexpr uint8_t REG_AMBIENT_TEMP = 0x06;
  static constexpr uint8_t REG_OBJECT_TEMP = 0x07;

  I2C_HandleTypeDef* i2c_;
  uint16_t address_;

  int16_t ambient_temperature_c_x100_ = 0;
  int16_t object_temperature_c_x100_ = 0;

  HAL_StatusTypeDef last_status_ = HAL_ERROR;

  HAL_StatusTypeDef readTemperatureRegister(uint8_t reg, int16_t& temperatureC_x100);

  static int16_t celsiusToFahrenheitX100(int16_t celsius_x100);
};

#endif