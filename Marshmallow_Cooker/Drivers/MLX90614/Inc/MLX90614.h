#pragma once

#include "stm32f4xx_hal.h"

class MLX90614 {
public:
    static constexpr uint8_t DEFAULT_ADDRESS = 0x5A;

    explicit MLX90614(I2C_HandleTypeDef* hi2c,
                      uint8_t address = DEFAULT_ADDRESS);

    HAL_StatusTypeDef isConnected();

    HAL_StatusTypeDef readAmbientC(float& temperatureC);
    HAL_StatusTypeDef readAmbientF(float& temperatureF);

    HAL_StatusTypeDef readObjectC(float& temperatureC);
    HAL_StatusTypeDef readObjectF(float& temperatureF);

    HAL_StatusTypeDef update();
    float getAmbientC() const;
    float getAmbientF() const;
    float getObjectC() const;
    float getObjectF() const;
    HAL_StatusTypeDef getLastStatus() const;

private:
    static constexpr uint8_t REG_AMBIENT_TEMP = 0x06;
    static constexpr uint8_t REG_OBJECT_TEMP  = 0x07;

    I2C_HandleTypeDef* i2c_;
    uint16_t address_;

    float ambient_temperature_c_ = 0.0f;
    float object_temperature_c_ = 0.0f;
    HAL_StatusTypeDef last_status_ = HAL_ERROR;

    HAL_StatusTypeDef readTemperatureRegister(uint8_t reg,
                                              float& temperatureC);

    float celsiusToFahrenheit(float celsius);
};
