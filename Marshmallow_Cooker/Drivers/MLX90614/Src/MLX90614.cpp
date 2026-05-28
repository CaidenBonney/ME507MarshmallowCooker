#include "MLX90614.h"

MLX90614::MLX90614(I2C_HandleTypeDef* hi2c, uint8_t address)
    : i2c_(hi2c),
      address_(static_cast<uint16_t>(address << 1))
{
}

HAL_StatusTypeDef MLX90614::isConnected()
{
    return HAL_I2C_IsDeviceReady(i2c_, address_, 3, 100);
}

HAL_StatusTypeDef MLX90614::readAmbientC(float& temperatureC)
{
    return readTemperatureRegister(REG_AMBIENT_TEMP, temperatureC);
}

HAL_StatusTypeDef MLX90614::readAmbientF(float& temperatureF)
{
    float tempC = 0.0f;

    HAL_StatusTypeDef status =
        readTemperatureRegister(REG_AMBIENT_TEMP, tempC);

    if (status != HAL_OK) {
        return status;
    }

    temperatureF = celsiusToFahrenheit(tempC);

    return HAL_OK;
}

HAL_StatusTypeDef MLX90614::readObjectC(float& temperatureC)
{
    return readTemperatureRegister(REG_OBJECT_TEMP, temperatureC);
}

HAL_StatusTypeDef MLX90614::readObjectF(float& temperatureF)
{
    float tempC = 0.0f;

    HAL_StatusTypeDef status =
        readTemperatureRegister(REG_OBJECT_TEMP, tempC);

    if (status != HAL_OK) {
        return status;
    }

    temperatureF = celsiusToFahrenheit(tempC);

    return HAL_OK;
}

HAL_StatusTypeDef MLX90614::readTemperatureRegister(uint8_t reg,
                                                    float& temperatureC)
{
    uint8_t data[3] = {0}; // LSB, MSB, PEC

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        i2c_,
        address_,
        reg,
        I2C_MEMADD_SIZE_8BIT,
        data,
        3,
        100
    );

    if (status != HAL_OK) {
        return status;
    }

    uint16_t raw = static_cast<uint16_t>(data[0]) |
                   static_cast<uint16_t>(data[1] << 8);

    temperatureC = (raw * 0.02f) - 273.15f;

    return HAL_OK;
}

float MLX90614::celsiusToFahrenheit(float celsius)
{
    return (celsius * 9.0f / 5.0f) + 32.0f;
}
