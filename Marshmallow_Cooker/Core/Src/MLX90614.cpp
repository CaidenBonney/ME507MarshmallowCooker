/**
 * @file MLX90614.cpp
 * @brief Implementation of the MLX90614 infrared temperature sensor driver.
 */

#include "MLX90614.h"

MLX90614::MLX90614(I2C_HandleTypeDef* hi2c, uint8_t address)
    : i2c_(hi2c),
      address_(static_cast<uint16_t>(address << 1)) {
}

HAL_StatusTypeDef MLX90614::isConnected() {
  return HAL_I2C_IsDeviceReady(i2c_, address_, 3, 100);
}

HAL_StatusTypeDef MLX90614::update() {
  int16_t ambientTempC = 0;
  int16_t objectTempC = 0;

  HAL_StatusTypeDef ambientStatus = readAmbientCx100(ambientTempC);
  HAL_StatusTypeDef objectStatus = readObjectCx100(objectTempC);

  if (ambientStatus == HAL_OK) {
    ambient_temperature_c_x100_ = ambientTempC;
  }

  if (objectStatus == HAL_OK) {
    object_temperature_c_x100_ = objectTempC;
  }

  last_status_ = (ambientStatus == HAL_OK && objectStatus == HAL_OK) ? HAL_OK : HAL_ERROR;

  return last_status_;
}

HAL_StatusTypeDef MLX90614::readAmbientCx100(int16_t& temperatureC_x100) {
  return readTemperatureRegister(REG_AMBIENT_TEMP, temperatureC_x100);
}

HAL_StatusTypeDef MLX90614::readAmbientFx100(int16_t& temperatureF_x100) {
  int16_t tempC_x100 = 0;

  HAL_StatusTypeDef status = readTemperatureRegister(REG_AMBIENT_TEMP, tempC_x100);

  if (status != HAL_OK) {
    return status;
  }

  temperatureF_x100 = celsiusToFahrenheitX100(tempC_x100);
  return HAL_OK;
}

HAL_StatusTypeDef MLX90614::readObjectCx100(int16_t& temperatureC_x100) {
  return readTemperatureRegister(REG_OBJECT_TEMP, temperatureC_x100);
}

HAL_StatusTypeDef MLX90614::readObjectFx100(int16_t& temperatureF_x100) {
  int16_t tempC_x100 = 0;

  HAL_StatusTypeDef status = readTemperatureRegister(REG_OBJECT_TEMP, tempC_x100);

  if (status != HAL_OK) {
    return status;
  }

  temperatureF_x100 = celsiusToFahrenheitX100(tempC_x100);
  return HAL_OK;
}

int16_t MLX90614::getAmbientCx100() const {
  return ambient_temperature_c_x100_;
}

int16_t MLX90614::getAmbientFx100() const {
  return celsiusToFahrenheitX100(ambient_temperature_c_x100_);
}

int16_t MLX90614::getObjectCx100() const {
  return object_temperature_c_x100_;
}

int16_t MLX90614::getObjectFx100() const {
  return celsiusToFahrenheitX100(object_temperature_c_x100_);
}

HAL_StatusTypeDef MLX90614::getLastStatus() const {
  return last_status_;
}

HAL_StatusTypeDef MLX90614::readTemperatureRegister(uint8_t reg, int16_t& temperatureC_x100) {
  uint8_t data[3] = {0}; // LSB, MSB, PEC

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(i2c_, address_, reg, I2C_MEMADD_SIZE_8BIT, data, 3, 100);

  if (status != HAL_OK) {
    return status;
  }

  uint16_t raw = static_cast<uint16_t>(data[0]) | static_cast<uint16_t>(data[1] << 8);

  // MLX90614 temperature data is in Kelvin with 0.02 K per LSB.
  // C x100 = raw * 2 - 273.15 C x100.
  temperatureC_x100 = static_cast<int16_t>((raw * 2) - 27315);

  return HAL_OK;
}

int16_t MLX90614::celsiusToFahrenheitX100(int16_t celsius_x100) {
  return static_cast<int16_t>(((int32_t)celsius_x100 * 9) / 5 + 3200);
}
