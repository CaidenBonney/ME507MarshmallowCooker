#include "MLX90614.h"
#include <stdio.h>

extern char print_buf[100];
extern void print_str(const char* str);

MLX90614::MLX90614(i2cbitbang* i2c) : i2c_(i2c) {
}

HAL_StatusTypeDef MLX90614::isConnected() {
  int16_t temp = 0;
  return readTemperatureRegister(REG_OBJECT_TEMP, temp);
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

uint32_t MLX90614::getLastI2CError() const {
  return last_i2c_error_;
}

HAL_StatusTypeDef MLX90614::readTemperatureRegister(uint8_t reg, int16_t& temperatureC_x100) {
  if (i2c_ == nullptr) {
    last_status_ = HAL_ERROR;
    last_i2c_error_ = I2CBB_ERROR_I2CBB;
    return HAL_ERROR;
  }

  uint8_t data[3] = {0}; // data[0] = LSB, data[1] = MSB, data[2] = PEC

  i2c_->readData(reg, data, 3);

  last_i2c_error_ = i2c_->getError();

  if (last_i2c_error_ != I2CBB_ERROR_NONE) {
    last_status_ = HAL_ERROR;
    return HAL_ERROR;
  }

  uint16_t raw = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
  sprintf(print_buf, "reg=0x%02X bytes=%02X %02X %02X raw=%u\n\r", reg, data[0], data[1], data[2], raw);
  print_str(print_buf);

  int32_t temperature_x100 = static_cast<int32_t>(raw) * 2 - 27315;

  temperatureC_x100 = static_cast<int16_t>(temperature_x100);

  last_status_ = HAL_OK;
  return HAL_OK;
}

int16_t MLX90614::celsiusToFahrenheitX100(int16_t celsius_x100) {
  int32_t fahrenheit_x100 = ((static_cast<int32_t>(celsius_x100) * 9) / 5) + 3200;
  return static_cast<int16_t>(fahrenheit_x100);
}