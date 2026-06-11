#include "Task_Temps.h"

extern I2C_HandleTypeDef hi2c3;

TaskTemps::TaskTemps()
    : tc_sensor_(&hi2c3),
      ir_sensor_(&hi2c3),
      last_update_ms_(0) {
}




volatile HAL_StatusTypeDef temperature_sensor_status = HAL_ERROR;
MLX90614 ir_temp_sensor(&hi2c3);
MCP9600 tc_temp_sensor(&hi2c3);


// I2C3 Sensors: Thermocouple and IR temperature sensor
void init_I2C3_sensors(void) {
  print_str("I2C3 sensors initialized\r\n");

  if (tc_temp_sensor.begin() != HAL_OK) {
    sprintf(print_buf, "MCP9600 init failed, status=%d\r\n", tc_temp_sensor.getLastStatus());
    print_str(print_buf);
  } else {
    sprintf(print_buf, "MCP9600 initialized, device_id=0x%04X\r\n", tc_temp_sensor.getDeviceId());
    print_str(print_buf);

    uint8_t sensor_config = 0;

    if (tc_temp_sensor.getSensorConfig(&sensor_config) == HAL_OK) {
      sprintf(print_buf, "MCP9600 sensor_config=0x%02X\r\n", sensor_config);
      print_str(print_buf);
    } else {
      print_str("MCP9600 sensor config read failed\r\n");
    }
  }
}


// Temperature sensor reading every kTemperatureReadPeriodMs milliseconds
    static uint32_t last_temperature_read_ms = 0;
    const uint32_t now_ms = HAL_GetTick();

    if ((now_ms - last_temperature_read_ms) >= kTemperatureReadPeriodMs) {
      last_temperature_read_ms = now_ms;

      // IR Temperature Sensor
      temperature_sensor_status = ir_temp_sensor.update();
      if (temperature_sensor_status == HAL_OK) {
        int16_t objectF_x100 = ir_temp_sensor.getObjectFx100();
        sprintf(print_buf, "IR Temp: %d.%02d F\n\r", objectF_x100 / 100, abs(objectF_x100 % 100));

        print_str(print_buf);
      } else {
        print_str("IR Temp read failed\n\r");
      }
    }

    // Thermocouple Temperature Sensor
    temperature_sensor_status = tc_temp_sensor.update();
    if (temperature_sensor_status == HAL_OK) {
      int16_t hot_fx100 = tc_temp_sensor.getHotFx100();
      int16_t cold_fx100 = tc_temp_sensor.getColdFx100();

      sprintf(print_buf,
              "TC Hot: %d.%02d F, Cold: %d.%02d F\r\n",
              hot_fx100 / 100,
              abs(hot_fx100 % 100),
              cold_fx100 / 100,
              abs(cold_fx100 % 100));
      print_str(print_buf);
    } else {
      sprintf(print_buf, "TC Temp read failed, status=%d\r\n", tc_temp_sensor.getLastStatus());
      print_str(print_buf);
    }