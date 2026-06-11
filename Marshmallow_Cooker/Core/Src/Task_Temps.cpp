#include "Task_Temps.h"

extern I2C_HandleTypeDef hi2c3;

TaskTemps::TaskTemps()
    : tc_sensor_(&hi2c3),
      ir_sensor_(&hi2c3) {
}

void TaskTemps::run() {
  switch (state_) {
    case State::Uninitialized:
      print_str("I2C3 sensors initializing\r\n");

      if (tc_sensor_.begin() != HAL_OK) {
        sprintf(print_buf, "MCP9600 init failed, status=%d\r\n", tc_sensor_.getLastStatus());
        print_str(print_buf);
        state_ = State::Fault;
        return;
      }

      sprintf(print_buf, "MCP9600 initialized, device_id=0x%04X\r\n", tc_sensor_.getDeviceId());
      print_str(print_buf);

      state_ = State::Idle;
      break;

    case State::Idle: {
      const uint32_t now_ms = HAL_GetTick();

      if ((now_ms - last_update_ms_) >= kUpdatePeriodMs) {
        last_update_ms_ = now_ms;
        state_ = State::Reading;
      }
      break;
    }

    case State::Reading:
      update();
      state_ = State::Idle;
      break;

    case State::Fault:
      break;
  }
}

void TaskTemps::update() {
  status = ir_sensor_.update();

  if (status == HAL_OK) {
    ir_object_fx100_ = ir_sensor_.getObjectFx100();
    valid_ir_reading_ = true;

  } else {
    valid_ir_reading_ = false;
    print_str("IR Temp read failed\r\n");
  }

  status = tc_sensor_.update();

  if (status == HAL_OK) {
    tc_hot_fx100_ = tc_sensor_.getHotFx100();
    tc_cold_fx100_ = tc_sensor_.getColdFx100();
    valid_tc_reading_ = true;

  } else {
    valid_tc_reading_ = false;

    sprintf(print_buf, "TC Temp read failed, status=%d\r\n", tc_sensor_.getLastStatus());
    print_str(print_buf);
  }
}

Task::Status TaskTemps::getStatus() const {
  if (state_ == State::Uninitialized) {
    return Task::Status::Uninitialized;
  }

  if (state_ == State::Fault) {
    return Task::Status::Fault;
  }

  return Task::Status::Running;
}

TaskTemps::State TaskTemps::getState() const {
  return state_;
}

bool TaskTemps::hasValidThermocoupleReading() const {
  return valid_tc_reading_;
}

bool TaskTemps::hasValidIrReading() const {
  return valid_ir_reading_;
}

int16_t TaskTemps::getThermocoupleHotFx100() const {
  return tc_hot_fx100_;
}

int16_t TaskTemps::getThermocoupleColdFx100() const {
  return tc_cold_fx100_;
}

int16_t TaskTemps::getIrObjectFx100() const {
  return ir_object_fx100_;
}

void TaskTemps::printTemperatures() const {
  sprintf(print_buf,
          "IR Temp: %d.%02d F, TC Hot: %d.%02d F, Cold: %d.%02d F\r\n",
          ir_object_fx100_ / 100,
          abs(ir_object_fx100_ % 100),
          tc_hot_fx100_ / 100,
          abs(tc_hot_fx100_ % 100),
          tc_cold_fx100_ / 100,
          abs(tc_cold_fx100_ % 100));
  print_str(print_buf);
}
