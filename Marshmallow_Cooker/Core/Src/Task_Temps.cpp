#include "Task_Temps.h"
#include "Task.h"

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
        sprintf(print_buf,
                "MCP9600 init failed, status=%d\r\n",
                tc_sensor_.getLastStatus());
        print_str(print_buf);

        state_ = State::Fault;
        return;
      }

      sprintf(print_buf,
              "MCP9600 initialized, device_id=0x%04X\r\n",
              tc_sensor_.getDeviceId());
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

    case State::Reading: {
      status = ir_sensor_.update();

      if (status == HAL_OK) {
        int16_t objectF_x100 = ir_sensor_.getObjectFx100();

        sprintf(print_buf,
                "IR Temp: %d.%02d F\n\r",
                objectF_x100 / 100,
                abs(objectF_x100 % 100));
        print_str(print_buf);
      } else {
        print_str("IR Temp read failed\n\r");
      }

      status = tc_sensor_.update();

      if (status == HAL_OK) {
        int16_t hot_fx100 = tc_sensor_.getHotFx100();
        int16_t cold_fx100 = tc_sensor_.getColdFx100();

        sprintf(print_buf,
                "TC Hot: %d.%02d F, Cold: %d.%02d F\r\n",
                hot_fx100 / 100,
                abs(hot_fx100 % 100),
                cold_fx100 / 100,
                abs(cold_fx100 % 100));
        print_str(print_buf);
      } else {
        sprintf(print_buf,
                "TC Temp read failed, status=%d\r\n",
                tc_sensor_.getLastStatus());
        print_str(print_buf);
      }

      state_ = State::Idle;
      break;
    }

    case State::Fault:
      break;
  }
}

void TaskTemps::update() {
  /*
   * Not needed yet because run() handles the periodic update.
   */
}

TaskTemps::State TaskTemps::getState() const {
  return state_;
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