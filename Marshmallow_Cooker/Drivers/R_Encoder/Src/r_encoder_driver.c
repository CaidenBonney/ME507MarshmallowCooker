#include "r_encoder_driver.h"

void r_encoder_init(r_encoder_t* encoder, TIM_HandleTypeDef* htim) {
  encoder->htim = htim;
  reset_r_encoder(encoder);
}

void update_r_encoder(r_encoder_t* encoder) {
  uint32_t counter_value = __HAL_TIM_GET_COUNTER(encoder->htim);
  int32_t delta = counter_value - encoder->last_counter_value;

  if (delta > 32768) { // delta > (ar + 1) / 2
    delta -= 65536; // reset in negative dir therefore must subtract ar + 1
  } else if (delta < -32768) { // delta < -(ar + 1) / 2
    delta += 65536; // reset in positive dir therefore must add ar + 1
  }

  encoder->last_counter_value = counter_value;
  encoder->velocity = delta;
  encoder->position += delta;
}

void reset_r_encoder(r_encoder_t* encoder) {
  encoder->velocity = 0;
  encoder->position = 0;
  encoder->last_counter_value = __HAL_TIM_GET_COUNTER(encoder->htim);
}
