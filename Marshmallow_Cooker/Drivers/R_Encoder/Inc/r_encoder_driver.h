#ifndef R_ENCODER_DRIVER_C
#define R_ENCODER_DRIVER_C
#include "stm32f4xx_hal.h"

typedef struct {
  TIM_HandleTypeDef* htim;
  int16_t velocity;
  int32_t position;
  uint32_t last_counter_value;
} r_encoder_t;

void r_encoder_init(r_encoder_t* encoder, TIM_HandleTypeDef* htim);
void update_r_encoder(r_encoder_t* encoder);
void reset_r_encoder(r_encoder_t* encoder);

#endif
