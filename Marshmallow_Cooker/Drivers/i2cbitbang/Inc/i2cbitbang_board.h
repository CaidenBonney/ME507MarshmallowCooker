#ifndef INC_I2CBITBANG_BOARD_H_
#define INC_I2CBITBANG_BOARD_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define I2CBB_IR_SENSOR 0

#define NUMBER_OF_I2CBB_INSTANCES 1

#define I2CBB_DECLARE_STRUCTURE()                                      \
const i2cbbConfig_t i2cbbConfigTable[NUMBER_OF_I2CBB_INSTANCES] =      \
{                                                                      \
    { BAD_SCL_GPIO_Port, BAD_SCL_Pin,                                  \
      BAD_SDA_GPIO_Port, BAD_SDA_Pin }                                 \
}

#endif