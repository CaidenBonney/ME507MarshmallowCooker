/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Z_STEP_Pin GPIO_PIN_0
#define Z_STEP_GPIO_Port GPIOA
#define Z_DIR_Pin GPIO_PIN_1
#define Z_DIR_GPIO_Port GPIOA
#define Z_ENN_Pin GPIO_PIN_4
#define Z_ENN_GPIO_Port GPIOA
#define Z_DIAG_Pin GPIO_PIN_5
#define Z_DIAG_GPIO_Port GPIOA
#define BAD_SDA_Pin GPIO_PIN_6
#define BAD_SDA_GPIO_Port GPIOA
#define BAD_SCL_Pin GPIO_PIN_7
#define BAD_SCL_GPIO_Port GPIOA
#define Z_TOP_Pin GPIO_PIN_0
#define Z_TOP_GPIO_Port GPIOB
#define Z_BOT_Pin GPIO_PIN_1
#define Z_BOT_GPIO_Port GPIOB
#define R_AIN1_Pin GPIO_PIN_9
#define R_AIN1_GPIO_Port GPIOA
#define R_AIN2_Pin GPIO_PIN_10
#define R_AIN2_GPIO_Port GPIOA
#define EN_CHA_Pin GPIO_PIN_4
#define EN_CHA_GPIO_Port GPIOB
#define EN_CHB_Pin GPIO_PIN_5
#define EN_CHB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
