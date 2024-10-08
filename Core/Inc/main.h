/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define LedG_Pin GPIO_PIN_12
#define LedG_GPIO_Port GPIOB
#define LedR_Pin GPIO_PIN_13
#define LedR_GPIO_Port GPIOB
#define Disable_Pin GPIO_PIN_10
#define Disable_GPIO_Port GPIOA
#define OUT1_Pin GPIO_PIN_7
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_6
#define OUT2_GPIO_Port GPIOB
#define OUT3_Pin GPIO_PIN_3
#define OUT3_GPIO_Port GPIOA
#define OUT4_Pin GPIO_PIN_2
#define OUT4_GPIO_Port GPIOA
#define Direction_Pin GPIO_PIN_8
#define Direction_GPIO_Port GPIOB
#define Step_Pin GPIO_PIN_9
#define Step_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
