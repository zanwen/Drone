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
#define SI24R1_CSN_Pin GPIO_PIN_4
#define SI24R1_CSN_GPIO_Port GPIOA
#define POWER_KEY_Pin GPIO_PIN_2
#define POWER_KEY_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_14
#define LED4_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define SI24R1_CE_Pin GPIO_PIN_8
#define SI24R1_CE_GPIO_Port GPIOA
#define SI24R1_IRQ_Pin GPIO_PIN_9
#define SI24R1_IRQ_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
