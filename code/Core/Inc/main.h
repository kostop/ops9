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
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOC
#define STEP2_Pin GPIO_PIN_0
#define STEP2_GPIO_Port GPIOA
#define STEP_ZERO1_Pin GPIO_PIN_14
#define STEP_ZERO1_GPIO_Port GPIOB
#define STEP_ZERO2_Pin GPIO_PIN_15
#define STEP_ZERO2_GPIO_Port GPIOB
#define STEP1_Pin GPIO_PIN_8
#define STEP1_GPIO_Port GPIOA
#define DIRT1_Pin GPIO_PIN_10
#define DIRT1_GPIO_Port GPIOA
#define DIRT1A11_Pin GPIO_PIN_11
#define DIRT1A11_GPIO_Port GPIOA
#define ANGLE_ZERO_Pin GPIO_PIN_5
#define ANGLE_ZERO_GPIO_Port GPIOB
#define USART_IMU_TX_Pin GPIO_PIN_6
#define USART_IMU_TX_GPIO_Port GPIOB
#define USART_IMU_RX_Pin GPIO_PIN_7
#define USART_IMU_RX_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
