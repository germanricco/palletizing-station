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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define A1X_Pin GPIO_PIN_0
#define A1X_GPIO_Port GPIOA
#define A1Y_Pin GPIO_PIN_1
#define A1Y_GPIO_Port GPIOA
#define A2X_Pin GPIO_PIN_2
#define A2X_GPIO_Port GPIOA
#define A2Y_Pin GPIO_PIN_3
#define A2Y_GPIO_Port GPIOA
#define S1_Pin GPIO_PIN_6
#define S1_GPIO_Port GPIOA
#define S2_Pin GPIO_PIN_7
#define S2_GPIO_Port GPIOA
#define S3_Pin GPIO_PIN_0
#define S3_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define MotorDC_Pin GPIO_PIN_10
#define MotorDC_GPIO_Port GPIOB
#define BombaSuccion_Pin GPIO_PIN_15
#define BombaSuccion_GPIO_Port GPIOA
#define ValvulaRetencion_Pin GPIO_PIN_3
#define ValvulaRetencion_GPIO_Port GPIOB
#define A1P_Pin GPIO_PIN_5
#define A1P_GPIO_Port GPIOB
#define A1P_EXTI_IRQn EXTI9_5_IRQn
#define A2P_Pin GPIO_PIN_6
#define A2P_GPIO_Port GPIOB
#define A2P_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
