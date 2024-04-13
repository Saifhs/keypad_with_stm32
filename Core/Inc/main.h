/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define RS_Pin GPIO_PIN_1
#define RS_GPIO_Port GPIOC
#define E_Pin GPIO_PIN_2
#define E_GPIO_Port GPIOC
#define Led5_Pin GPIO_PIN_2
#define Led5_GPIO_Port GPIOA
#define Led6_Pin GPIO_PIN_4
#define Led6_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOC
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOC
#define C1_Pin GPIO_PIN_0
#define C1_GPIO_Port GPIOB
#define C2_Pin GPIO_PIN_1
#define C2_GPIO_Port GPIOB
#define C3_Pin GPIO_PIN_2
#define C3_GPIO_Port GPIOB
#define Led1_Pin GPIO_PIN_12
#define Led1_GPIO_Port GPIOB
#define Led2_Pin GPIO_PIN_13
#define Led2_GPIO_Port GPIOB
#define Led3_Pin GPIO_PIN_14
#define Led3_GPIO_Port GPIOB
#define Led4_Pin GPIO_PIN_15
#define Led4_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOC
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_6
#define R1_GPIO_Port GPIOB
#define R1_EXTI_IRQn EXTI9_5_IRQn
#define R2_Pin GPIO_PIN_7
#define R2_GPIO_Port GPIOB
#define R2_EXTI_IRQn EXTI9_5_IRQn
#define R3_Pin GPIO_PIN_8
#define R3_GPIO_Port GPIOB
#define R3_EXTI_IRQn EXTI9_5_IRQn
#define R4_Pin GPIO_PIN_9
#define R4_GPIO_Port GPIOB
#define R4_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
