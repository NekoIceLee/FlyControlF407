/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#define DSHOT_Motor3_Pin GPIO_PIN_0
#define DSHOT_Motor3_GPIO_Port GPIOA
#define DSHOT_Motor4_Pin GPIO_PIN_1
#define DSHOT_Motor4_GPIO_Port GPIOA
#define DHSOT_Motor2_Pin GPIO_PIN_2
#define DHSOT_Motor2_GPIO_Port GPIOA
#define DSHOT_Motor1_Pin GPIO_PIN_3
#define DSHOT_Motor1_GPIO_Port GPIOA
#define NRF_CS_Pin GPIO_PIN_4
#define NRF_CS_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_4
#define NRF_IRQ_GPIO_Port GPIOC
#define NRF_IRQ_EXTI_IRQn EXTI4_IRQn
#define NRF_CE_Pin GPIO_PIN_5
#define NRF_CE_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
