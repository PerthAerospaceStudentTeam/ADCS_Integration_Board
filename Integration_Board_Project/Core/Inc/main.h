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
#include "stm32h7xx_hal.h"

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
#define MAG_MOSI_Pin GPIO_PIN_3
#define MAG_MOSI_GPIO_Port GPIOC
#define MTA_PWM1_Pin GPIO_PIN_1
#define MTA_PWM1_GPIO_Port GPIOA
#define MTA_PWM2_Pin GPIO_PIN_2
#define MTA_PWM2_GPIO_Port GPIOA
#define MTB_PWM1_Pin GPIO_PIN_3
#define MTB_PWM1_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define SUN_ZN_Pin GPIO_PIN_6
#define SUN_ZN_GPIO_Port GPIOA
#define SUN_YP_Pin GPIO_PIN_7
#define SUN_YP_GPIO_Port GPIOA
#define SUN_ZP_Pin GPIO_PIN_4
#define SUN_ZP_GPIO_Port GPIOC
#define SUN_XN_Pin GPIO_PIN_5
#define SUN_XN_GPIO_Port GPIOC
#define SUN_YN_Pin GPIO_PIN_0
#define SUN_YN_GPIO_Port GPIOB
#define SUN_XP_Pin GPIO_PIN_1
#define SUN_XP_GPIO_Port GPIOB
#define MAG_CS_Pin GPIO_PIN_2
#define MAG_CS_GPIO_Port GPIOB
#define MAG_SCK_Pin GPIO_PIN_10
#define MAG_SCK_GPIO_Port GPIOB
#define MTC_PWM2_Pin GPIO_PIN_6
#define MTC_PWM2_GPIO_Port GPIOC
#define MTC_PWM1_Pin GPIO_PIN_7
#define MTC_PWM1_GPIO_Port GPIOC
#define MTB_PWM2_Pin GPIO_PIN_15
#define MTB_PWM2_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_4
#define IMU_CS_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_5
#define IMU_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
