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
#include "stm32g0xx_hal.h"

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
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOF
#define MGH_Pin GPIO_PIN_1
#define MGH_GPIO_Port GPIOA
#define MGL_Pin GPIO_PIN_4
#define MGL_GPIO_Port GPIOA
#define VBAT_TEMP_Pin GPIO_PIN_5
#define VBAT_TEMP_GPIO_Port GPIOA
#define ISNSV_Pin GPIO_PIN_6
#define ISNSV_GPIO_Port GPIOA
#define ISNSU_Pin GPIO_PIN_7
#define ISNSU_GPIO_Port GPIOA
#define ISNSW_Pin GPIO_PIN_1
#define ISNSW_GPIO_Port GPIOB
#define OVERCURRENT_Pin GPIO_PIN_12
#define OVERCURRENT_GPIO_Port GPIOB
#define LSU_Pin GPIO_PIN_13
#define LSU_GPIO_Port GPIOB
#define LSV_Pin GPIO_PIN_14
#define LSV_GPIO_Port GPIOB
#define LSW_Pin GPIO_PIN_15
#define LSW_GPIO_Port GPIOB
#define HSU_Pin GPIO_PIN_8
#define HSU_GPIO_Port GPIOA
#define HSV_Pin GPIO_PIN_9
#define HSV_GPIO_Port GPIOA
#define HSW_Pin GPIO_PIN_10
#define HSW_GPIO_Port GPIOA
#define OC_SEL_Pin GPIO_PIN_11
#define OC_SEL_GPIO_Port GPIOA
#define BTN_Pin GPIO_PIN_12
#define BTN_GPIO_Port GPIOA
#define HALL_CS_Pin GPIO_PIN_15
#define HALL_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
