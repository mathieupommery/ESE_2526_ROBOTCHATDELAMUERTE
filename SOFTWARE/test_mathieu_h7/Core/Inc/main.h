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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
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
#define SD_CS_Pin GPIO_PIN_15
#define SD_CS_GPIO_Port GPIOA
#define BLUE_Pin GPIO_PIN_5
#define BLUE_GPIO_Port GPIOD
#define GREEN_Pin GPIO_PIN_2
#define GREEN_GPIO_Port GPIOD
#define RED_Pin GPIO_PIN_3
#define RED_GPIO_Port GPIOD
#define INT2_Pin GPIO_PIN_2
#define INT2_GPIO_Port GPIOC
#define INT2_EXTI_IRQn EXTI2_IRQn
#define INT1_Pin GPIO_PIN_0
#define INT1_GPIO_Port GPIOC
#define INT1_EXTI_IRQn EXTI0_IRQn
#define ACCEL_CS_Pin GPIO_PIN_1
#define ACCEL_CS_GPIO_Port GPIOC
#define LED0_Pin GPIO_PIN_11
#define LED0_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_10
#define LED1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define SD_SPI_HANDLE hspi1
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
