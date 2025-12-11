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
#include "adxl343.h"
#include "ssd1306.h"
#include "led.h"
#include "mcc_com_master.h"
#include "odometry.h"
#include "control.h"
#include "vl53l0x.h"
#include "lidar.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include <stdlib.h>
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
#define LIDAR_SPEED_Pin GPIO_PIN_7
#define LIDAR_SPEED_GPIO_Port GPIOB
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

/* ============================================
 *  TIME-OF-FLIGHT SENSORS
 * ============================================ */

 /* TOF1  → I2C4
  * SDA : PD13
  * SCL : PD12
  */

 /* TOF2  → I2C1
  * SDA : PB9
  * SCL : PB8
  */

 /* TOF3  → I2C3
  * SDA : PC9
  * SCL : PA8
  */


/* ===========================
 *  OLED DISPLAY
 * =========================== */

 /* OLED → I2C2
  * SDA : PB11
  * SCL : PB10
  */


/* ===========================
 *  MOTOR CONTROLLER (UART1)
 * =========================== */

 /* MOTOR UART1
  * TX : PB14
  * RX : PB15
  * Baudrate : 921600
  */


/* ===========================
 *  LIDAR (UART8)
 * =========================== */

 /* LIDAR UART8
  * TX : PE1
  * RX : PE0
  * Baudrate : 115200 (ou 128000 suivant module)
  */


/* ===========================
 *  ESP32 LINK (UART3)
 * =========================== */

 /* ESP32 UART3
  * TX : PD8
  * RX : PD9
  * Baudrate : (à définir : 921600 / 460800 / 115200)
  */


/* I2C handlers (address form) */
#define TOF1_I2C        (&hi2c4)
#define TOF2_I2C        (&hi2c1)
#define TOF3_I2C        (&hi2c3)
#define OLED_I2C        (&hi2c2)

/* UART handlers (address form) */
#define MOTOR_UART      (&huart1)
#define LIDAR_UART      (&huart8)
#define ESP_UART        (&huart3)


#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define SD_SPI_HANDLE hspi1

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
