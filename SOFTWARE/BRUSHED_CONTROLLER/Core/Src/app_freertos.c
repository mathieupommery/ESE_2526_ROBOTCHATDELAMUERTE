/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcc_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId COMtaskHandle;
osThreadId PIDtaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCOMTask(void const * argument);
void StartPIDTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of COMtask */
  osThreadDef(COMtask, StartCOMTask, osPriorityNormal, 0, 256);
  COMtaskHandle = osThreadCreate(osThread(COMtask), NULL);

  /* definition and creation of PIDtask */
  osThreadDef(PIDtask, StartPIDTask, osPriorityHigh, 0, 512);
  PIDtaskHandle = osThreadCreate(osThread(PIDtask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartCOMTask */
/**
  * @brief  Function implementing the COMtask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCOMTask */
void StartCOMTask(void const * argument)
{
  /* USER CODE BEGIN StartCOMTask */

	TickType_t lastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	  vTaskDelayUntil(&lastWakeTime,pdMS_TO_TICKS(100));
  }
  /* USER CODE END StartCOMTask */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the PIDtask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartPIDTask */
    const TickType_t periodTicks = pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();
    const float dt = (float)MOTOR_CTRL_PERIOD_MS / 1000.0f;
  /* Infinite loop */
  for(;;)
  {
	  MotorControlTask(dt);


	  vTaskDelayUntil(&lastWakeTime, periodTicks);
  }
  /* USER CODE END StartPIDTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

