/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
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
#include "ylidar.h"
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
extern uint16_t ylidar_read_index;
extern uint16_t ylidar_write_index;

extern uint8_t ylidar_finalbuffer[1024];

/* USER CODE END Variables */
osThreadId maintaskHandle;
osThreadId lidarparseHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Startmaintask(void const * argument);
void Startlidarparse(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of maintask */
  osThreadDef(maintask, Startmaintask, osPriorityNormal, 0, 256);
  maintaskHandle = osThreadCreate(osThread(maintask), NULL);

  /* definition and creation of lidarparse */
  osThreadDef(lidarparse, Startlidarparse, osPriorityIdle, 0, 300);
  lidarparseHandle = osThreadCreate(osThread(lidarparse), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_Startmaintask */
/**
  * @brief  Function implementing the maintask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Startmaintask */
void Startmaintask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Startmaintask */
  /* Infinite loop */
  for(;;)
  {

	 //HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_3);
	 //HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_4);
	CDC_Transmit_FS((uint8_t*) ylidar_finalbuffer, 721*sizeof(uint8_t));
    osDelay(1000);
  }
  /* USER CODE END Startmaintask */
}

/* USER CODE BEGIN Header_Startlidarparse */
/**
* @brief Function implementing the lidarparse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startlidarparse */
void Startlidarparse(void const * argument)
{
  /* USER CODE BEGIN Startlidarparse */
  /* Infinite loop */
  for(;;)
  {


      // Attend HALF ou FULL
//      osEvent evt = osSignalWait(SIG_LIDAR_HALF | SIG_LIDAR_FULL, osWaitForever);
//      if (evt.status == osEventSignal) {
//
//          if (evt.value.signals & SIG_LIDAR_HALF) {
//              HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_3);
//              ylidar_fsm();
//              osDelay(10);
//          }
//
//          if (evt.value.signals & SIG_LIDAR_FULL) {
//              HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_4);
//              ylidar_fsm();
//              osDelay(10);
//          }

	  while(ylidar_read_index!=ylidar_write_index){
		  ylidar_fsm();

      }

	  osDelay(10);
  }
  /* USER CODE END Startlidarparse */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
