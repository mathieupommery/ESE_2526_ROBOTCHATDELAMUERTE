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
#include "adxl343.h"
#include "mcc_com_master.h"
#include "odometry.h"
#include "control.h"
#include "vl53l0x.h"
#include "lidar.h"
#include "led.h"
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

extern adxl343_t adxldata;
extern MOTOR_COM com_struct;
extern Lidar_t lidar_struct;

extern led_strip_t tomandjerry;
extern led_strip_t extled;

extern uint8_t  tomjerry_grb[WS2812_GRB_BUF_LEN(TOM_JERRY_LED_COUNT)];
extern uint32_t tomjerry_dma[WS2812_DMA_BUF_LEN(TOM_JERRY_LED_COUNT)];

extern uint8_t  ext_grb[WS2812_GRB_BUF_LEN(EXT_LED_COUNT)];
extern uint32_t ext_dma[WS2812_DMA_BUF_LEN(EXT_LED_COUNT)];

extern VL53L0X_t tof_array[3];


uint32_t time_sum=0;
uint32_t t0=0;
float time_avg;
uint32_t max_time;
/* USER CODE END Variables */
osThreadId maintaskHandle;
osThreadId Sensor_parseHandle;
osThreadId PimptaskHandle;
osThreadId Odom_taskHandle;
osThreadId ClustertaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Startmaintask(void const * argument);
void Startsensorparse(void const * argument);
void StartPimptask(void const * argument);
void StartOdomtask(void const * argument);
void StartClustertask(void const * argument);

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

  /* definition and creation of Sensor_parse */
  osThreadDef(Sensor_parse, Startsensorparse, osPriorityNormal, 0, 512);
  Sensor_parseHandle = osThreadCreate(osThread(Sensor_parse), NULL);

  /* definition and creation of Pimptask */
  osThreadDef(Pimptask, StartPimptask, osPriorityNormal, 0, 512);
  PimptaskHandle = osThreadCreate(osThread(Pimptask), NULL);

  /* definition and creation of Odom_task */
  osThreadDef(Odom_task, StartOdomtask, osPriorityNormal, 0, 512);
  Odom_taskHandle = osThreadCreate(osThread(Odom_task), NULL);

  /* definition and creation of Clustertask */
  osThreadDef(Clustertask, StartClustertask, osPriorityNormal, 0, 512);
  ClustertaskHandle = osThreadCreate(osThread(Clustertask), NULL);

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
  TickType_t xLastWakeTime;
  const TickType_t period = pdMS_TO_TICKS(5000);
  xLastWakeTime = xTaskGetTickCount();
  int i=0;
  /* Infinite loop */
  for(;;)
  {
//	  switch(i){
//	  case 0:
//		  i++;
//		  break;
//	  case 1:
//		  com_struct.w0=74.0f;
//		  com_struct.w1=195.0f;
//		  com_struct.w2=14.0f;
//		  i--;
//		  break;
//	  }

	   HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_11);
	  vTaskDelayUntil(&xLastWakeTime, period);
  }
  /* USER CODE END Startmaintask */
}

/* USER CODE BEGIN Header_Startsensorparse */
/**
* @brief Function implementing the Sensor_parse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startsensorparse */
void Startsensorparse(void const * argument)
{
  /* USER CODE BEGIN Startsensorparse */
	VL53L0X_Init(&tof_array[0],TOF1_I2C,1,0.3);
	VL53L0X_Init(&tof_array[1],TOF2_I2C,2,0.3);
	VL53L0X_Init(&tof_array[2],TOF3_I2C,3,0.3);

	int i=0;
	uint32_t mid_time=0;

	  TickType_t xLastWakeTime;
	  const TickType_t period = pdMS_TO_TICKS(10);
	  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {

	  ylidar_fsm(&lidar_struct);

	  t0=DWT->CYCCNT;
	  //VL53L0X_Update(&tof_array[0]);
	  //VL53L0X_Update(&tof_array[1]);
	  //VL53L0X_Update(&tof_array[2]);
	  mid_time=(DWT->CYCCNT-t0);
	  time_sum+=mid_time;
	  i+=1;
	  if(i>=100){
		  time_avg=(float)time_sum/100.0f;
		  time_sum=0;
		  i=0;
	  }
	  if(mid_time>max_time){
		  max_time=mid_time;
	  }
	  //ADXL343_ReadXYZ(&adxldata, 100);
	  vTaskDelayUntil(&xLastWakeTime, period);
  }
  /* USER CODE END Startsensorparse */
}

/* USER CODE BEGIN Header_StartPimptask */
/**
* @brief Function implementing the Pimptask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPimptask */
void StartPimptask(void const * argument)
{
  /* USER CODE BEGIN StartPimptask */
    LED_Strip_Init(&tomandjerry,&htim1,TIM_CHANNEL_2,tomjerry_grb,TOM_JERRY_LED_COUNT,tomjerry_dma,(uint16_t)(sizeof(tomjerry_dma) / sizeof(tomjerry_dma[0])));
    LED_Strip_Init(&extled,&htim3,TIM_CHANNEL_4,ext_grb,EXT_LED_COUNT,ext_dma,(uint16_t)(sizeof(ext_dma) / sizeof(ext_dma[0])));
	uint16_t offset=0;




	  TickType_t xLastWakeTime;
	  const TickType_t period = pdMS_TO_TICKS(10);
	  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {

	  LED_Strip_Refresh(&extled);
	  LED_Strip_FillRainbow(&extled,offset,255);
	  offset+=2;


	  vTaskDelayUntil(&xLastWakeTime, period);
  }
  /* USER CODE END StartPimptask */
}

/* USER CODE BEGIN Header_StartOdomtask */
/**
* @brief Function implementing the Odom_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOdomtask */
void StartOdomtask(void const * argument)
{
  /* USER CODE BEGIN StartOdomtask */

	//odometry_Init();
	  TickType_t xLastWakeTime;
	  const TickType_t period = pdMS_TO_TICKS(5);
	  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {



	  MotorCom_Process(&com_struct);
	  vTaskDelayUntil(&xLastWakeTime, period);
  }
  /* USER CODE END StartOdomtask */
}

/* USER CODE BEGIN Header_StartClustertask */
/**
* @brief Function implementing the Clustertask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartClustertask */
void StartClustertask(void const * argument)
{
  /* USER CODE BEGIN StartClustertask */


	  TickType_t xLastWakeTime;
	  const TickType_t period = pdMS_TO_TICKS(100);
	  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {


	  HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_10);
	  vTaskDelayUntil(&xLastWakeTime, period);
  }
  /* USER CODE END StartClustertask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
