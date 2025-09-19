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
extern uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
extern uint16_t temp_buf[TEMP_BUF_SIZE];
extern float Anglei[TEMP_BUF_SIZE];
extern uint16_t LaserScan[LASERSCAN_SIZE];

// State machine
extern volatile uint8_t isReading;
extern volatile uint8_t isWriting;
extern volatile uint8_t HalfFlag;
extern volatile uint8_t CompleteFlag;
extern volatile uint8_t PH1;
extern volatile uint8_t PH2;
extern volatile uint8_t isPointcloud;
extern volatile uint8_t start_to_read;
extern volatile uint8_t reading_offset;

extern UART_HandleTypeDef huart1;

// Extrapolated data
extern volatile uint16_t CT_LSN;
extern volatile uint8_t data_size;
extern volatile uint16_t FSA;
extern volatile uint16_t LSA;
extern volatile uint16_t checksum;

// Data handling
extern volatile uint16_t read_index;
extern volatile uint16_t write_index;
extern volatile uint16_t XOR;
extern volatile float start_angle;
extern volatile float last_angle;
extern volatile float AngleDiff;
extern volatile float AngleCorrect;
extern volatile LaserScan_index;




/* USER CODE END Variables */
/* Definitions for main */
osThreadId_t mainHandle;
const osThreadAttr_t main_attributes = {
  .name = "main",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ParseLiDAR */
osThreadId_t ParseLiDARHandle;
const osThreadAttr_t ParseLiDAR_attributes = {
  .name = "ParseLiDAR",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMain(void *argument);
void StartParseLiDAR(void *argument);

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
  /* creation of main */
  mainHandle = osThreadNew(StartMain, NULL, &main_attributes);

  /* creation of ParseLiDAR */
  ParseLiDARHandle = osThreadNew(StartParseLiDAR, NULL, &ParseLiDAR_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMain */
/**
  * @brief  Function implementing the main thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMain */
void StartMain(void *argument)
{
  /* USER CODE BEGIN StartMain */

	//TickType_t xLastWakeTime = xTaskGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(500);
  }
  /* USER CODE END StartMain */
}

/* USER CODE BEGIN Header_StartParseLiDAR */
/**
* @brief Function implementing the ParseLiDAR thread.
* @param argument: Not used
* @retval None
*/

/* USER CODE END Header_StartParseLiDAR */
void StartParseLiDAR(void *argument)
{
  /* USER CODE BEGIN StartParseLiDAR */

  /* Infinite loop */
  for(;;)
  {
	  if (isReading && (HalfFlag || CompleteFlag)){
		  while (read_index != (start_to_read + UART_RX_BUFFER_SIZE)){
			  PH1 = (uart_rx_buffer[read_index] == 0xAA)
					  ? 1 : 0;
			  PH2 = (PH1 && uart_rx_buffer[(read_index + 1) % UART_RX_BUFFER_SIZE] == 0x55)
					  ? 1 : 0;
			  if (PH1){
				  if (PH2 && (uart_rx_buffer[(read_index + 2) % UART_RX_BUFFER_SIZE] & 0x01) == 0){
					  isPointcloud = 1;
				  }
				  else{
					  PH1 = 0;
					  PH2 = 0;
				  }
			  }
			  if (isPointcloud && (start_to_read + UART_RX_BUFFER_SIZE - read_index) > 10){
				  CT_LSN = ((uart_rx_buffer[(read_index + 3) % UART_RX_BUFFER_SIZE]) << 8) |
						  (uart_rx_buffer[(read_index + 2) % UART_RX_BUFFER_SIZE]);

				  data_size = uart_rx_buffer[read_index + 3];

				  FSA = ((uart_rx_buffer[(read_index + 5) % UART_RX_BUFFER_SIZE]) << 8) |
								  (uart_rx_buffer[(read_index + 4) % UART_RX_BUFFER_SIZE]);

				  LSA = ((uart_rx_buffer[(read_index + 7) % UART_RX_BUFFER_SIZE]) <<8) |
								  (uart_rx_buffer[(read_index + 6) % UART_RX_BUFFER_SIZE]);

				  checksum = (uint16_t)((uart_rx_buffer[(read_index + 8) % UART_RX_BUFFER_SIZE]) <<8) |
								  (uart_rx_buffer[(read_index + 9) % UART_RX_BUFFER_SIZE]);
				  read_index += 10;
				  if ((start_to_read + UART_RX_BUFFER_SIZE - read_index) >= data_size){
//					  printf("%02x\r\n", ((CT_LSN & 0x00FF) << 8) | (CT_LSN & 0xFF00) >> 8);
//					  printf("%02x\r\n", ((FSA & 0x00FF )<< 8) | (FSA & 0xFF00) >> 8);
//					  printf("%02x\r\n", ((LSA & 0x00FF) << 8) | (LSA & 0xFF00) >> 8);
					  XOR = 0xAA55 ^
							  (((CT_LSN & 0x00FF) << 8) | ((CT_LSN & 0xFF00) >> 8)) ^
							  (((FSA & 0x00FF )<< 8) | ((FSA & 0xFF00) >> 8)) ^
							  (((LSA & 0x00FF) << 8) | ((LSA & 0xFF00) >> 8));
					  for (int i = 0; i < data_size*2; i+=2){
//						  printf("%02x\r\n",((uart_rx_buffer[(read_index + i)] << 8)|
//								  (uart_rx_buffer[(read_index + i + 1) % UART_RX_BUFFER_SIZE])));
						  XOR ^= (uint16_t)((uart_rx_buffer[(read_index + i)] << 8)|
								  (uart_rx_buffer[(read_index + i + 1) % UART_RX_BUFFER_SIZE]));
					  }
					  if (checksum == XOR){ //store data
						  FSA = (FSA >> 1) / 64;
						  LSA = (LSA >> 1) / 64;
						  for (int i = 0; i < data_size * 2; i += 2){
							  temp_buf[(int)(roundf(i / 2.0f))] = ((uart_rx_buffer[(read_index + i + 1) % UART_RX_BUFFER_SIZE] << 8)|
									  	  	  (uart_rx_buffer[(read_index + i) % UART_RX_BUFFER_SIZE])) / 4;
						  }
						  isWriting = 1;
						  PH1 = 0;
						  PH2 = 0;
						  isPointcloud = 0;
					  }
					  else {
						  PH1 = 0;
						  PH2 = 0;
						  isPointcloud = 0;
					  }
					  read_index = (read_index + data_size*2 - 1) % UART_RX_BUFFER_SIZE;
				  }
				  else { //handle incomplete message
//					  isReading = 0;
					  HalfFlag = 0;
					  CompleteFlag = 0;
					  break;
				  }
			  }
			  if (isWriting){
			  		  if (LSA < FSA) {
			  		      LSA += 360.0;
			  		  }
			  		  AngleDiff = LSA - FSA;
			  		  for (int i = 2; i < data_size; i ++){
			  			  AngleCorrect = ((temp_buf[i]) == 0) ? 0 : atanf(21.8 * ((155.3 - temp_buf[i]) / (155.3 * temp_buf[i])));
			  			  LaserScan_index = (int)(roundf((float)((((/*(*/(AngleDiff / (data_size -1)) * (i-1) + FSA + AngleCorrect) /* * 2.0f) / 2.0f*/))/** 2*/)))% LASERSCAN_SIZE;
//			  			  printf("Index %d\r\n", LaserScan_index);
			  		  }
			  		  for (int i = 0; i < data_size; i++){
			  			  LaserScan[LaserScan_index] = temp_buf[i];
//						  printf("%d\r\n", LaserScan[LaserScan_index]);
			  		  }
			  		  printf("--START--\r\n");
			  		  for (int i = 0; i < LASERSCAN_SIZE; i++){
			  			  printf("%d\r\n", LaserScan[i]);
			  		  }
			  		  isWriting = 0;
			  }
			  read_index = (read_index + 1) % UART_RX_BUFFER_SIZE;
		  }
	  osDelay(1);
	  }
  }
  /* USER CODE END StartParseLiDAR */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

