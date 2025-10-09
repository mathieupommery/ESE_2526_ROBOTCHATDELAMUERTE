/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MA330.h"
#include "foc.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLDC_PWM_FREQ AUDIO_SAMPLE_RATE
// #define BLDC_PWM_FREQ 10000
#define R_SHUNT	0.01f
#define V_OFFSET_A	1.645f
#define V_OFFSET_B	1.657f
#define POLE_PAIR	(7)
#define SPEED_CONTROL_CYCLE	10
#define FOC_TS (1.0f / (float)BLDC_PWM_FREQ)
#define CAL_ITERATION 100
#define VD_CAL 0.6f
#define VQ_CAL 0.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t adc_data[5];
MA330_t ma330data;
foc_t hfoc;

float angle_deg = 0.0f;
float sp_input = 0.0f;
int start_cal = 0;
_Bool com_init_flag = 0;
_Bool calibration_flag = 0;

uint32_t tps1=0;
uint32_t tps_tot=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin){

	HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_0);


}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){

	HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_0);



}


///******************************************************************************/

//float get_power_voltage(void) {
//	static float pv_filtered = 0.0f;
//  const float filter_alpha = 0.2f;
//
//	// convert to volt
//	float pv = (float)ADC3->JDR1 * ADC_2_POWER_VOLT;
//
//  // Low-pass filter for noise reduction
//	pv_filtered = (1.0f - filter_alpha) * pv_filtered + filter_alpha * pv;
//
//	return pv_filtered;
//}


uint32_t get_dt_us(void) {

  uint32_t elapsed_us = TIM2->CNT;
  TIM2->CNT = 0;
  return elapsed_us;
}


///******************************************************************************/

void encoder_get_error(void) {
	  open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, 0.0f);
	  HAL_Delay(500);
	  float rad_offset = 0.0f;
	  for (int i = 0; i < CAL_ITERATION; i++) {
	    rad_offset += DEG_TO_RAD(ma330data.angle_filtered);
	    HAL_Delay(1);
	  }
	  open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
	  rad_offset = rad_offset / (float)CAL_ITERATION;
	  hfoc.m_angle_offset = rad_offset;
  }

//void svpwm_test(void) {
//  open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, 0.0f);
//  HAL_Delay(500);
//
//  for (int i = 0; i < ERROR_LUT_SIZE; i++) {
//    float mech_deg = (float)i * (360.0f / (float)ERROR_LUT_SIZE);
//    float elec_rad = DEG_TO_RAD(mech_deg * POLE_PAIR);
//    open_loop_voltage_control(&hfoc, VD_CAL, VQ_CAL, elec_rad);
//    HAL_Delay(10);
//    // float pwm[3];
//    // pwm[0] = TIM1->CCR1;
//    // pwm[1] = TIM1->CCR2;
//    // pwm[2] = TIM1->CCR3;
//    // send_data_float(pwm, 3);
//    float rad[2];
//    rad[0] = hfoc.e_angle_rad_comp;
//    rad[1] = elec_rad;
//    norm_angle_rad(&rad[0]);
//    norm_angle_rad(&rad[1]);
//    send_data_float(rad, 2);
//  }
//
//  open_loop_voltage_control(&hfoc, 0.0f, 0.0f, 0.0f);
//}
//
///******************************************************************************/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  MA330_Init(&ma330data, HALL_CS_GPIO_Port, HALL_CS_Pin,NORMAL_FW);

  __HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_TRIGGER | TIM_IT_COM | TIM_IT_BREAK);
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE | TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4 | TIM_IT_TRIGGER | TIM_IT_COM | TIM_IT_BREAK);
  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);


	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

	//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_data, 5);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


	HAL_TIM_Base_Start(&htim2);







  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

//		    if (is_foc_ready()) {
//		      foc_reset_flag();
//
//		      switch (hfoc.control_mode) {
//		      case TORQUE_CONTROL_MODE:
//		        hfoc.id_ref = 0.0f;
//		        hfoc.iq_ref = sp_input;
//		        break;
//		      case SPEED_CONTROL_MODE: {
//		        foc_speed_control_update(&hfoc, sp_input);
//		        break;
//		      }
//		      case POSITION_CONTROL_MODE:
//		        hfoc.actual_angle = MA330_get_actual_degree(&ma330data);
//		        foc_position_control_update(&hfoc, sp_input);
//		        break;
//		      case CALIBRATION_MODE:
//		        break;
//		      case TEST_MODE:
//		        open_loop_voltage_control(&hfoc, 0.0f, 0.1f, 0.0f);
//		        break;
//		      default:
//		        break;
//		      }
//		    }
//
//		    if (hfoc.control_mode == CALIBRATION_MODE) {
//		      if (start_cal == 1) {
//		        encoder_get_error();
//		        // svpwm_test();
//		        start_cal = 0;
//		      }
//		    }

		HAL_Delay(5);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
