
/**
  ******************************************************************************
  * @file    mc_parameters.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides definitions of HW parameters specific to the
  *          configuration of the subsystem.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
//cstat -MISRAC2012-Rule-21.1
#include "main.h" //cstat !MISRAC2012-Rule-21.1
//cstat +MISRAC2012-Rule-21.1
#include "parameters_conversion.h"
#include "r3_g0xx_pwm_curr_fdbk.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

#define FREQ_RATIO 1                /* Dummy value for single drive */
#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */

extern  PWMC_R3_1_Handle_t PWM_Handle_M1;

/**
  * @brief  Current sensor parameters Single Drive - three shunt, STM32G0X
  */
const R3_1_Params_t R3_1_Params =
{
/* PWM generation parameters --------------------------------------------------*/
  .RepetitionCounter = REP_COUNTER,
  .Tafter            = TW_AFTER,
  .Tbefore           = TW_BEFORE_R3_1,
  .TIMx              = TIM1,
  .Tcase2            = ((uint16_t)TDEAD + (uint16_t)TNOISE + (uint16_t)TW_BEFORE_R3_1) / 2u,
  .Tcase3            = (uint16_t)TW_BEFORE_R3_1 + (uint16_t)TDEAD + (uint16_t)TRISE,

  .ADCConfig = {
                 (uint32_t)(1<< 7U) | (uint32_t)(1<< 9U),
                 (uint32_t)(1<< 6U) | (uint32_t)(1<< 9U),
                 (uint32_t)(1<< 6U) | (uint32_t)(1<< 9U),
                 (uint32_t)(1<< 6U) | (uint32_t)(1<< 7U),
                 (uint32_t)(1<< 6U) | (uint32_t)(1<< 7U),
                 (uint32_t)(1<< 7U) | (uint32_t)(1<< 9U),
               },
  .ADCScandir = {
LL_ADC_REG_SEQ_SCAN_DIR_FORWARD>>ADC_CFGR1_SCANDIR_Pos,
LL_ADC_REG_SEQ_SCAN_DIR_FORWARD>>ADC_CFGR1_SCANDIR_Pos,
LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD>>ADC_CFGR1_SCANDIR_Pos,
LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD>>ADC_CFGR1_SCANDIR_Pos,
LL_ADC_REG_SEQ_SCAN_DIR_FORWARD>>ADC_CFGR1_SCANDIR_Pos,
LL_ADC_REG_SEQ_SCAN_DIR_BACKWARD>>ADC_CFGR1_SCANDIR_Pos,
                },
  .ADCDataReg1 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                 },

  .ADCDataReg2 = {
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                   &PWM_Handle_M1.ADC1_DMA_converted[1],
                   &PWM_Handle_M1.ADC1_DMA_converted[0],
                 },
};

ScaleParams_t scaleParams_M1 =
{
 .voltage = NOMINAL_BUS_VOLTAGE_V/(1.73205 * 32767), /* sqrt(3) = 1.73205 */
 .current = CURRENT_CONV_FACTOR_INV,
 .frequency = U_RPM/SPEED_UNIT
};

/* USER CODE BEGIN Additional parameters */

/* USER CODE END Additional parameters */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

