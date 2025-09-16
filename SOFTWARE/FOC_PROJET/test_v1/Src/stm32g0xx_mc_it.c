
/**
  ******************************************************************************
  * @file    stm32g0xx_mc_it.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Main Interrupt Service Routines.
  *          This file provides exceptions handler and peripherals interrupt
  *          service routine related to Motor Control
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
  * @ingroup STM32G0xx_IRQ_Handlers
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "motorcontrol.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
  * @{
  */

/* USER CODE BEGIN PRIVATE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */

void DMA1_Channel1_IRQHandler(void);

void TIM1_BRK_UP_TRG_COM_IRQHandler(void);

/**
  * @brief  This function handles current regulation interrupt request.
  * @param  None
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 0 */

  /* USER CODE END CURRENT_REGULATION_IRQn 0 */

  /* Clear Flags */
  DMA1->IFCR = (LL_DMA_ISR_GIF1|LL_DMA_ISR_TCIF1|LL_DMA_ISR_HTIF1);

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 1 */

  /* USER CODE END CURRENT_REGULATION_IRQn 1 */

  TSK_HighFrequencyTask();

  /* USER CODE BEGIN CURRENT_REGULATION_IRQn 2 */

  /* USER CODE END CURRENT_REGULATION_IRQn 2 */
}

/**
  * @brief  This function handles first motor TIMx Update, Break-in interrupt request.
  * @param  None
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 0 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 0 */

  if(LL_TIM_IsActiveFlag_UPDATE(TIM1) && LL_TIM_IsEnabledIT_UPDATE(TIM1))
  {
    LL_TIM_ClearFlag_UPDATE(TIM1);
    R3_1_TIMx_UP_IRQHandler(&PWM_Handle_M1);

    /* USER CODE BEGIN PWM_Update */

    /* USER CODE END PWM_Update */
  }
  else
  {
    /* Nothing to do */
  }

  if(LL_TIM_IsActiveFlag_BRK(TIM1) && LL_TIM_IsEnabledIT_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK(TIM1);
    PWMC_OCP_Handler(&PWM_Handle_M1._Super);

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */
  }
  else
  {
    /* Nothing to do */
  }

  if (LL_TIM_IsActiveFlag_BRK2(TIM1) && LL_TIM_IsEnabledIT_BRK(TIM1))
  {
    LL_TIM_ClearFlag_BRK2(TIM1);
    PWMC_OVP_Handler(&PWM_Handle_M1._Super, TIM1);

    /* USER CODE BEGIN Break */

    /* USER CODE END Break */
  }
  else
  {
   /* No other interrupts are routed to this handler */
  }

  /* USER CODE BEGIN TIMx_UP_BRK_M1_IRQn 1 */

  /* USER CODE END TIMx_UP_BRK_M1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
