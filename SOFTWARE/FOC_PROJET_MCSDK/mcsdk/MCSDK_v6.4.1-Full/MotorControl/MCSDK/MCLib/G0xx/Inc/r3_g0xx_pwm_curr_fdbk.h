/**
  ******************************************************************************
  * @file    r3_g0xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_G0XX_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup R3_G0XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_G0XX_PWMNCURRFDBK_H
#define __R3_G0XX_PWMNCURRFDBK_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_1_pwm_curr_fdbk
  * @{
  */

/* Exported constants --------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

/*
  * @brief  PWM and current feedback component parameters definition for single ADC configurations.
  */
typedef struct
{
  /* HW IP involved ------------------------------------------------------------- */
  TIM_TypeDef * TIMx;                       /* Timer used for PWM generation */
  
  /* Currents sampling parameters ----------------------------------------------- */
  uint16_t Tafter;                          /* Sum of dead time plus max value between rise time and noise time expressed in number of TIM clocks */
  uint16_t Tbefore;                         /* Total time of the sampling sequence expressed in number of TIM clocks */
  uint16_t Tcase2;                          /* Sum of dead time, noise time and sampling time divided by 2 ; expressed in number of TIM clocks */
  uint16_t Tcase3;                          /* Sum of dead time, rise time and sampling time ; expressed in number of TIM clocks */
  uint32_t ADCConfig[6];                    /* Stores ADC sequence for the 6 sectors */
  volatile uint16_t *ADCDataReg1[6];        /* Stores ADC read value's address for the 6 sectors */
  volatile uint16_t *ADCDataReg2[6];        /* Stores ADC read value's address for the 6 sectors */
  uint8_t  ADCScandir[6];                   /* Stores ADC scan direction for the 6 sectors */

  /* PWM Driving signals initialization ----------------------------------------- */
  uint8_t  RepetitionCounter;               /* Number of elapsed PWM periods before Compare Registers are updated again.
                                             * In particular : RepetitionCounter = (2 * PWM periods) - 1 */

}R3_1_Params_t;

/*
  * @brief  PWM and current feedback component for single ADC configurations.
  */
typedef struct
{
  PWMC_Handle_t _Super;                     /* Base component handler */
  uint32_t PhaseAOffset;                    /* Offset of Phase A current sensing network */
  uint32_t PhaseBOffset;                    /* Offset of Phase B current sensing network */
  uint32_t PhaseCOffset;                    /* Offset of Phase C current sensing network */
  uint16_t Half_PWMPeriod;                  /* Half PWM Period in timer clock counts */
  volatile uint16_t ADC1_DMA_converted[2];  /* Buffer used for DMA data transfer after the ADC conversion */
  volatile uint32_t ADCTriggerEdge;         /* Polarity of the ADC triggering, can be either on rising or falling edge */
  volatile uint8_t PolarizationCounter;     /* Number of conversions performed during the calibration phase */
  uint8_t PolarizationSector;               /* Sector selected during calibration phase */

  R3_1_Params_t const * pParams_str;
}PWMC_R3_1_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIM1, ADC1, GPIO, DMA1 and NVIC for three shunt current
  * reading configuration using STM32G0x.
  */
void R3_1_Init(PWMC_R3_1_Handle_t *pHandle);

/*
  * Stores into the handler the voltage present on the
  * current feedback analog channel when no current is flowing into the
  * motor.
  */
void R3_1_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void R3_1_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format. Specific to overmodulation.
  */
void R3_1_GetPhaseCurrents_OVM( PWMC_Handle_t * pHdl, ab_t * Iab );

/*
  * Configures the ADC for the currents sampling related to sector X (X = [1..6] ).
  */
uint16_t R3_1_SetADCSampPointSectX(PWMC_Handle_t * pHdl );

/*
  * Configure the ADC for the currents sampling related to sector X (X = [1..6] ) in case of overmodulation.
  */
uint16_t R3_1_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHdl );

/*
  * Configures the ADC for the currents sampling during calibration.
  */
uint16_t R3_1_SetADCSampPointCalibration( PWMC_Handle_t * pHdl );

/*
  * Turns on low side switches.
  */
void R3_1_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_1_SwitchOnPWM( PWMC_Handle_t * pHdl );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit and resets the TIM status.
  */
void R3_1_SwitchOffPWM( PWMC_Handle_t * pHdl );

/*
  * Executes a regular conversion.
  */
uint16_t R3_1_ExecRegularConv(PWMC_Handle_t *pHdl, uint8_t bChannel);

/*
  * Contains the TIMx Update event interrupt.
  */
void * R3_1_TIMx_UP_IRQHandler( PWMC_R3_1_Handle_t * pHandle );

/*
  * Sets the calibrated offset.
  */
void R3_1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets.
  */
void R3_1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /*__R3_G0XX_PWMNCURRFDBK_H*/

 /************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
