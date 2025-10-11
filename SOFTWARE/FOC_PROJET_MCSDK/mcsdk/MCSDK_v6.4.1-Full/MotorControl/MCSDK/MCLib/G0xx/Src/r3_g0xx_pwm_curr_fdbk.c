/**
  ******************************************************************************
  * @file    r3_g0xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement current sensor
  *          class to be stantiated when the three shunts current sensing
  *          topology is used.
  * 
  *          It is specifically designed for STM32G0XX microcontrollers and
  *          implements the successive sampling of motor current using only one ADC.
  *           + MCU peripheral and handle initialization fucntion
  *           + three shunt current sensing
  *           + space vector modulation function
  *           + ADC sampling function
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
  * @ingroup R3_G0XX_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r3_g0xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_1_pwm_curr_fdbk
  * @{
  */

/* Private defines -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123       (LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N |\
                                   LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N)
/* Private function prototypes -----------------------------------------------*/
void R3_1_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl,ab_t* pStator_Currents);
void R3_1_HFCurrentsCalibrationC(PWMC_Handle_t *pHdl,ab_t* pStator_Currents);
uint16_t R3_1_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg);

/* Private functions ---------------------------------------------------------*/


/*
  * @brief  Initializes TIM1, ADC1, GPIO, DMA1 and NVIC for three shunt current
  *         reading configuration using STM32G0XX.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_1_Init(PWMC_R3_1_Handle_t *pHandle)
{
  
  if ((uint32_t)pHandle == (uint32_t)&pHandle->_Super)
  {

    /* Peripheral clocks enabling END ----------------------------------------*/

    /* Clear TIMx break flag. */
    LL_TIM_ClearFlag_BRK( TIM1 );
    LL_TIM_EnableIT_BRK( TIM1 );
    LL_TIM_SetCounter( TIM1, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
  
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_DBGMCU);
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);

    if ( LL_ADC_IsInternalRegulatorEnabled(ADC1) == 0u)
    {
      /* Enable ADC internal voltage regulator */
      LL_ADC_EnableInternalRegulator(ADC1);

      /* Wait for Regulator Startup time */
      /* Note: Variable divided by 2 to compensate partially              */
      /*       CPU processing cycles, scaling in us split to not          */
      /*       exceed 32 bits register capacity and handle low frequency. */
      volatile uint32_t wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US / 10UL) * (SystemCoreClock / (100000UL * 2UL)));
      while(wait_loop_index != 0UL)
      {
        wait_loop_index--;
      }
    }
    /* ADC Calibration */
    LL_ADC_StartCalibration(ADC1);
    while (LL_ADC_IsCalibrationOnGoing(ADC1))
    {
    }

    /* Enables the ADC peripheral */
    LL_ADC_Enable(ADC1);

    /* Wait ADC Ready */
    while ( LL_ADC_IsActiveFlag_ADRDY( ADC1 ) == RESET )
    {
      /* wait */
    }

    /* DMA1 Channel1 Config */
    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pHandle->ADC1_DMA_converted);
    LL_DMA_SetPeriphAddress( DMA1, LL_DMA_CHANNEL_1, ( uint32_t )&ADC1->DR );
    LL_DMA_SetDataLength( DMA1, LL_DMA_CHANNEL_1, 2u );
    
    /* Enable ADC DMA request*/
    LL_ADC_REG_SetDMATransfer( ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED );
    /* Enables the DMA1 Channel1 peripheral */
    LL_DMA_EnableChannel( DMA1, LL_DMA_CHANNEL_1 );
    /* Enable PWM channel */
    LL_TIM_CC_EnableChannel( TIM1, TIMxCCER_MASK_CH123 );

    /* set default triggering edge */
    pHandle->ADCTriggerEdge = LL_ADC_REG_TRIG_EXT_RISING; 
    
    /* Set the trigger polarity as computed inside SetADCSampPointSectX*/
    LL_ADC_REG_SetTriggerEdge (ADC1, pHandle->ADCTriggerEdge);
    /* set ADC trigger source */
    LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_TRGO2);
    /* Set scan direction according to the sector */  
    LL_ADC_REG_SetSequencerScanDirection(ADC1, pHandle->pParams_str->ADCScandir[pHandle->_Super.Sector]<<ADC_CFGR1_SCANDIR_Pos);
    /* Configure the ADC scheduler as selected inside SetADCSampPointSectX*/
    ADC1->CHSELR = pHandle->pParams_str->ADCConfig[pHandle->_Super.Sector];

    /* Clear EOC */
    LL_ADC_ClearFlag_EOC( ADC1 );
    /* ADC needs to be restarted because DMA is configured as limited */
    LL_ADC_REG_StartConversion( ADC1 );
    
    /* Clear Pending Interrupt Bits */
    LL_DMA_ClearFlag_HT1( DMA1 ); // TBC: for TC1, GL1 (not cleared ...)
  
    /* DMA Interrupt Event configuration */
    LL_DMA_EnableIT_TC( DMA1, LL_DMA_CHANNEL_1 );
  
    /* Clear Update Flag */
    LL_TIM_ClearFlag_UPDATE( TIM1 );
    /* Enable Update IRQ */
    LL_TIM_EnableIT_UPDATE( TIM1 );
    
  }
}

/*
  * @brief  Stores in @p pHdl handler the calibrated @p offsets.
  * 
  */
__weak void R3_1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  pHandle->PhaseAOffset = offsets->phaseAOffset;
  pHandle->PhaseBOffset = offsets->phaseBOffset;
  pHandle->PhaseCOffset = offsets->phaseCOffset;
  pHdl->offsetCalibStatus = true;
}

/*
  * @brief Reads the calibrated @p offsets stored in @p pHdl.
  * 
  */
__weak void R3_1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_1_Handle_t *pHandle = (PWMC_R3_1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  offsets->phaseAOffset = pHandle->PhaseAOffset;
  offsets->phaseBOffset = pHandle->PhaseBOffset;
  offsets->phaseCOffset = pHandle->PhaseCOffset;
}

/*
  * @brief  Stores into the @p pHdl handler the voltage present on Ia and
  *         Ib current feedback analog channels when no current is flowing into the
  *         motor.
  * 
  */
__weak void R3_1_CurrentReadingCalibration(PWMC_Handle_t *pHdl)
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef*  TIMx = TIM1;
  volatile PWMC_GetPhaseCurr_Cb_t GetPhaseCurrCbSave;
  volatile PWMC_SetSampPointSectX_Cb_t SetSampPointSectXCbSave;

  if (false == pHandle->_Super.offsetCalibStatus)
  {
    /* Save callback routines */
    GetPhaseCurrCbSave = pHandle->_Super.pFctGetPhaseCurrents;
    SetSampPointSectXCbSave = pHandle->_Super.pFctSetADCSampPointSectX;
    pHandle->PhaseAOffset = 0u;
    pHandle->PhaseBOffset = 0u;
    pHandle->PhaseCOffset = 0u;

    pHandle->PolarizationCounter = 0u;

    /* It forces inactive level on TIMx CHy and CHyN */
    LL_TIM_CC_DisableChannel( TIMx, TIMxCCER_MASK_CH123 );

    /* Offset calibration for A B c phases */
    /* Change function to be executed in ADCx_ISR */
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents     = &R3_1_HFCurrentsCalibrationAB;
    pHandle->_Super.pFctSetADCSampPointSectX = &R3_1_SetADCSampPointCalibration;
    pHandle->PolarizationSector = SECTOR_5;
    pHandle->_Super.Sector = SECTOR_5;
    __enable_irq();

    R3_1_SwitchOnPWM(&pHandle->_Super);

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd( TIMx,
                            &pHandle->_Super.SWerror,
                            pHandle->pParams_str->RepetitionCounter,
                            &pHandle->PolarizationCounter );

    R3_1_SwitchOffPWM(&pHandle->_Super);

    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = &R3_1_HFCurrentsCalibrationC;
    pHandle->PolarizationSector = SECTOR_1;
    pHandle->_Super.Sector = SECTOR_1;
    pHandle->PolarizationCounter = 0;
    __enable_irq();

    R3_1_SwitchOnPWM(&pHandle->_Super);

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd( TIMx,
                            &pHandle->_Super.SWerror,
                            pHandle->pParams_str->RepetitionCounter,
                            &pHandle->PolarizationCounter );

    R3_1_SwitchOffPWM(&pHandle->_Super);
    pHandle->_Super.Sector = SECTOR_5;
    pHandle->PhaseAOffset = pHandle->PhaseAOffset / NB_CONVERSIONS;
    pHandle->PhaseBOffset = pHandle->PhaseBOffset / NB_CONVERSIONS;
    pHandle->PhaseCOffset = pHandle->PhaseCOffset / NB_CONVERSIONS;
    if (0U == pHandle->_Super.SWerror)
    {
      pHandle->_Super.offsetCalibStatus = true;
    }
    else
    {
      /* nothing to do */
    }

    /* restore function to be executed in ADCx_ISR */
    __disable_irq();
    pHandle->_Super.pFctGetPhaseCurrents = GetPhaseCurrCbSave;
    pHandle->_Super.pFctSetADCSampPointSectX = SetSampPointSectXCbSave;
    __enable_irq();
  }
	
  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverer legs */
  LL_TIM_OC_SetCompareCH1 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH2 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH3 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  pHandle->_Super.BrakeActionLock = false;

}

/*
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p pStator_Currents ab_t format.
  *
  */
__weak void R3_1_GetPhaseCurrents(PWMC_Handle_t *pHdl,ab_t* pStator_Currents)
{ 
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  int32_t wAux;
  uint16_t hReg1;
  uint16_t hReg2;
  uint8_t bSector;

  /* disable ADC trigger source */
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);

  bSector = ( uint8_t ) pHandle->_Super.Sector;

  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  hReg1 = *pHandle->pParams_str->ADCDataReg1[bSector];
  hReg2 = *pHandle->pParams_str->ADCDataReg2[bSector];
  
  switch (bSector)
  {
  case SECTOR_4:
  case SECTOR_5: 
    /* Current on Phase C is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) ------------------------------*/
        
    wAux = (int32_t)(pHandle->PhaseAOffset)-(int32_t)(hReg1);
    
    /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
    {
        pStator_Currents->a = -INT16_MAX;
    }  
      else  if ( wAux > INT16_MAX )
    { 
        pStator_Currents->a = INT16_MAX;
    }
    else
    {
      pStator_Currents->a= (int16_t)wAux;
    }
    
    /* Ib = PhaseBOffset - ADC converted value) ------------------------------*/
    
    wAux = (int32_t)(pHandle->PhaseBOffset)-(int32_t)(hReg2);
      
    /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
    {
        pStator_Currents->b = -INT16_MAX;
    }  
      else  if ( wAux > INT16_MAX )
    { 
        pStator_Currents->b = INT16_MAX;
    }
    else
    {
      pStator_Currents->b= (int16_t)wAux;
    }
    break;
 
  case SECTOR_6:
  case SECTOR_1:  
    /* Current on Phase A is not accessible     */
    /* Ib = (PhaseBOffset - ADC converted value) ------------------------------*/ 
    wAux = (int32_t)(pHandle->PhaseBOffset)-(int32_t)(hReg1);
    
   /* Saturation of Ib */
      if ( wAux < -INT16_MAX )
    {
        pStator_Currents->b = -INT16_MAX;
    }  
      else  if ( wAux > INT16_MAX )
    { 
        pStator_Currents->b = INT16_MAX;
    }
    else
    {
      pStator_Currents->b= (int16_t)wAux;
    }
    
    wAux = (int32_t)(pHandle->PhaseCOffset)-(int32_t)(hReg2);
    /* Ia = -Ic -Ib ----------------------------------------------------------*/
    wAux =-wAux - (int32_t)pStator_Currents->b;           /* Ia  */

    /* Saturation of Ia */
      if ( wAux > INT16_MAX )
    {
        pStator_Currents->a = INT16_MAX;
    }
      else  if ( wAux < -INT16_MAX )
    {
        pStator_Currents->a = -INT16_MAX;
    }
    else
    {  
      pStator_Currents->a = (int16_t)wAux;
    }
    break;
 
  case SECTOR_2:
  case SECTOR_3:
   /* Current on Phase B is not accessible     */
    /* Ia = PhaseAOffset - ADC converted value) ------------------------------*/    
    wAux = (int32_t)(pHandle->PhaseAOffset)-(int32_t)(hReg1);
    
   /* Saturation of Ia */
      if ( wAux < -INT16_MAX )
    {
        pStator_Currents->a = -INT16_MAX;
    }  
      else  if ( wAux > INT16_MAX )
    { 
        pStator_Currents->a = INT16_MAX;
    }
    else
    {
      pStator_Currents->a= (int16_t)wAux;
    }

    /* Ic = PhaseCOffset - ADC converted value) ------------------------------*/    
    wAux = (int32_t)(pHandle->PhaseCOffset)-(int32_t)(hReg2);

    /* Ib = -Ic -Ia */
    wAux = -wAux -  (int32_t)pStator_Currents->a;           /* Ib  */

    /* Saturation of Ib */
      if ( wAux > INT16_MAX )
    {
        pStator_Currents->b = INT16_MAX;
    }
      else  if ( wAux < -INT16_MAX )
    {  
        pStator_Currents->b = -INT16_MAX;
    }
    else  
    {
      pStator_Currents->b = (int16_t)wAux;
    }                     
    break;
    
  default:
    break;
  }  

  pHandle->_Super.Ia = pStator_Currents->a;
  pHandle->_Super.Ib = pStator_Currents->b;
  pHandle->_Super.Ic = -pStator_Currents->a - pStator_Currents->b;
}

/*
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p Iab ab_t format. Specific to overmodulation.
  *
  */
__weak void R3_1_GetPhaseCurrents_OVM( PWMC_Handle_t * pHdl, ab_t * Iab )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

  uint8_t Sector;
  int32_t Aux;
  uint32_t ADCDataReg1;
  uint32_t ADCDataReg2;

  /* disable ADC trigger source */
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
  
  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
  Sector = ( uint8_t )pHandle->_Super.Sector;
  ADCDataReg1 = *pHandle->pParams_str->ADCDataReg1[Sector];
  ADCDataReg2 = *pHandle->pParams_str->ADCDataReg2[Sector];

switch ( Sector )
  {
    case SECTOR_4:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      if (pHandle->_Super.useEstCurrent == true)
      {
        // Ib not available, use estimated Ib
        Aux = ( int32_t )( pHandle->_Super.IbEst );
      }
      else
      {
        /* Ib = PhaseBOffset - ADC converted value) */
        Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg2 );
      }

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;
      
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      if (pHandle->_Super.useEstCurrent == true)
      {
        // Ia not available, use estimated Ia
        Aux = ( int32_t )( pHandle->_Super.IaEst );
      }
      else
      {
        Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );
      }

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg2 );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    case SECTOR_6:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }

      if (pHandle->_Super.useEstCurrent == true)
      {
        Aux =  ( int32_t ) pHandle->_Super.IcEst ; /* -Ic */
        Aux -= ( int32_t )Iab->b; 
      }
      else
      {
      /* Ia = -Ic -Ib */
        Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
        Aux -= ( int32_t )Iab->b;             /* Ia  */
      }
      /* Saturation of Ia */
      if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }
      break;
      
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      if (pHandle->_Super.useEstCurrent == true)
      {
        Aux = ( int32_t ) pHandle->_Super.IbEst;
      }
      else
      {
        Aux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( ADCDataReg1 );
      }
      /* Saturation of Ib */
      if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }

      /* Ia = -Ic -Ib */
      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux -= ( int32_t )Iab->b;             /* Ia  */

      /* Saturation of Ia */
      if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }
      break;

    case SECTOR_2:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      if (pHandle->_Super.useEstCurrent == true)
      {
        Aux = ( int32_t ) pHandle->_Super.IaEst;
      }
      else
      {
        Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );
      }
      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      /* Ib = -Ic -Ia */
      Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
      Aux -= ( int32_t )Iab->a;             /* Ib */

      /* Saturation of Ib */
      if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      Aux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( ADCDataReg1 );

      /* Saturation of Ia */
      if ( Aux < -INT16_MAX )
      {
        Iab->a = -INT16_MAX;
      }
      else  if ( Aux > INT16_MAX )
      {
        Iab->a = INT16_MAX;
      }
      else
      {
        Iab->a = ( int16_t )Aux;
      }

      if (pHandle->_Super.useEstCurrent == true)
      {
        /* Ib = -Ic -Ia */
        Aux = ( int32_t ) pHandle->_Super.IcEst; /* -Ic */
        Aux -= ( int32_t )Iab->a;             /* Ib */
      }
      else
      {
        /* Ib = -Ic -Ia */
        Aux = ( int32_t )( ADCDataReg2 ) - ( int32_t )( pHandle->PhaseCOffset ); /* -Ic */
        Aux -= ( int32_t )Iab->a;             /* Ib */
      }
      
      /* Saturation of Ib */
      if ( Aux > INT16_MAX )
      {
        Iab->b = INT16_MAX;
      }
      else  if ( Aux < -INT16_MAX )
      {
        Iab->b = -INT16_MAX;
      }
      else
      {
        Iab->b = ( int16_t )Aux;
      }
      break;

    default:
      break;
  }

    pHandle->_Super.Ia = Iab->a;
    pHandle->_Super.Ib = Iab->b;
    pHandle->_Super.Ic = -Iab->a - Iab->b;
}

/*
 * @brief  Configures the ADC for the current sampling during calibration.
 * 
 * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
 * It then calls the WriteTIMRegisters method.
 * 
 * @param pHdl: Handler of the current instance of the PWM component.
 * @retval Return value of R3_1_WriteTIMRegisters.
 */
__weak uint16_t R3_1_SetADCSampPointCalibration( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif

  pHandle->ADCTriggerEdge = LL_ADC_REG_TRIG_EXT_RISING;
  pHandle->_Super.Sector = pHandle->PolarizationSector;

  return R3_1_WriteTIMRegisters( pHdl,  ( uint16_t )( pHandle->Half_PWMPeriod ) - 1u);
}

/*
  * @brief  Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  * 
  * Sets the ADC sequence length and channels and sets the sampling point via TIMx_Ch4 value and polarity.
  * The WriteTIMRegisters method is then called.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns the value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_1_SetADCSampPointSectX( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif

  uint16_t SamplingPoint;
  uint16_t DeltaDuty;
  register uint16_t lowDuty = pHdl->lowDuty;
  register uint16_t midDuty = pHdl->midDuty;
  
  /* Check if sampling the AB phases in the middle of the PWM period is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - lowDuty ) > pHandle->pParams_str->Tafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are
     * differences between offsets */

    /* Sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds to 
     * sector 4 or 5 */
    pHandle->_Super.Sector = SECTOR_5;

    /* Set the sampling point in the middle of the PWM period */
    SamplingPoint = pHandle->Half_PWMPeriod - 1u;
  }
  else
  {
    /* In this case it is necessary to sample phases with maximum and variable complementary duty cycle */

    /* In every sector there is always one phase with maximum complementary duty, one with minimum 
     * complementary duty and one with variable complementary duty. In this case, phases with variable 
     * and maximum complementary duty are converted, always starting with the one with variable 
     * complementary duty cycle */

    /* Crossing Point Searching */
    DeltaDuty = ( uint16_t )( lowDuty - midDuty );

    /* Definition of crossing point */
    if ( DeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - lowDuty ) * 2u )
    {
      SamplingPoint = lowDuty - pHandle->pParams_str->Tbefore;
    }
    else
    {
      SamplingPoint = lowDuty + pHandle->pParams_str->Tafter;

      if ( SamplingPoint >= pHandle->Half_PWMPeriod )
      {
        pHandle->ADCTriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;
        SamplingPoint = ( 2u * pHandle->Half_PWMPeriod ) - SamplingPoint - 1u;
      }
    }
  }
  return R3_1_WriteTIMRegisters( &pHandle->_Super, SamplingPoint );
}

/*
  * @brief  Configures the ADC for the current sampling related to sector X (X = [1..6] ) in case of overmodulation.
  * 
  * Sets the ADC sequence length and channels and sets the sampling point via TIMx_Ch4 value and polarity.
  * The WriteTIMRegisters method is then called.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval uint16_t Returns the value of R3_1_WriteTIMRegisters.
  */
uint16_t R3_1_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */

  uint16_t SamplingPoint;
  uint16_t DeltaDuty;

  pHandle->_Super.useEstCurrent = false;
  DeltaDuty = ( uint16_t )( pHdl->lowDuty - pHdl->midDuty );

  /* Check if sampling the AB phases in the middle of the PWM period is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - pHdl->lowDuty ) > pHandle->pParams_str->Tafter )
  {
	  /* When it is possible to sample in the middle of the PWM period, always sample the same phases
	   * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
	   * between offsets */

	  /* Sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
	   * to sector 4 or 5  */
    pHandle->_Super.Sector = SECTOR_5;

    /* Set the sampling point in the middle of the PWM period */
    SamplingPoint =  pHandle->Half_PWMPeriod - 1u;
  }
  else
  {
    /* In this case it is necessary to sample phases with maximum and variable complementary duty cycle.*/

    /* In every sector there is always one phase with maximum complementary duty, one with minimum 
     * complementary duty and one with variable complementary duty. In this case, phases with variable
     * complementary duty and with maximum duty are converted and the first will be always the phase
     * with variable complementary duty cycle */

    if ( DeltaDuty >= pHandle->pParams_str->Tcase3 )
    {
      SamplingPoint = pHdl->lowDuty - pHandle->pParams_str->Tbefore;
    }
    else
    {
      /* case 2 (cf user manual) */
      if ((pHandle->Half_PWMPeriod - pHdl->lowDuty) > pHandle->pParams_str->Tcase2)
      {
        /* ADC trigger edge must be changed from positive to negative */
        pHandle->ADCTriggerEdge = LL_ADC_REG_TRIG_EXT_FALLING;
        SamplingPoint = pHdl->lowDuty + pHandle->pParams_str->Tbefore;
      }
      else
      {
        /* No suitable sampling window has been found, sampling is executed in the middle of the PWM
         * Period. Sampled currents will be disregarded and estimated currents will be used instead */
        SamplingPoint = pHandle->Half_PWMPeriod - 1u;
        pHandle->_Super.useEstCurrent = true;
      }
    }
  }
  return R3_1_WriteTIMRegisters( &pHandle->_Super, SamplingPoint );
}

/*
  * @brief  Writes into peripheral registers the new duty cycles and sampling point.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  hCCR4Reg: New capture/compare register value, written in timer clock counts.
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__weak uint16_t R3_1_WriteTIMRegisters(PWMC_Handle_t *pHdl, uint16_t hCCR4Reg)
{ 
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef*  TIMx = TIM1;
  uint16_t hAux;

  LL_TIM_OC_SetCompareCH1 ( TIMx, (uint32_t)pHandle->_Super.CntPhA );
  LL_TIM_OC_SetCompareCH2 ( TIMx, (uint32_t)pHandle->_Super.CntPhB );
  LL_TIM_OC_SetCompareCH3 ( TIMx, (uint32_t)pHandle->_Super.CntPhC );
  LL_TIM_OC_SetCompareCH4 ( TIMx, (uint32_t)hCCR4Reg );
  
 /* Re-configuration of CCR4 must be done before the timer update to be taken
    into account at the next PWM cycle. Otherwise we are too late, we flag a
    FOC_DURATION error */
  if (((TIMx->CR2) & TIM_CR2_MMS2_Msk) != LL_TIM_TRGO2_RESET )
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  
  if (pHandle->_Super.SWerror == 1u)
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  
  return hAux;
}

/*
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during calibration.
  *         
  * It sums up injected conversion data into PhaseAOffset and wPhaseBOffset
  * to compute the offset introduced in the current feedback network. It is required to 
  * properly configure ADC inputs before in order to enable offset computation. Called R3_1_HFCurrentsPolarizationAB in F30X and G4XX.
  * 
  * @param  pHdl: Pointer on the target component instance.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
__weak void R3_1_HFCurrentsCalibrationAB(PWMC_Handle_t *pHdl, ab_t* pStator_Currents)
{ 
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = TIM1;
  uint8_t bSector = pHandle->PolarizationSector;

  /* disable ADC trigger source */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_RESET);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseAOffset += *pHandle->pParams_str->ADCDataReg1[bSector];
    pHandle->PhaseBOffset += *pHandle->pParams_str->ADCDataReg2[bSector];
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;
}

/*
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during calibration.
  *         
  * It sums up injected conversion data into PhaseCOffset to compute the offset
  * introduced in the current feedback network. It is required to properly configure 
  * ADC inputs before in order to enable offset computation.
  * 
  * @param  pHdl: Pointer on the target component instance.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
void R3_1_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef * TIMx = TIM1;
  uint8_t bSector = pHandle->PolarizationSector;

  /* disable ADC trigger source */
  LL_TIM_SetTriggerOutput2(TIMx, LL_TIM_TRGO2_RESET);
  
  pHandle->_Super.Sector = SECTOR_1;
  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseCOffset += *pHandle->pParams_str->ADCDataReg2[bSector];
    pHandle->PolarizationCounter++;
  }

  /* during offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;
}

/*
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors of driving section. It has to be
  * called each motor start-up when using high voltage drivers.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component
  * @param  ticks: Timer ticks value to be applied
  *                Min value: 0 (low sides ON)
  *                Max value: PWM_PERIOD_CYCLES/2 (low sides OFF)
  */
__weak void R3_1_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks)
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef*  TIMx = TIM1;
  
  pHandle->_Super.TurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  
  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx, ticks);
  LL_TIM_OC_SetCompareCH2(TIMx, ticks);
  LL_TIM_OC_SetCompareCH3(TIMx, ticks);
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->_Super.LowSideOutputs)== ES_GPIO)
  {
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );

  }
  return;   
}

/*
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_1_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef* TIMx = TIM1;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Disable TIMx preload */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod - 5u));
  /* Apply new CC values */
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIMx, LL_TIM_CHANNEL_CH4);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE; 
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->_Super.LowSideOutputs)== ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );

    }
  }
  pHandle->_Super.PWMState = true;
}
 
/*
  * @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_1_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif
  PWMC_R3_1_Handle_t * pHandle = ( PWMC_R3_1_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif
  TIM_TypeDef* TIMx = TIM1;

  pHandle->_Super.PWMState = false;
  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if ( pHandle->_Super.BrakeActionLock == true )
  {
  }
  else
  {
    if ((pHandle->_Super.LowSideOutputs)== ES_GPIO)
    {
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
}

/*
  * @brief  Contains the TIMx Update event interrupt.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
void * R3_1_TIMx_UP_IRQHandler( PWMC_R3_1_Handle_t * pHandle )
{
  
  /* Set the trigger polarity as computed inside SetADCSampPointSectX*/
  LL_ADC_REG_SetTriggerEdge (ADC1, pHandle->ADCTriggerEdge);
  /* Clear Configuration Ready Flag before modifying ADC configuration */
  LL_ADC_ClearFlag_CCRDY(ADC1);
  /* Configure the ADC scheduler as selected inside SetADCSampPointSectX*/
  ADC1->CHSELR = pHandle->pParams_str->ADCConfig[pHandle->_Super.Sector];
  /* Poll for ADC channel configuration ready */
  while (LL_ADC_IsActiveFlag_CCRDY(ADC1) == 0)
     {

     }
  /* Clear Configuration Ready Flag before modifying ADC configuration again */
  LL_ADC_ClearFlag_CCRDY(ADC1);
  /* Set scan direction according to the sector. Poll for CCRDY cannot be done here as the flag is set only if SCANDIR is modified, which is not always the case */  
  LL_ADC_REG_SetSequencerScanDirection(ADC1, pHandle->pParams_str->ADCScandir[pHandle->_Super.Sector]<<ADC_CFGR1_SCANDIR_Pos);
  /* Re-enable ADC trigger */
  LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_OC4);
  /* ADC needs to be restarted because DMA is configured as limited */
  LL_ADC_REG_StartConversion( ADC1 );

  /* Reset the ADC trigger edge for next conversion */
  pHandle->ADCTriggerEdge = LL_ADC_REG_TRIG_EXT_RISING;

  return &pHandle->_Super.Motor;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
