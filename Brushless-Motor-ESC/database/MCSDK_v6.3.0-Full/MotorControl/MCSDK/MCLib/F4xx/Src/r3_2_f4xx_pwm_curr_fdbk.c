/**
  ******************************************************************************
  * @file    r3_2_f4xx_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the three shunts current sensing topology.
  * 
  *          It is specifically designed for STM32F4XX microcontrollers and
  *          implements the successive sampling of current using two ADCs.
  *           + MCU peripheral and handle initialization function
  *           + three shunt current sensing
  *           + space vector modulation function
  *           + ADC sampling function
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup R3_2_F4XX_pwm_curr_fdbk
  */

/* Includes ------------------------------------------------------------------*/
#include "r3_2_f4xx_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_2_pwm_curr_fdbk
  * @{
  */

/* Private defines -----------------------------------------------------------*/

#define TIMxCCER_MASK_CH123        ((uint16_t)  (LL_TIM_CHANNEL_CH1|LL_TIM_CHANNEL_CH1N|\
                                                 LL_TIM_CHANNEL_CH2|LL_TIM_CHANNEL_CH2N|\
                                                 LL_TIM_CHANNEL_CH3|LL_TIM_CHANNEL_CH3N))

/* Private function prototypes -----------------------------------------------*/
__STATIC_INLINE uint16_t R3_2_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg );
void R3_2_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
void R3_2_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
static void R3_2_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );
static void R3_2_RLTurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );
static void R3_2_RLSwitchOnPWM( PWMC_Handle_t * pHdl );

/*
  * @brief  Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  *         in three shunt topology using two ADCs.
  * 
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void R3_2_Init( PWMC_R3_2_Handle_t * pHandle )
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCDataReg1[0];
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCDataReg2[0];

  LL_TIM_DisableCounter( TIMx );

  LL_TIM_ClearFlag_BRK(TIMx);
  LL_TIM_EnableIT_BRK(TIMx);

  /* Prepare timer for synchronization */
  LL_TIM_GenerateEvent_UPDATE(TIMx);
  if ( pHandle->pParams_str->bFreqRatio == 2u )
  {
    if ( pHandle->pParams_str->bIsHigherFreqTim == HIGHER_FREQ )
    {
      if ( pHandle->pParams_str->RepetitionCounter == 3u )
      {
        /* Set TIMx repetition counter to 1 */
        LL_TIM_SetRepetitionCounter( TIMx, 1u );
        LL_TIM_GenerateEvent_UPDATE( TIMx );
        /* Repetition counter will be set to 3 at next Update */
        LL_TIM_SetRepetitionCounter( TIMx, 3 );
      }
    }

    LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
  }
  else /* FreqRatio equal to 1 or 3 */
  {
    if ( pHandle->_Super.Motor == M1 )
    {
      LL_TIM_SetCounter( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u );
    }
  }

  /* Enable PWM channel */
  LL_TIM_CC_EnableChannel( TIMx, TIMxCCER_MASK_CH123 );

  if ( TIMx == TIM1 )
  {
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM1_STOP);
    pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM1_CH4;
  }
#if defined(TIM8)
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_TIM8_STOP);
    pHandle->ADC_ExternalTriggerInjected = LL_ADC_INJ_TRIG_EXT_TIM8_CH4;
  }
#endif

  /* ADCs registers configuration ---------------------------------*/
  /* Enable ADCx_1 and ADCx_2 */
  LL_ADC_Enable( ADCx_1 );
  LL_ADC_Enable( ADCx_2 );

  /* ADCx_1 Injected conversions end interrupt enabling */
  LL_ADC_ClearFlag_JEOS(ADCx_1);
  LL_ADC_EnableIT_JEOS( ADCx_1 );

  /* reset regular conversion sequencer length set by cubeMX */
  LL_ADC_REG_SetSequencerLength( ADCx_1, LL_ADC_REG_SEQ_SCAN_DISABLE );
  LL_ADC_INJ_SetSequencerLength( ADCx_1, LL_ADC_INJ_SEQ_SCAN_DISABLE );
  LL_ADC_INJ_SetSequencerLength( ADCx_2, LL_ADC_INJ_SEQ_SCAN_DISABLE );

  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;


}

/*
  * @brief  Stores in @p pHdl handler the calibrated @p offsets.
  * 
  */
__weak void R3_2_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  pHandle->PhaseAOffset = offsets->phaseAOffset;
  pHandle->PhaseBOffset = offsets->phaseBOffset;
  pHandle->PhaseCOffset = offsets->phaseCOffset;
  pHdl->offsetCalibStatus = true;
}

/*
  * @brief Reads the calibrated @p offsets stored in @p pHdl handler.
  * 
  */
__weak void R3_2_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R3_2_Handle_t *pHandle = (PWMC_R3_2_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  offsets->phaseAOffset = pHandle->PhaseAOffset;
  offsets->phaseBOffset = pHandle->PhaseBOffset;
  offsets->phaseCOffset = pHandle->PhaseCOffset;
}

/*
  * @brief  Stores into the @p pHdl the voltage present on Ia and Ib current 
  *         feedback analog channels when no current is flowing into the motor.
  * 
  */
__weak void R3_2_CurrentReadingCalibration( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef*  TIMx = pHandle->pParams_str->TIMx;

  if (false == pHandle->_Super.offsetCalibStatus)
  {
    volatile PWMC_GetPhaseCurr_Cb_t GetPhaseCurrCbSave;
    volatile PWMC_SetSampPointSectX_Cb_t SetSampPointSectXCbSave;

    /* Save callback routines */
    GetPhaseCurrCbSave = pHandle->_Super.pFctGetPhaseCurrents;
    SetSampPointSectXCbSave = pHandle->_Super.pFctSetADCSampPointSectX;

    pHandle->PhaseAOffset = 0u;
    pHandle->PhaseBOffset = 0u;
    pHandle->PhaseCOffset = 0u;

    pHandle->PolarizationCounter = 0u;

    LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);

    /* Offset calibration for A & B phases */
    /* Change function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_HFCurrentsCalibrationAB;
    pHandle->_Super.pFctSetADCSampPointSectX = &R3_2_SetADCSampPointCalibration;

    pHandle->CalibSector = SECTOR_4;
    /* Required to force first polarization conversion on SECTOR_4*/
    pHandle->_Super.Sector = SECTOR_4;

    R3_2_SwitchOnPWM( &pHandle->_Super );

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd( TIMx,
                            &pHandle->_Super.SWerror,
                            pHandle->pParams_str->RepetitionCounter,
                            &pHandle->PolarizationCounter );

    R3_2_SwitchOffPWM( &pHandle->_Super );
    /* Offset calibration for C phase */
    /* Reset PolarizationCounter */
    pHandle->PolarizationCounter = 0u;

    /* Change function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_HFCurrentsCalibrationC;

    pHandle->CalibSector = SECTOR_1;
    /* Required to force first polarization conversion on SECTOR_1*/
    pHandle->_Super.Sector = SECTOR_1;
    R3_2_SwitchOnPWM( &pHandle->_Super );

    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd( TIMx,
                            &pHandle->_Super.SWerror,
                            pHandle->pParams_str->RepetitionCounter,
                            &pHandle->PolarizationCounter );

    R3_2_SwitchOffPWM( &pHandle->_Super );

    pHandle->PhaseAOffset >>= 3;
    pHandle->PhaseBOffset >>= 3;
    pHandle->PhaseCOffset >>= 3;
    if (0U == pHandle->_Super.SWerror)
    {
      pHandle->_Super.offsetCalibStatus = true;
    }
    else
    {
      /* nothing to do */
    }

    /* Change back function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = GetPhaseCurrCbSave;
    pHandle->_Super.pFctSetADCSampPointSectX = SetSampPointSectXCbSave;
  }
  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
     force 50% duty cycle on the three inverter legs */
  /* Disable TIMx preload */
  LL_TIM_OC_SetCompareCH1 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH2 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH3 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  /* generate  COM event to apply new CC values */
  LL_TIM_GenerateEvent_COM( TIMx );

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

  /* sector and phase sequence for the switch on phase */
  pHandle->_Super.Sector = SECTOR_4;

}

/*
  * @brief  Computes and stores in @p pHdl handler the latest converted motor phase currents in @p pStator_Currents ab_t format.
  *
  */
__weak void R3_2_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t* pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  int32_t wAux;
  uint16_t hReg1;
  uint16_t hReg2;
  uint8_t bSector;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  bSector = pHandle->_Super.Sector;
  hReg1 = pHandle->pParams_str->ADCDataReg1[bSector]->JDR1 * 2;
  hReg2 = pHandle->pParams_str->ADCDataReg2[bSector]->JDR1 * 2;

  switch ( bSector )
  {
    case SECTOR_4:
    case SECTOR_5:
      /* Current on Phase C is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( hReg1 );
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
        pStator_Currents->a = ( int16_t )wAux;
      }

      /* Ib = PhaseBOffset - ADC converted value) */
        wAux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( hReg2 );

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
        pStator_Currents->b = ( int16_t )wAux;
      }
      break;

    case SECTOR_6:
    case SECTOR_1:
      /* Current on Phase A is not accessible     */
      /* Ib = PhaseBOffset - ADC converted value) */
      wAux = ( int32_t )( pHandle->PhaseBOffset ) - ( int32_t )( hReg1 );
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
        pStator_Currents->b = ( int16_t )wAux;
      }

      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ia = -Ic -Ib */
      wAux = ( int32_t )( pHandle->PhaseCOffset ) - ( int32_t )( hReg2 );
      wAux = -wAux - ( int32_t )pStator_Currents->b;

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
        pStator_Currents->a = ( int16_t )wAux;
      }
      break;

    case SECTOR_2:
    case SECTOR_3:
      /* Current on Phase B is not accessible     */
      /* Ia = PhaseAOffset - ADC converted value) */
      wAux = ( int32_t )( pHandle->PhaseAOffset ) - ( int32_t )( hReg1 );
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
        pStator_Currents->a = ( int16_t )wAux;
      }

      /* Ic = PhaseCOffset - ADC converted value) */
      /* Ib = -Ic -Ia */
      wAux = ( int32_t )( pHandle->PhaseCOffset ) - ( int32_t )( hReg2 );
      wAux = -wAux -  ( int32_t )pStator_Currents->a;

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
        pStator_Currents->b = ( int16_t )wAux;
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
__weak void R3_2_GetPhaseCurrents_OVM( PWMC_Handle_t * pHdl, ab_t * Iab )
{
#if defined (__ICCARM__)
  #pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
#if defined (__ICCARM__)
  #pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  uint8_t Sector;
  int32_t Aux;
  uint32_t ADCDataReg1;
  uint32_t ADCDataReg2;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  Sector = ( uint8_t )pHandle->_Super.Sector;
  ADCDataReg1 = pHandle->pParams_str->ADCDataReg1[Sector]->JDR1 * 2;
  ADCDataReg2 = pHandle->pParams_str->ADCDataReg2[Sector]->JDR1 * 2;

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
  * @brief  Implementation of PWMC_GetPhaseCurrents to be performed during calibration.
  *         
  * It sums up injected conversion data into PhaseAOffset and wPhaseBOffset
  * to compute the offset introduced in the current feedback network. It is required to 
  * properly configure ADC inputs before in order to enable offset computation.
  * 
  * @param  pHdl: Pointer on the target component instance.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
__weak void R3_2_HFCurrentsCalibrationAB( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseAOffset += pHandle->pParams_str->ADCDataReg1[pHandle->CalibSector]->JDR1;
    pHandle->PhaseBOffset += pHandle->pParams_str->ADCDataReg2[pHandle->CalibSector]->JDR1;
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
__weak void R3_2_HFCurrentsCalibrationC( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;  

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  if ( pHandle->PolarizationCounter < NB_CONVERSIONS )
  {
    pHandle->PhaseCOffset += pHandle->pParams_str->ADCDataReg2[pHandle->CalibSector]->JDR1;
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
__weak void R3_2_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /*Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx,ticks);
  LL_TIM_OC_SetCompareCH2(TIMx,ticks);
  LL_TIM_OC_SetCompareCH3(TIMx,ticks);

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET)
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);
  if ( (pHandle->_Super.LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin (pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
    LL_GPIO_SetOutputPin (pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
    LL_GPIO_SetOutputPin (pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
  }
  return;
}

/*
  * @brief  Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_2_SwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod  >> 1));
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod - 5u));

  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    if ( LL_TIM_CC_IsEnabledChannel(TIMx,TIMxCCER_MASK_CH123) != 0u )
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

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  /* Enable Update IRQ */
  LL_TIM_EnableIT_UPDATE(TIMx);

  return;
}

/*
  * @brief  Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
__weak void R3_2_SwitchOffPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;

  /* Disable UPDATE ISR */
  LL_TIM_DisableIT_UPDATE( TIMx );

  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs( TIMx );
  if ( (pHandle->_Super.LowSideOutputs) == ES_GPIO )
  {
    LL_GPIO_ResetOutputPin (pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
    LL_GPIO_ResetOutputPin (pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
    LL_GPIO_ResetOutputPin (pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
  }
  
  /* wait for a new PWM period to flush last HF task */
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE(TIMx);

  return;
}

/*
  * @brief  Writes into peripheral registers the new duty cycles and sampling point.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  hCCR4Reg: New capture/compare register value, written in timer clock counts.
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
__STATIC_INLINE uint16_t R3_2_WriteTIMRegisters( PWMC_Handle_t * pHdl, uint16_t hCCR4Reg )
{
  PWMC_R3_2_Handle_t * pHandle = (PWMC_R3_2_Handle_t *) pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  uint16_t hAux;

  LL_TIM_OC_SetCompareCH1( TIMx, pHandle->_Super.CntPhA );
  LL_TIM_OC_SetCompareCH2( TIMx, pHandle->_Super.CntPhB );
  LL_TIM_OC_SetCompareCH3( TIMx, pHandle->_Super.CntPhC );
  LL_TIM_OC_SetCompareCH4( TIMx, hCCR4Reg );

  /* Limit for update event */
  /* Check the if TIMx CH4 is enabled. If it is set, an update event has occurred
  and thus the FOC rate is too high */
  if (LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
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
__weak uint16_t R3_2_SetADCSampPointCalibration( PWMC_Handle_t * pHdl)
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;

  /* Set rising edge trigger (default) */
  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;
  pHandle->_Super.Sector = pHandle->CalibSector;

  return R3_2_WriteTIMRegisters( &pHandle->_Super, (uint32_t)(pHandle->Half_PWMPeriod - 1u) );
}

/*
  * @brief  Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval Return value of R3_1_WriteTIMRegisters.
  */
__weak uint16_t R3_2_SetADCSampPointSectX( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;

  uint16_t hCntSmp;
  uint16_t hDeltaDuty;
  register uint16_t lowDuty = pHdl->lowDuty;
  register uint16_t midDuty = pHdl->midDuty;

  /* Check if sampling AB in the middle of PWM is possible */
  if ( ( uint16_t )( pHandle->Half_PWMPeriod - lowDuty ) > pHandle->pParams_str->hTafter )
  {
    /* When it is possible to sample in the middle of the PWM period, always sample the same phases
     * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
     * between offsets */

    /* sector number needed by GetPhaseCurrent, phase A and B are sampled wIch corresponds
     * to sector 4 */
    pHandle->_Super.Sector = SECTOR_4;

    /* set sampling  point trigger in the middle of PWM period */
    hCntSmp = ( uint32_t )( pHandle->Half_PWMPeriod ) - 1u;
  }
  else
  {
    /* ADC Injected sequence configuration. The stator phase with minimum value of complementary
    duty cycle is set as first. In every sector there is always one phase with maximum complementary duty,
    one with minimum complementary duty and one with variable complementary duty. In this case, phases
    with variable complementary duty and with maximum duty are converted and the first will be always
    the phase with variable complementary duty cycle */

    /* Crossing Point Searching */
    hDeltaDuty = ( uint16_t )( lowDuty - midDuty );

    /* Definition of crossing point */
    if ( hDeltaDuty > ( uint16_t )( pHandle->Half_PWMPeriod - lowDuty ) * 2u )
    {
      /* hTbefore = 2*Ts + Tc, where Ts = Sampling time of ADC, Tc = Conversion Time of ADC */
      hCntSmp = lowDuty - pHandle->pParams_str->hTbefore;
    }
    else
    {
      /* hTafter = DT + max(Trise, Tnoise) */
      hCntSmp = lowDuty + pHandle->pParams_str->hTafter;

      if ( hCntSmp >= pHandle->Half_PWMPeriod )
      {
          /* It must be changed the trigger direction from positive to negative
               to sample after middle of PWM*/
        pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_FALLING;

        hCntSmp = ( 2u * pHandle->Half_PWMPeriod ) - hCntSmp - 1u;
      }
    }
  }

  return R3_2_WriteTIMRegisters( &pHandle->_Super, hCntSmp );
}

/*
  * @brief  Configures the ADC for the current sampling related to sector X (X = [1..6] ) in case of overmodulation.
  * 
  * It sets the ADC sequence length and channels, and the sampling point via TIMx_Ch4 value and polarity.
  * It then calls the WriteTIMRegisters method.
  * 
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @retval Return value of R3_1_WriteTIMRegisters.
  */
uint16_t R3_2_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHdl )
{
#if defined (__ICCARM__)
#pragma cstat_disable = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
#if defined (__ICCARM__)
#pragma cstat_restore = "MISRAC2012-Rule-11.3"
#endif /* __ICCARM__ */
  uint16_t SamplingPoint;
  uint16_t DeltaDuty;

  pHandle->_Super.useEstCurrent = false;
  DeltaDuty = ( uint16_t )( pHdl->lowDuty - pHdl->midDuty );

  /* case 1 (cf user manual) */
  if (( uint16_t )( pHandle->Half_PWMPeriod - pHdl->lowDuty ) > pHandle->pParams_str->hTafter)
  {
  /* When it is possible to sample in the middle of the PWM period, always sample the same phases
   * (AB are chosen) for all sectors in order to not induce current discontinuities when there are differences
   * between offsets */

  /* sector number needed by GetPhaseCurrent, phase A and B are sampled which corresponds
   * to sector 4 or 5  */
    pHandle->_Super.Sector = SECTOR_5;

    /* set sampling  point trigger in the middle of PWM period */
    SamplingPoint =  pHandle->Half_PWMPeriod - (uint16_t) 1;
  }
  else /* case 2 (cf user manual) */
  {
    if (DeltaDuty >= pHandle->pParams_str->Tcase2)
    {
      SamplingPoint = pHdl->lowDuty - pHandle->pParams_str->hTbefore;
    }
    else
    {
      /* case 3 (cf user manual) */
      if ((pHandle->Half_PWMPeriod - pHdl->lowDuty) > pHandle->pParams_str->Tcase3)
      {
        /* ADC trigger edge must be changed from positive to negative */
        pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_FALLING;
        SamplingPoint = pHdl->lowDuty + pHandle->pParams_str->Tsampling;
      }
      else
      {

        SamplingPoint = pHandle->Half_PWMPeriod-1;
        pHandle->_Super.useEstCurrent = true;

      }
    }
  }
  return R3_2_WriteTIMRegisters( &pHandle->_Super, SamplingPoint );
}

/*
  * @brief  Contains the TIMx Update event interrupt.
  *
  * @param  pHandle: Handler of the current instance of the PWM component.
  */
__weak void * R3_2_TIMx_UP_IRQHandler( PWMC_R3_2_Handle_t * pHandle)
{
  TIM_TypeDef* TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCDataReg1[0];
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCDataReg2[0];
  uint32_t ADCInjFlags;
  
  /* dual drive check */
  ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
  if ( ADCInjFlags == LL_ADC_FLAG_JSTRT )
  {
    /* ADC conversion is on going on the second motor */
    do
    {
      /* wait for end of conversion */
      ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
    } 
    while ( ADCInjFlags != (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS) );
  }
  else if ( ADCInjFlags == 0 )
  {
    /* ADC conversion on the second motor is not yet started */
    while ( ( TIMx->CNT ) < ( pHandle->pParams_str->Tw ) )
    {
      /* wait for a maximum delay */
    }
    
    ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
    if ( ADCInjFlags == LL_ADC_FLAG_JSTRT )
    {
      /* ADC conversion is on going on the second motor */
      do
      {
        /* wait for end of conversion */
        ADCInjFlags = ADCx_1->SR & (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS);
      }
      while ( ADCInjFlags != (LL_ADC_FLAG_JSTRT|LL_ADC_FLAG_JEOS) );
    }
  }
  else
  {
    /* ADC conversion on the second motor is done */
  }
  
  /* Disabling trigger to avoid unwanted conversion */
  LL_ADC_INJ_StopConversionExtTrig(ADCx_1);
  LL_ADC_INJ_StopConversionExtTrig(ADCx_2);

  /* Set next current channel according to sector  */
  ADCx_1->JSQR = pHandle->pParams_str->ADCConfig1[pHandle->_Super.Sector];
  ADCx_2->JSQR = pHandle->pParams_str->ADCConfig2[pHandle->_Super.Sector];

  LL_ADC_INJ_SetTriggerSource(ADCx_1,pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_SetTriggerSource(ADCx_2,pHandle->ADC_ExternalTriggerInjected);
    
  /* enable ADC trigger source */
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_1, pHandle->ADCTriggerEdge);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_2, pHandle->ADCTriggerEdge);

  /* reset default edge detection trigger */
  pHandle->ADCTriggerEdge = LL_ADC_INJ_TRIG_EXT_RISING;

  return &( pHandle->_Super.Motor );
}

/*
  * @brief  Sets the PWM mode for R/L detection.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void R3_2_RLDetectionModeEnable( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == false )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    LL_TIM_OC_SetCompareCH1( TIMx, 0u );

    /*  Channel2 configuration */
    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_ACTIVE );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_INACTIVE );
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM2 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3 );
    LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );

  }

  pHandle->_Super.pFctGetPhaseCurrents = &R3_2_RLGetPhaseCurrents;
  pHandle->_Super.pFctTurnOnLowSides = &R3_2_RLTurnOnLowSides;
  pHandle->_Super.pFctSwitchOnPwm = &R3_2_RLSwitchOnPWM;
  pHandle->_Super.pFctSwitchOffPwm = &R3_2_SwitchOffPWM;

  pHandle->_Super.RLDetectionMode = true;
}

/*
  * @brief  Disables the PWM mode for R/L detection.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
void R3_2_RLDetectionModeDisable( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  if ( pHandle->_Super.RLDetectionMode == true )
  {
    /*  Channel1 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH1N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH1( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel2 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH2N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH2( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    /*  Channel3 configuration */
    LL_TIM_OC_SetMode ( TIMx, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1 );
    LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3 );

    if ( ( pHandle->_Super.LowSideOutputs ) == LS_PWM_TIMER )
    {
      LL_TIM_CC_EnableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
    {
      LL_TIM_CC_DisableChannel( TIMx, LL_TIM_CHANNEL_CH3N );
    }
    else
    {
    }

    LL_TIM_OC_SetCompareCH3( TIMx, ( uint32_t )( pHandle->Half_PWMPeriod ) >> 1 );

    pHandle->_Super.pFctGetPhaseCurrents = &R3_2_GetPhaseCurrents;
    pHandle->_Super.pFctTurnOnLowSides = &R3_2_TurnOnLowSides;
    pHandle->_Super.pFctSwitchOnPwm = &R3_2_SwitchOnPWM;
    pHandle->_Super.pFctSwitchOffPwm = &R3_2_SwitchOffPWM;

    pHandle->_Super.RLDetectionMode = false;
  }
}

/*
  * @brief  Sets the PWM dutycycle for R/L detection.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  hDuty: Duty cycle to apply, written in uint16_t.
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
uint16_t R3_2_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  uint32_t val;
  uint16_t hAux;

  val = ( ( uint32_t )( pHandle->Half_PWMPeriod ) * ( uint32_t )( hDuty ) ) >> 16;
  pHandle->_Super.CntPhA = ( uint16_t )( val );

  /* TIM1 Channel 1 Duty Cycle configuration.
   * In RL Detection mode only the Up-side device of Phase A are controlled*/
  LL_TIM_OC_SetCompareCH1(TIMx, ( uint32_t )pHandle->_Super.CntPhA);

  /* set the sector that correspond to Phase A and B sampling */
  pHdl->Sector = SECTOR_4;

  /* Limit for update event */
  /* Check the status flag. If an update event has occurred before to set new
  values of regs the FOC rate is too high */
  if (LL_TIM_CC_IsEnabledChannel(TIMx, LL_TIM_CHANNEL_CH4))
  {
    hAux = MC_DURATION;
  }
  else
  {
    hAux = MC_NO_ERROR;
  }
  if ( pHandle->_Super.SWerror == 1u )
  {
    hAux = MC_DURATION;
    pHandle->_Super.SWerror = 0u;
  }
  return hAux;
}

/*
  * @brief  Computes and stores into @p pHandle latest converted motor phase currents
  *         during RL detection phase.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  pStator_Currents: Pointer to the structure that will receive motor current
  *         of phase A and B in ab_t format.
  */
void R3_2_RLGetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  int32_t wAux;

  /* disable ADC trigger source */
  LL_TIM_CC_DisableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  wAux = (int32_t)( pHandle->PhaseBOffset ) - (int32_t)pHandle->pParams_str->ADCDataReg2[pHandle->_Super.Sector]*2;

  /* Check saturation */
  if ( wAux > -INT16_MAX )
  {
    if ( wAux < INT16_MAX )
    {
    }
    else
    {
      wAux = INT16_MAX;
    }
  }
  else
  {
    wAux = -INT16_MAX;
  }

  pStator_Currents->a = (int16_t)wAux;
  pStator_Currents->b = (int16_t)wAux;
}

/*
  * @brief  Turns on low sides switches.
  * 
  * This function is intended to be used for charging boot capacitors
  * of driving section. It has to be called at each motor start-up when
  * using high voltage drivers.
  * This function is specific for RL detection phase.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  * @param  ticks: Duty cycle of the boot capacitors charge, specific to motor.
  */
static void R3_2_RLTurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;

  /*Turn on the phase A low side switch */
  LL_TIM_OC_SetCompareCH1 ( TIMx, 0u );

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Wait until next update */
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
    LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
    LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }
  return;
}


/*
  * @brief  Enables PWM generation on the proper Timer peripheral.
  * 
  * This function is specific for RL detection phase.
  *
  * @param  pHdl: Handler of the current instance of the PWM component.
  */
static void R3_2_RLSwitchOnPWM( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;


  /* wait for a new PWM period */
  LL_TIM_ClearFlag_UPDATE( TIMx );
  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1( TIMx, 1u );
  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod ) - 5u );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  LL_TIM_ClearFlag_UPDATE( TIMx );
  

  
  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
    if ( ( TIMx->CCER & TIMxCCER_MASK_CH123 ) != 0u )
    {
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_ResetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
    }
  }
  
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  /* enable TIMx update interrupt*/
  LL_TIM_EnableIT_UPDATE( TIMx );

  return;
}

/*
 * @brief  Turns on low sides switches and start ADC triggering.
 * 
 * This function is specific for MP phase.
 *
 * @param  pHdl: Handler of the current instance of the PWM component.
 */
void R3_2_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl )
{
  PWMC_R3_2_Handle_t * pHandle = ( PWMC_R3_2_Handle_t * )pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx_1 = pHandle->pParams_str->ADCDataReg1[0];
  ADC_TypeDef * ADCx_2 = pHandle->pParams_str->ADCDataReg2[0];

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}
  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE( TIMx );

  LL_TIM_OC_SetCompareCH1 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH2 ( TIMx, 0x0u );
  LL_TIM_OC_SetCompareCH3 ( TIMx, 0x0u );

  LL_TIM_OC_SetCompareCH4( TIMx, ( pHandle->Half_PWMPeriod - 5u));

  while ( LL_TIM_IsActiveFlag_UPDATE( TIMx ) == 0 )
  {}

  /* Main PWM Output Enable */
  TIMx->BDTR |= LL_TIM_OSSI_ENABLE ;
  LL_TIM_EnableAllOutputs ( TIMx );

  if ( ( pHandle->_Super.LowSideOutputs ) == ES_GPIO )
  {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin );
      LL_GPIO_SetOutputPin( pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin );
  }

  pHdl->Sector = SECTOR_4;
  LL_ADC_INJ_SetTriggerSource( ADCx_1, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_SetTriggerSource( ADCx_2, pHandle->ADC_ExternalTriggerInjected);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_1,LL_ADC_INJ_TRIG_EXT_RISING);
  LL_ADC_INJ_StartConversionExtTrig(ADCx_2,LL_ADC_INJ_TRIG_EXT_RISING);

  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  LL_TIM_EnableIT_UPDATE( TIMx );

  return;
}


/**
 * @brief  Sets ADC sampling points.
 * 
 * This function is specific for MP phase. Specific to F4XX, F7XX, L4XX and H5XX.
 * 
 * @param  pHdl: Handler of the current instance of the PWM component.
 */
void RLSetADCSampPoint( PWMC_Handle_t * pHdl )
{
  /* dummy sector setting to get correct Ia value */
  pHdl->Sector = SECTOR_4;

  return;
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

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
