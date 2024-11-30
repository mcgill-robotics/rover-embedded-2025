<#ftl strip_whitespace = true>
<#include "*/ftl/header.ftl">
<#include "*/ftl/common_assign.ftl">
<#include "*/ftl/common_fct.ftl">
<#include "*/ftl/ip_assign.ftl">
/**
  ******************************************************************************
  * @file    r1_ps_pwm_curr_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the following features
  *          of the CCC component of the Motor Control SDK:
  *           + initializes MCU peripheral for 1 shunt topology and F3 family
  *           + performs PWM duty cycle computation and generation
  *           + performs current sensing
  *           +
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
  */

/* Includes ------------------------------------------------------------------*/
#include "r1_ps_pwm_curr_fdbk.h"
#include "pwm_common.h"
#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/**
 * @defgroup r1_ps_pwm_curr_fdbk R1 F30x PWM & Current Feedback
 *
 * @brief STM32F3, 1-Shunt with phase shift PWM & Current Feedback implementation
 *
 * This component is used in applications based on an STM32F3 MCU
 * and using a single shunt resistor current sensing topology.
 *
 * @todo: TODO: complete documentation.
 * @{
 */

/* Constant values -----------------------------------------------------------*/
#define TIMxCCER_MASK_CH123 (TIM_CCER_CC1E|TIM_CCER_CC2E|TIM_CCER_CC3E|\
                             TIM_CCER_CC1NE|TIM_CCER_CC2NE|TIM_CCER_CC3NE)


#define DMA_CFG (LL_DMA_PERIPH_NOINCREMENT|LL_DMA_MEMORY_INCREMENT|LL_DMA_PDATAALIGN_HALFWORD|\
                 LL_DMA_DIRECTION_MEMORY_TO_PERIPH|LL_DMA_MDATAALIGN_HALFWORD|LL_DMA_PRIORITY_VERYHIGH|\
                 LL_DMA_MODE_${DMA_MODE})


#define DMA_TRANSFER_LENGTH_CCR             6u
<#if CondFamily_STM32F4>
#define DMA_TRANSFER_LENGTH_SAMPLING_POINT  2u 
<#else><#-- CondFamily_STM32F4 == false -->
#define DMA_TRANSFER_LENGTH_SAMPLING_POINT  3u 
</#if><#-- CondFamily_STM32F4 -->
#define DMA_TRANSFER_LENGTH_ADC             2u 
#define IA_OK                               0x01
#define IB_OK                               0x02
#define IC_OK                               0x04

static const int8_t ALFLAG[3] = {IA_OK,IB_OK,IC_OK};
/* Private typedef -----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static void R1_HFCurrentsCalibration(PWMC_Handle_t * pHdl, ab_t * pStator_Currents);
<#if CondFamily_STM32F3 || CondFamily_STM32G4>
  <#if MC.M1_OCP_TOPOLOGY == "EMBEDDED" || MC.M2_OCP_TOPOLOGY == "EMBEDDED" || 
       MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true>
static void R1_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref);
  </#if><#-- MC.M1_OCP_TOPOLOGY == "EMBEDDED" || MC.M2_OCP_TOPOLOGY == "EMBEDDED" || 
             MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true -->
</#if><#-- CondFamily_STM32F3 || CondFamily_STM32G4 -->
static void R1_1ShuntMotorVarsInit(PWMC_Handle_t *pHdl);
static void R1_TIMxInit(TIM_TypeDef *TIMx, PWMC_R1_Handle_t *pHdl);
static uint16_t R1_SetADCSampPointPolarization(PWMC_Handle_t *pHdl);

/**
 * @brief  It initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
 *         in ICS configuration using STM32F103x High Density
 * @param pHandle: handler of the current instance of the PWM component
 */
void R1_Init(PWMC_R1_Handle_t * pHandle)
{
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;
  DMA_TypeDef * DMAx = pHandle->pParams_str->DMAx;

  R1_1ShuntMotorVarsInit(&pHandle->_Super);

  /********************************************************************************************/
  /*                                      TIM Initialization                                  */
  /********************************************************************************************/
  R1_TIMxInit(TIMx, pHandle);
  
  /********************************************************************************************/
  /*                                      DMA Initialization                                  */
  /********************************************************************************************/
  LL_DMA_ConfigTransfer(DMAx, pHandle->pParams_str->DMAChannelX, DMA_CFG); /* To be removed should be done by cubeMX */

  LL_TIM_ConfigDMABurst(TIMx,
                        LL_TIM_DMABURST_BASEADDR_CCR1,
                        LL_TIM_DMABURST_LENGTH_3TRANSFERS);

  /* cfg dma source and dest address */
  LL_DMA_SetMemoryAddress(DMAx, pHandle->pParams_str->DMAChannelX, (uint32_t)&pHandle->DmaBuffCCR[0]);
  LL_DMA_SetPeriphAddress(DMAx, pHandle->pParams_str->DMAChannelX, (uint32_t) &TIMx->DMAR);
  /* cfg dma transfer size */
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH_CCR);
  
  /* DMA dedicated to TIMx CC register modification of the ADC sampling point */
  LL_DMA_ConfigTransfer(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, DMA_CFG); /* To be removed should be done by cubeMX */
  /* Set dma source and dest address for TIMx CC register modification on the fly */
  LL_DMA_SetMemoryAddress(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, (uint32_t)&pHandle->DmaBuffCCR_ADCTrig[0]);
  LL_DMA_SetPeriphAddress(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, (uint32_t) &TIMx->CCR4);
  /* Set dma transfer size */
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, DMA_TRANSFER_LENGTH_SAMPLING_POINT);
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);

<#if HAS_ADC_INJ == false>
  /* DMA dedicated to ADC conversion */
  LL_DMA_SetMemoryAddress(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, (uint32_t)pHandle->CurConv);
  LL_DMA_SetPeriphAddress(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, (uint32_t)&ADCx->DR);
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, DMA_TRANSFER_LENGTH_ADC);
</#if><#-- HAS_ADC_INJ == false -->

  /********************************************************************************************/
  /*                                      ADC Initialization                                  */
  /********************************************************************************************/

    /* disable IT and flags in case of LL driver usage
   * workaround for unwanted interrupt enabling done by LL driver */
<#if HAS_ADC_INJ == false>
  LL_ADC_DisableIT_EOC(ADCx);
  LL_ADC_ClearFlag_EOC(ADCx);
  LL_ADC_DisableIT_EOS(ADCx);
  LL_ADC_ClearFlag_EOS(ADCx);
  
<#else><#-- HAS_ADC_INJ == true -->
  LL_ADC_DisableIT_EOCS(ADCx);
  LL_ADC_ClearFlag_EOCS(ADCx);
  LL_ADC_DisableIT_JEOS(ADCx);
  LL_ADC_ClearFlag_JEOS(ADCx);
  
</#if><#-- HAS_ADC_INJ == false -->
  
<#if HAS_ADC_INJ == false>
  /* Start calibration of ADC1 */
  LL_ADC_StartCalibration(ADC1);
  while ((LL_ADC_IsCalibrationOnGoing(ADC1) == SET) ||
         (LL_ADC_REG_IsConversionOngoing(ADC1) == SET) ||
         (LL_ADC_REG_IsStopConversionOngoing(ADC1) == SET) ||
         (LL_ADC_IsDisableOngoing(ADC1) == SET))
  {
    /* wait */
  }
  /* Enable ADC */
  LL_ADC_Enable(ADC1);
  /* Enable ADC DMA request*/
  LL_ADC_REG_SetTriggerSource (ADCx, LL_ADC_REG_TRIG_SOFTWARE);
  LL_ADC_REG_SetDMATransfer(ADC1 , LL_ADC_REG_DMA_TRANSFER_LIMITED);

  /* Wait ADC Ready */
  while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == RESET)
  {}  
  
  LL_ADC_REG_SetTriggerSource (ADCx, LL_ADC_REG_TRIG_EXT_TIM1_TRGO);
  LL_ADC_REG_SetSequencerChannels (ADCx, __LL_ADC_DECIMAL_NB_TO_CHANNEL (pHandle->pParams_str->IChannel));
    
<#else><#-- HAS_ADC_INJ == true -->
  if (LL_ADC_IsEnabled (ADCx) == 0)
  {
    LL_ADC_Enable (ADCx);
  }
  else
  {
    /* Nothing to do */
  }
  
  /* work-around cubeMX code generator bug */
  /* overwrite cubeMX ADC JSQR setting     */
  if (TIMx == TIM1)
  {
    pHandle->AdcExtTrigger = LL_ADC_INJ_TRIG_EXT_TIM1_TRGO;
  }
#ifdef TIM8 
  else 
  {
    pHandle->AdcExtTrigger = LL_ADC_INJ_TRIG_EXT_TIM8_CH4; /* Should be removed with cubeMX */
  }
#else
  else
  {
    /* Nothing to do */
  }
#endif 
  LL_ADC_INJ_SetTriggerSource(ADCx, LL_ADC_INJ_TRIG_SOFTWARE);
  LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_1, pHandle->pParams_str->IChannel);
  LL_ADC_INJ_SetSequencerRanks(ADCx, LL_ADC_INJ_RANK_2, pHandle->pParams_str->IChannel);
  LL_ADC_INJ_SetSequencerLength(ADCx, LL_ADC_INJ_SEQ_SCAN_ENABLE_2RANKS);
  LL_ADC_INJ_SetSequencerDiscont(ADCx, LL_ADC_INJ_SEQ_DISCONT_1RANK);
  LL_ADC_INJ_StopConversionExtTrig(ADCx);
</#if><#-- HAS_ADC_INJ == false -->

  /* Clear the flags */
<#if HAS_ADC_INJ == false>
  LL_TIM_EnableCounter(TIM1);

  pHandle->ADCRegularLocked=false; /* We allow ADC usage for regular conversion on Systick */
</#if><#-- HAS_ADC_INJ == false -->


}

/**
  * @brief  It initializes TIMx peripheral for PWM generation
  * @param TIMx: Timer to be initialized
  * @param pHandle: handler of the current instance of the PWM component
  */
void R1_TIMxInit(TIM_TypeDef * TIMx, PWMC_R1_Handle_t * pHandle)
{

  /* Freeze timer for the debug */
<#if (MC.DRIVE_NUMBER)?number gt 1 >
  if (pHandle->pParams_str->TIMx == TIM1)
  {
    /* TIM1 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_${_last_word(MC.M1_PWM_TIMER_SELECTION)}_STOP);
  }
  else
  {
    /* TIM8 Counter Clock stopped when the core is halted */
    LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_${_last_word(MC.M2_PWM_TIMER_SELECTION)}_STOP);
  }
<#else><#-- MC.DRIVE_NUMBER == 1 -->
  <#if CondFamily_STM32F0 == true>
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_DBGMCU);
  LL_DBGMCU_APB1_GRP2_FreezePeriph(LL_DBGMCU_APB1_GRP2_${_last_word(MC.M1_PWM_TIMER_SELECTION)}_STOP);
  <#else><#--  CondFamily_STM32F0 == false -->
  LL_DBGMCU_APB2_GRP1_FreezePeriph(LL_DBGMCU_APB2_GRP1_${_last_word(MC.M1_PWM_TIMER_SELECTION)}_STOP);
  </#if><#-- CondFamily_STM32F0 == true -->
</#if><#-- MC.DRIVE_NUMBER > 1 -->
  /* disable main TIM counter to ensure
   * a synchronous start by TIM2 trigger */
  LL_TIM_DisableCounter(TIMx);

  /* Update CC interrupt flag only during up counting */
  LL_TIM_SetCounterMode(TIMx, TIM_CR1_CMS_1);
  
  /* Disable ADC trigger */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
    
  /* Disable preload register to be able to update CC registers
     twice per PWM cycle with DMA for phase shifting */
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_DisablePreload(TIMx, LL_TIM_CHANNEL_CH4);
  LL_TIM_CC_EnableChannel(TIMx, LL_TIM_CHANNEL_CH4);

  /* Update event generation at each overflow/underflow to update 
     CC register twice per PWM cycle with DMA for phase shifting */
  LL_TIM_SetRepetitionCounter(TIMx, 0);
  
  /* Always enable BKIN for safety feature */
  LL_TIM_ClearFlag_BRK(TIMx);
  LL_TIM_EnableIT_BRK(TIMx);
  
  /* Enable drive of TIMx CHy and CHyN by TIMx CHyRef*/
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);
  <#if MC.DRIVE_NUMBER != "1">
 if (M1 == pHandle->_Super.Motor)
  {
    LL_TIM_SetCounter(TIMx, (uint32_t)(pHandle->Half_PWMPeriod) - 1U);
  }
</#if><#-- MC.DRIVE_NUMBER != "1" -->
}

/**
  * @brief  First initialization of the handler
  * @param pHdl: handler of the current instance of the PWM component
  */
void R1_1ShuntMotorVarsInit(PWMC_Handle_t * pHdl)
{
  PWMC_R1_Handle_t * pHandle = (PWMC_R1_Handle_t *)pHdl;

  /* Init motor vars */
  pHandle->iflag = 0;
  pHandle->FOCDurationFlag = false;
  pHandle->Half_PWMPeriod = (pHandle->_Super.PWMperiod/2u);

  pHandle->CntSmp1 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   - (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   + (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
      
  pHandle->_Super.CntPhA = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhB = pHandle->Half_PWMPeriod >> 1;
  pHandle->_Super.CntPhC = pHandle->Half_PWMPeriod >> 1;

  /* initialize buffer with the default duty cycle value */
  pHandle->DmaBuffCCR[0]       = pHandle->_Super.CntPhA;      /* CCR1 value overwritten during first half PWM period */
  pHandle->DmaBuffCCR_latch[0] = pHandle->_Super.CntPhA;      /* CCR1 value overwritten during first half PWM period */
  pHandle->DmaBuffCCR[1]       = pHandle->_Super.CntPhB;      /* CCR2 value overwritten during first half PWM period */
  pHandle->DmaBuffCCR_latch[1] = pHandle->_Super.CntPhB;      /* CCR2 value overwritten during first half PWM period */
  pHandle->DmaBuffCCR[2]       = pHandle->_Super.CntPhC;      /* CCR3 value overwritten during first half PWM period */
  pHandle->DmaBuffCCR_latch[2] = pHandle->_Super.CntPhC;      /* CCR3 value overwritten during first half PWM period */
  
  pHandle->DmaBuffCCR[3]       = pHandle->_Super.CntPhA;      /* CCR1 value overwritten during second half PWM period */
  pHandle->DmaBuffCCR_latch[3] = pHandle->_Super.CntPhA;      /* CCR1 value overwritten during second half PWM period */
  pHandle->DmaBuffCCR[4]       = pHandle->_Super.CntPhB;      /* CCR2 value overwritten during second half PWM period */
  pHandle->DmaBuffCCR_latch[4] = pHandle->_Super.CntPhB;      /* CCR2 value overwritten during second half PWM period */
  pHandle->DmaBuffCCR[5]       = pHandle->_Super.CntPhC;      /* CCR3 value overwritten during second half PWM period */
  pHandle->DmaBuffCCR_latch[5] = pHandle->_Super.CntPhC;      /* CCR3 value overwritten during second half PWM period */

  /* initialize buffer with default sampling value */
  pHandle->DmaBuffCCR_ADCTrig[0] = pHandle->CntSmp2;
<#if CondFamily_STM32F4 == true>
  pHandle->DmaBuffCCR_ADCTrig[1] = pHandle->Half_PWMPeriod+1u;
<#else><#-- CondFamily_STM32F4 == false -->
  pHandle->DmaBuffCCR_ADCTrig[1] = pHandle->Half_PWMPeriod-1u;
</#if><#-- CondFamily_STM32F4 == true -->
  pHandle->DmaBuffCCR_ADCTrig[2] = pHandle->CntSmp1;
  
  pHandle->_Super.BrakeActionLock = false;
}

/**
  * @brief  It sets the calibrated offsets
  * @param pHdl: handler of the current instance of the PWM component
  * @param offsets: pointer to the structure that contains the offsets
  */
__weak void R1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  pHandle->PhaseOffset = offsets->phaseAOffset;

  pHdl->offsetCalibStatus = true;
}

/**
  * @brief  It reads the calibrated offsets
  * @param pHdl: handler of the current instance of the PWM component
  * @param offsets: pointer to the structure that will contain the offsets
  */
__weak void R1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl; //cstat !MISRAC2012-Rule-11.3

  offsets->phaseAOffset = pHandle->PhaseOffset;
}

/**
  * @brief It stores into pHandle the offset voltage read onchannels when no
  * current is flowing into the motor
  * @param pHdl: handler of the current instance of the PWM component
  */
__weak void R1_CurrentReadingCalibration(PWMC_Handle_t * pHdl)
{
  PWMC_R1_Handle_t * pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
 
  if (false == pHandle->_Super.offsetCalibStatus)
  {
    pHandle->PhaseOffset = 0u;
    pHandle->Index = 0u;
    
    /* It forces inactive level on TIMx CHy and CHyN */
    LL_TIM_CC_DisableChannel(TIMx, TIMxCCER_MASK_CH123);
    
    /* Offset calibration */
    /* Change function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R1_HFCurrentsCalibration;
    pHandle->_Super.pFctSetADCSampPointSectX = &R1_SetADCSampPointPolarization;
    
    R1_SwitchOnPWM(&pHandle->_Super);
    
    /* Wait for NB_CONVERSIONS to be executed */
    waitForPolarizationEnd(TIMx,
                  &pHandle->_Super.SWerror,
            pHandle->pParams_str->RepetitionCounter,
            &pHandle->Index);
    
    R1_SwitchOffPWM(&pHandle->_Super);
    
    pHandle->PhaseOffset >>= 4u;
    if (0U == pHandle->_Super.SWerror)
    {
      pHandle->_Super.offsetCalibStatus = true;
    }
    else
    {
      /* nothing to do */
    }
    
    /* Change back function to be executed in ADCx_ISR */
    pHandle->_Super.pFctGetPhaseCurrents = &R1_GetPhaseCurrents;
    pHandle->_Super.pFctSetADCSampPointSectX = &R1_CalcDutyCycles;
  }
  else
  {
    /* Nothing to do */
  }

  /* It over write TIMx CCRy wrongly written by FOC during calibration so as to
   force 50% duty cycle on the three inverer legs */
  /* Disable TIMx preload */
  LL_TIM_OC_SetCompareCH1 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH2 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  LL_TIM_OC_SetCompareCH3 (TIMx, pHandle->Half_PWMPeriod >> 1u);
  /* generate  COM event to apply new CC values */
  LL_TIM_GenerateEvent_COM( TIMx );

  /* It re-enable drive of TIMx CHy and CHyN by TIMx CHyRef */
  LL_TIM_CC_EnableChannel(TIMx, TIMxCCER_MASK_CH123);

  R1_1ShuntMotorVarsInit(&pHandle->_Super);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  It computes and return latest converted motor phase currents motor
  * @param pHdl: handler of the current instance of the PWM component
  * @retval Ia and Ib current in Curr_Components format
  */
__weak void R1_GetPhaseCurrents(PWMC_Handle_t * pHdl, ab_t * pStator_Currents)
{
  PWMC_R1_Handle_t * pHandle = (PWMC_R1_Handle_t *)pHdl;
<#if CondFamily_STM32F0>
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
</#if><#-- CondFamily_STM32F0 -->
<#if HAS_ADC_INJ == true>
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;
</#if><#-- HAS_ADC_INJ == true -->
  int32_t wAux1;
  int32_t wAux2;
  int16_t hCurrA = 0;
  int16_t hCurrB = 0;
  int16_t hCurrC = 0;
  
  /* Clear flag used for FOC duration check */
  pHandle->FOCDurationFlag = false;

  /* Disabling the External triggering for ADCx */
<#if CondFamily_STM32F4>
  LL_ADC_INJ_StopConversionExtTrig(ADCx);
<#else><#-- CondFamily_STM32F4 == false -->
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
</#if><#-- CondFamily_STM32F4 -->
  /* First sampling point */
<#if HAS_ADC_INJ == false>
  wAux1 = (int32_t) pHandle->CurConv[0] ${currentFactor};
<#else><#-- HAS_ADC_INJ == true -->
  wAux1 = (int32_t)(ADCx->JDR1 ${currentFactor});
</#if><#-- HAS_ADC_INJ == false -->
  wAux1 -= (int32_t)(pHandle->PhaseOffset);
  
  /* Check saturation */
  if (wAux1 > -INT16_MAX)
  {
    if (wAux1 < INT16_MAX)
    {
    }
    else
    {
      wAux1 = INT16_MAX;
    }
  }
  else
  {
    wAux1 = -INT16_MAX;
  } 
   /* Second sampling point */
<#if HAS_ADC_INJ == false>
  wAux2 = (int32_t) pHandle->CurConv[1] ${currentFactor};
<#else><#-- HAS_ADC_INJ == true -->
  wAux2 = (int32_t)(ADCx->JDR2 ${currentFactor});
</#if><#-- HAS_ADC_INJ == false -->
  wAux2 -= (int32_t)(pHandle->PhaseOffset);

  /* Check saturation */
  if (wAux2 > -INT16_MAX)
  {
    if (wAux2 < INT16_MAX)
    {
    }
    else
    {
      wAux2 = INT16_MAX;
    }
  }
  else
  {
    wAux2 = -INT16_MAX;
  }


  switch (pHandle->_Super.Sector)
  {
    case SECTOR_1:
    {
      if((pHandle->iflag & (IA_OK | IC_OK)) == (IA_OK | IC_OK)) /* iA and -iC are available to be sampled */
      {
        hCurrA = (int16_t) wAux2;
        wAux1 = -wAux1;
        hCurrC = (int16_t) wAux1;
        hCurrB = -hCurrA-hCurrC;
      }
      else
      {
        if((pHandle->iflag & (IA_OK | IC_OK)) != 0x00) /* iA or -iC is available to be sampled */
        {
          if(pHandle->_Super.AlignFlag == 0x01) /* START Position Aligning_angle=30 degree */
          {
            if((pHandle->iflag & (IA_OK | IC_OK)) == IA_OK) /* iA is available to be sampled and not iC */
            {
              hCurrA = (int16_t) wAux2;
              hCurrB = 0;
              hCurrC = -hCurrA;
            }
            else   /* 0x04 -ic is available */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t) wAux1;
              hCurrA = -hCurrC;
              hCurrB = 0;
            }
          }
          else  /* Not START Position */
          {
            if((pHandle->iflag & (IA_OK | IC_OK)) == IA_OK) /* iA, is available to be sampled */
            {
              hCurrA = (int16_t) wAux2;
              hCurrB = pHandle->_Super.IbEst;
            }
            else   /* 0x04 -ic is available */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t) wAux1;
              hCurrB = pHandle->_Super.IbEst;
              hCurrA = -hCurrB-hCurrC;
            }
          }
        }
        else
        {
          hCurrA = pHandle->_Super.IaEst;
          hCurrC = pHandle->_Super.IcEst;
          hCurrB = -hCurrA-hCurrC;
        }
      }
      break;
    }

    case SECTOR_2:
    {
      if((pHandle->iflag & (IB_OK | IC_OK)) == (IB_OK | IC_OK)) /* iB,-iC are available to be sampled */
      {
        hCurrB = (int16_t) wAux2;
        wAux1 = -wAux1;
        hCurrC = (int16_t) wAux1;
        hCurrA = -hCurrB-hCurrC;
      }
      else
      {
        if((pHandle->iflag & (IB_OK | IC_OK)) != 0x00) /* iB, or -iC is available to be sampled */
        {
          if(pHandle->_Super.AlignFlag == 0x01) /* START Position Aligning_angle=90 degree */
          {
            if((pHandle->iflag & (IB_OK | IC_OK)) == IB_OK) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t) wAux2;
              hCurrA = 0;
            }
            else   /* 0x04 -ic */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t) wAux1;
              hCurrA = 0;
              hCurrB = -hCurrC;
           }
          }
          else  /* Not START Position */
          {
            if((pHandle->iflag & (IB_OK | IC_OK)) == IB_OK) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t) wAux2;
              hCurrA = pHandle->_Super.IaEst;
            }
            else   /* 0x04 -ic */
            {
              wAux1 = -wAux1;
              hCurrC = (int16_t) wAux1;
              hCurrA = pHandle->_Super.IaEst;
              hCurrB = -hCurrA-hCurrC;
            }
          }
        }
        else
        {
          hCurrB = pHandle->_Super.IbEst;
          hCurrC = pHandle->_Super.IcEst;
          hCurrA = -hCurrB-hCurrC;
        }
      }
      break;
    }

    case SECTOR_3:
    {
      if((pHandle->iflag & (IA_OK | IB_OK)) == (IA_OK | IB_OK)) /* iB,-iA are available to be sampled */
      { 
        hCurrB = (int16_t) wAux2;
        wAux1 = -wAux1;
        hCurrA = (int16_t) wAux1;
      }
      else
      {
        if((pHandle->iflag & (IA_OK | IB_OK)) != 0x00) /* iB, or -iA is available to be sampled */
        {
          if(pHandle->_Super.AlignFlag == 0x01) /* START Position Aligning_angle=150 degree */
          {
            if((pHandle->iflag & (IA_OK | IB_OK)) == IB_OK) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t) wAux2;
              hCurrA = -hCurrB;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t) wAux1;
              hCurrB = -hCurrA;
            }
          }
          else  /* Not START Position */
          {
            if((pHandle->iflag & (IA_OK | IB_OK)) == IB_OK) /* iB, is available to be sampled */
            {
              hCurrB = (int16_t) wAux2;
              hCurrA = pHandle->_Super.IaEst;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t) wAux1;
              hCurrB = pHandle->_Super.IbEst;
            }
          }
        }
        else
        {
          hCurrB = pHandle->_Super.IbEst;
          hCurrA = pHandle->_Super.IaEst;
        }
      }
      break;
    }

    case SECTOR_4:
    {
      if((pHandle->iflag & (IA_OK | IC_OK)) == (IA_OK | IC_OK)) /* iC,-iA are available to be sampled */
      {
        hCurrC = (int16_t) wAux2;
        wAux1 = -wAux1;
        hCurrA = (int16_t) wAux1;
        hCurrB = -hCurrA-hCurrC;
      }
      else
      {
        if((pHandle->iflag & (IA_OK | IC_OK)) != 0x00) /* iC, or -iA is available to be sampled */
        {
          if(pHandle->_Super.AlignFlag == 0x01) /* START Position Aligning_angle=210 degree */
          {
            if((pHandle->iflag & (IA_OK | IC_OK)) == IC_OK) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t) wAux2;
              hCurrA = -hCurrC;
              hCurrB = 0;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t) wAux1;
              hCurrB = 0;
            }
          }
          else  /* Not START Position */
          {
            if((pHandle->iflag & (IA_OK | IC_OK)) == IC_OK) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t) wAux2;
              hCurrB = pHandle->_Super.IbEst;
              hCurrA = -hCurrB-hCurrC;
            }
            else  /* 0x01 -ia */
            {
              wAux1 = -wAux1;
              hCurrA = (int16_t) wAux1;
              hCurrB = pHandle->_Super.IbEst;
            }
          }
        }
        else
        {
          hCurrC = pHandle->_Super.IcEst;
          hCurrA = pHandle->_Super.IaEst;
          hCurrB = -hCurrA-hCurrC;
        }
      }
      break;
    }

    case SECTOR_5:
    {
      if((pHandle->iflag & (IB_OK | IC_OK)) == (IB_OK | IC_OK)) /* iC,-iB are available to be sampled */
      {
        hCurrC = (int16_t) wAux2;
        wAux1 = -wAux1;
        hCurrB = (int16_t) wAux1;
        hCurrA = -hCurrB-hCurrC;
      }
      else
      {
        if((pHandle->iflag & (IB_OK | IC_OK)) != 0x00) /* iC, or -iB is available to be sampled */
        {
          if(pHandle->_Super.AlignFlag == 0x01) /* START Position Aligning_angle=270 degree */
          {
            if((pHandle->iflag & (IB_OK | IC_OK)) == IC_OK) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t) wAux2;
              hCurrA = 0;
              hCurrB = -hCurrC;
            }
            else  /* 0x02 -ib */
             {
              wAux1 = -wAux1;
              hCurrB = (int16_t) wAux1;
              hCurrA = 0;
            }
          }
          else  /* Not START Position */
          {
            if((pHandle->iflag & (IB_OK | IC_OK)) == IC_OK) /* iC, is available to be sampled */
            {
              hCurrC = (int16_t) wAux2;
              hCurrA = pHandle->_Super.IaEst;
              hCurrB = -hCurrA-hCurrC;
            }
            else /* 0x02 -ib */
            {
              wAux1 = -wAux1;
              hCurrB = (int16_t) wAux1;
              hCurrA = pHandle->_Super.IaEst;
            }
          }
        }
        else
        {
          hCurrC = pHandle->_Super.IcEst;
          hCurrB = pHandle->_Super.IbEst;
          hCurrA = -hCurrB-hCurrC;
        }
      }
      break;
    }

    case SECTOR_6:
    {
      if((pHandle->iflag & (IA_OK | IB_OK)) == (IA_OK | IB_OK)) /* iA,-iB are available to be sampled */
      {
        hCurrA = (int16_t) wAux2;
        wAux1 = -wAux1;
        hCurrB = (int16_t) wAux1;
      }
      else
      {
        if((pHandle->iflag & (IA_OK | IB_OK)) != 0x00) /* iA, or -iB is available to be sampled */
        {
          if(pHandle->_Super.AlignFlag == 0x01) /* START Position Aligning_angle=330 degree */
          {
            if((pHandle->iflag & (IA_OK | IB_OK)) == IA_OK) /* iA, is available to be sampled */
            {
              hCurrA = (int16_t) wAux2;
              hCurrB = -hCurrA;
            }
            else  /* 0x02 -ib */
            {
              wAux1 = -wAux1;
              hCurrB = (int16_t) wAux1;
              hCurrA = -hCurrB;
            }
          }
          else  /* Not START Position */
          {
            if((pHandle->iflag & (IA_OK | IB_OK)) == IA_OK) /* iA, is available to be sampled */
            {
              hCurrA = (int16_t) wAux2;
              hCurrB = pHandle->_Super.IbEst;
            }
            else  /* 0x02 -ib */
            {
              wAux1 = -wAux1;
              hCurrB = (int16_t) wAux1;
              hCurrA = pHandle->_Super.IaEst;
            }
          }
        }
        else
        {
          hCurrA = pHandle->_Super.IaEst;
          hCurrB = pHandle->_Super.IbEst;
        }
      }
      break;
    }

    default:
      break;
    }
  
  pHandle->CurrAOld = hCurrA;
  pHandle->CurrBOld = hCurrB;
  
  pStator_Currents->a = hCurrA;
  pStator_Currents->b = hCurrB;
}

/**
  * @brief  Implementaion of PWMC_GetPhaseCurrents to be performed during
  *         calibration. It sum up injected conversion data into wPhaseCOffset
  *         to compute the offset introduced in the current feedback
  *         network. It is requied to proper configure ADC input before to enable
  *         the offset computation.
  * @param pHdl: handler of the current instance of the PWM component
  * @retval It always returns {0,0} in Curr_Components format
  */
static void R1_HFCurrentsCalibration(PWMC_Handle_t *pHdl, ab_t *pStator_Currents)
{
  /* Derived class members container */
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
<#if CondFamily_STM32F0>
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
</#if><#-- CondFamily_STM32F0 -->
<#if HAS_ADC_INJ == true>
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
</#if><#-- HAS_ADC_INJ == true -->
  /* Clear flag used for FOC duration check */
  pHandle->FOCDurationFlag = false;
  
  /* Disabling the External triggering for ADCx */
<#if CondFamily_STM32F4>
  LL_ADC_INJ_StopConversionExtTrig(ADCx);
<#else><#-- CondFamily_STM32F4 == false -->
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
</#if><#-- CondFamily_STM32F4 -->
  if (pHandle->Index < NB_CONVERSIONS)
  {
<#if HAS_ADC_INJ == false>
    pHandle->PhaseOffset += pHandle->CurConv[1] ${currentFactor};
<#else><#-- HAS_ADC_INJ == true -->
    pHandle->PhaseOffset += ADCx->JDR2 ${currentFactor};
</#if><#-- HAS_ADC_INJ == false -->
    pHandle->Index++;
  }

  /* During offset calibration no current is flowing in the phases */
  pStator_Currents->a = 0;
  pStator_Currents->b = 0;

}

static uint16_t R1_SetADCSampPointPolarization(PWMC_Handle_t *pHdl)
{
  /* Derived class members container */
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;

  uint16_t hAux;
  pHandle->CntSmp1 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   - (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   + (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
<#if CondFamily_STM32F0>
  LL_ADC_REG_SetSequencerChannels(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL (pHandle->pParams_str->IChannel));
  LL_ADC_SetSamplingTimeCommonChannels (ADC1, pHandle->pParams_str->ISamplingTime);
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_CH4);
</#if><#-- CondFamily_STM32F0 -->
    
  /* Check software error */
  if (pHandle->FOCDurationFlag == true)
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
  else
  {
    /* Nothing to do */
  }
  return (hAux);
}

/**
  * @brief  It turns on low sides switches. This function is intended to be
  *         used for charging boot capacitors of driving section. It has to be
  *         called each motor start-up when using high voltage drivers
  * @param pHdl: handler of the current instance of the PWM component
  */
__weak void R1_TurnOnLowSides(PWMC_Handle_t *pHdl, uint32_t ticks)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;

  pHandle->_Super.TurnOnLowSidesAction = true;
  /* Turn on the three low side switches */
  LL_TIM_OC_SetCompareCH1(TIMx, ticks);
  LL_TIM_OC_SetCompareCH2(TIMx, ticks);
  LL_TIM_OC_SetCompareCH3(TIMx, ticks);

  /* Clear Update Flag */
  LL_TIM_ClearFlag_UPDATE(TIMx);

  /* Wait until next update */
  while (LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET)
  {
    /* Nothing to do */
  }

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->_Super.LowSideOutputs) == ES_GPIO)
  {
    LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
    LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
    LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
  }
  else
  {
    /* Nothing to do */
  }
}


/**
  * @brief  It enables PWM generation on the proper Timer peripheral acting on MOE
  *         bit
  * @param pHdl: handler of the current instance of the PWM component
  */
__weak void R1_SwitchOnPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
  DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;
  pHandle->CntSmp1 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   - (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   + (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
   
  pHandle->_Super.TurnOnLowSidesAction = false;

  pHandle->DmaBuffCCR_ADCTrig[0] = pHandle->CntSmp2;
  pHandle->DmaBuffCCR_ADCTrig[2] = pHandle->CntSmp1;
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod + 1));
  
  /* Set all duty to 50% */
  LL_TIM_OC_SetCompareCH1(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH2(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));
  LL_TIM_OC_SetCompareCH3(TIMx, (uint32_t)(pHandle->Half_PWMPeriod >> 1));

  /* Main PWM Output Enable */
  LL_TIM_EnableAllOutputs(TIMx);

  if ((pHandle->_Super.LowSideOutputs) == ES_GPIO)
  {
    if ((TIMx->CCER & TIMxCCER_MASK_CH123) != 0u)
    {
      LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_SetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
    else
    {
      /* It is executed during calibration phase the EN signal shall stay off */
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
  }
  else
  {
    /* Nothing to do */
  }
  
  /* Wait for second half PWM cycle to enable dma request */
  if (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
  {
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  }
  else
  {
    while (LL_TIM_COUNTERDIRECTION_DOWN == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is downcounting */
    }
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  }
  
  /* At this point we are down counting */
  
  /* Clear peripheral flags */
  LL_DMA_ClearFlag_TC(DMAx, pHandle->pParams_str->DMAChannelX);
  LL_DMA_ClearFlag_HT(DMAx, pHandle->pParams_str->DMAChannelX);
  
  LL_DMA_ClearFlag_TC(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
  LL_DMA_ClearFlag_HT(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
  
  LL_TIM_ClearFlag_UPDATE(TIMx);
<#if HAS_ADC_INJ == true>
  LL_ADC_ClearFlag_JEOS(ADCx);
</#if><#-- HAS_ADC_INJ == true -->

  pHandle->TCCnt = 0;
  pHandle->TCDoneFlag = false;
  pHandle->FOCDurationFlag = false;
  
  /* Start DMA that modifies CC register of CH1,2 and 3 in order to create the
     phase shifting */
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH_CCR);
  LL_DMA_Enable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMAChannelX);
  LL_TIM_EnableDMAReq_UPDATE(TIMx);

<#if CondFamily_STM32F4>
  /* Enable DMA related to sampling */
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t) (pHandle->Half_PWMPeriod+1));
  /* Set CCR4 to the very first sampling point
     when CNT matches CCR4 value (first sampling point), it raises CH4 signal:
     - it triggers the first ADC injected conversion
     - it triggers a DMA transfer to load CCR4 with the second sampling value
     which lowers CH4 signal.
     when CNT matches CCR4 value (second sampling point value) it raises CH4 signal
     - it triggers the second ADC injected conversion
     - it triggers a DMA to transfer to load 0 into CCR4 register which makes CH4
       stay high until next PWM cycle
     when CNT matches CCR4 value (0 i.e start of a new PWM cycle) it lowers CH4 signal
     - it triggers a DMA to transfer to load CCR4 with the first sampling point value */
  
  LL_ADC_INJ_SetTriggerSource(ADCx, pHandle->AdcExtTrigger);
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while ((LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET) || (LL_TIM_GetDirection(TIMx) == LL_TIM_COUNTERDIRECTION_DOWN))
  {
    /* TIMx is downcounting */
  }
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, DMA_TRANSFER_LENGTH_SAMPLING_POINT);
  LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
  LL_TIM_EnableDMAReq_CC4(TIMx);
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t) pHandle->CntSmp1);
  LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_RISING);
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);
  LL_DMA_EnableIT_HT(DMAx, pHandle->pParams_str->DMAChannelX);
  LL_ADC_EnableIT_JEOS(ADCx);

<#else><#-- CondFamily_STM32F4 == false -->

  
  /* Enable DMA related to sampling  */
  LL_TIM_EnableDMAReq_CC4(TIMx);
 
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, 3);
  LL_DMA_Enable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
  /* Set CCR4 to the very first sampling point
     when CNT matches CCR4 value (first sampling point), it raises CH4 signal:
     - it triggers the first ADC injected conversion  
     - it triggers a DMA transfer to load CCR4 with the second sampling value
     which lowers CH4 signal. 
     when CNT matches CCR4 value (second sampling point value) it raises CH4 signal
     - it triggers the second ADC injected conversion 
     - it triggers a DMA to transfer to load 0 into CCR4 register which makes CH4 
       stay high until next PWM cycle 
     when CNT matches CCR4 value (0 i.e start of a new PWM cycle) it lowers CH4 signal 
     - it triggers a DMA to transfer to load CCR4 with the first sampling point value */
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t) (pHandle->Half_PWMPeriod+1));

  LL_DMA_Disable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);
  LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX, DMA_TRANSFER_LENGTH_ADC);
  LL_DMA_Enable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);
  LL_ADC_REG_SetSequencerChannels (ADCx, __LL_ADC_DECIMAL_NB_TO_CHANNEL (pHandle->pParams_str->IChannel));
  LL_ADC_SetSamplingTimeCommonChannels (ADC1, pHandle->pParams_str->ISamplingTime);
  LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_LIMITED);
  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_EXT_TIM1_CH4);
  LL_ADC_REG_StartConversion (ADC1);
    
  /* Enable ADC trigger */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_OC4REF);
  
  /* Reset flag for FOC duration detection */
  pHandle->FOCDurationFlag = false;
    
  /* Lock ADC */
  pHandle->ADCRegularLocked=true;

  /* Enable peripheral interrupt */
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);
  <#if HAS_ADC_INJ == false>
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);
  
  LL_TIM_ClearFlag_UPDATE(TIMx);
  while ((LL_TIM_IsActiveFlag_UPDATE(TIMx) == RESET) || (LL_TIM_GetDirection(TIMx) == LL_TIM_COUNTERDIRECTION_DOWN))
  {
    /* TIMx is downcounting */
  }
  LL_TIM_EnableIT_UPDATE(TIM1);
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t) pHandle->CntSmp1);
    
  </#if><#-- HAS_ADC_INJ == false -->
</#if><#-- CondFamily_STM32F4 -->
}


/**
  * @brief  It disables PWM generation on the proper Timer peripheral acting on
  *         MOE bit
  * @param pHdl: handler of the current instance of the PWM component
  */
__weak void R1_SwitchOffPWM(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;
<#if CondFamily_STM32F4>
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
</#if><#-- CondFamily_STM32F4 -->
  pHandle->CntSmp1 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   - (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);
  pHandle->CntSmp2 = ((uint32_t)(pHandle->Half_PWMPeriod) >> 1)
                   + (uint32_t)(pHandle->pParams_str->hTADConv + pHandle->pParams_str->TSample);

  /* This allow to control cycles during next sequence */
  LL_TIM_DisableIT_UPDATE(TIMx);
  
  /* Synchronize the disable of the DMA and all settings with the douwn counting */
  if (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
  {
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  }
  else
  {
    while (LL_TIM_COUNTERDIRECTION_DOWN == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is downcounting */
    }
    while (LL_TIM_COUNTERDIRECTION_UP == LL_TIM_GetDirection(TIMx))
    {
      /* TIMx is upcounting */
    }
  } 

  LL_DMA_DisableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);
  LL_DMA_DisableIT_HT(DMAx, pHandle->pParams_str->DMAChannelX);

<#if CondFamily_STM32F4>
  LL_ADC_INJ_SetTriggerSource(ADCx, LL_ADC_INJ_TRIG_SOFTWARE);
</#if><#-- CondFamily_STM32F4 -->
  pHandle->_Super.TurnOnLowSidesAction = false;

  /* Main PWM Output Disable */
  LL_TIM_DisableAllOutputs(TIMx);
  if (pHandle->_Super.BrakeActionLock == true)
  {
    /* Nothing to do */
  }
  else
  {
    if ((pHandle->_Super.LowSideOutputs) == ES_GPIO)
    {
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_u_port, pHandle->_Super.pwm_en_u_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_v_port, pHandle->_Super.pwm_en_v_pin);
      LL_GPIO_ResetOutputPin(pHandle->_Super.pwm_en_w_port, pHandle->_Super.pwm_en_w_pin);
    }
  } 

<#if CondFamily_STM32F0>
  /* Disable DMA channel related to ADC current sampling */
  LL_DMA_DisableChannel(DMAx, pHandle->pParams_str->DMA_ADC_DR_ChannelX);
  LL_DMA_ClearFlag_GI1(DMAx);
  LL_DMA_ClearFlag_TC1(DMAx);
  LL_DMA_ClearFlag_HT1(DMAx);
  
<#else><#-- CondFamily_STM32F0 == false -->
  LL_ADC_INJ_StopConversionExtTrig(ADCx);
  LL_ADC_INJ_SetTriggerSource(ADCx, LL_ADC_INJ_TRIG_SOFTWARE);
</#if><#-- CondFamily_STM32F0 -->
  /* Disable DMA related to phase shift */
  LL_DMA_Disable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMAChannelX);
<#if CondFamily_STM32F4>
  while (LL_DMA_IsEnabledStream(DMAx, pHandle->pParams_str->DMAChannelX))
  {
    /* Nothing to do */
  }
</#if><#-- CondFamily_STM32F4 -->
  LL_TIM_DisableDMAReq_UPDATE(TIMx);
    
  /* Disable DMA related to sampling */
  LL_DMA_Disable${DMA_TYPE}(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
<#if CondFamily_STM32F4>
  while (LL_DMA_IsEnabledStream(DMAx, pHandle->pParams_str->DMASamplingPtChannelX))
  {
    /* Nothing to do */
  }
</#if><#-- CondFamily_STM32F4 -->
  LL_TIM_DisableDMAReq_CC4(TIMx);

  pHandle->DmaBuffCCR_ADCTrig[0] = pHandle->CntSmp2;
  pHandle->DmaBuffCCR_ADCTrig[2] = pHandle->CntSmp1;
  LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)(pHandle->Half_PWMPeriod + 1));
   
<#if CondFamily_STM32F0>
  /* Disable ADC trigger */
  LL_TIM_SetTriggerOutput(TIMx, LL_TIM_TRGO_RESET);
  
  /* Clear potential ADC Ongoing conversion */
  if (LL_ADC_REG_IsConversionOngoing (ADC1))
  {
    LL_ADC_REG_StopConversion (ADC1);
    while (LL_ADC_REG_IsConversionOngoing(ADC1))
    {
      /* Nothing to do */
    }
  }

  LL_ADC_REG_SetTriggerSource (ADC1, LL_ADC_REG_TRIG_SOFTWARE);

   /* We allow ADC usage for regular conversion on Systick */
  pHandle->ADCRegularLocked=false;
</#if><#-- CondFamily_STM32F0 -->
  
  R1_1ShuntMotorVarsInit(&pHandle->_Super);
}

<#if CondFamily_STM32F3 || CondFamily_STM32G4>
<#if MC.M1_OCP_TOPOLOGY == "EMBEDDED" || MC.M2_OCP_TOPOLOGY == "EMBEDDED" || 
     MC.INTERNAL_OVERVOLTAGEPROTECTION == true || MC.INTERNAL_OVERVOLTAGEPROTECTION2 == true>
/**
  * @brief  It is used to configure the analog output used for protection
  *         thresholds.
  * @param  DAC_Channel: the selected DAC channel.
  *          This parameter can be:
  *            @arg DAC_Channel_1: DAC Channel1 selected
  *            @arg DAC_Channel_2: DAC Channel2 selected
  * @param  hDACVref Value of DAC reference expressed as 16bit unsigned integer.
  *         Ex. 0 = 0V 65536 = VDD_DAC.
  */
void R1_SetAOReferenceVoltage(uint32_t DAC_Channel, uint16_t hDACVref)
{

  LL_DAC_ConvertData12LeftAligned(DAC1, DAC_Channel, hDACVref);
  LL_DAC_TrigSWConversion(DAC1, DAC_Channel);
  /* Enable DAC Channel */
  LL_DAC_Enable(DAC1, DAC_Channel);

}

</#if>
</#if>
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Implementation of the single shunt algorithm to setup the
  *         TIM1 register and DMA buffers values for the next PWM period.
  * @param  pHandle related object of class CPWMC
  * @retval uint16_t It returns MC_DURATION if the TIMx update occurs
  *         before the end of FOC algorithm else returns MC_NO_ERROR
  */
__weak uint16_t R1_CalcDutyCycles(PWMC_Handle_t *pHdl)
{
  PWMC_R1_Handle_t *pHandle = (PWMC_R1_Handle_t *)pHdl;
  DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;
<#if CondFamily_STM32F4>
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef *ADCx = pHandle->pParams_str->ADCx;
</#if><#-- CondFamily_STM32F4 -->
  int16_t submax_mid;
  int16_t submax_mid_deltmin;
  int16_t submid_min;
  int16_t submid_min_deltmin;
  int16_t aCCRval[3];   /* CCR setting values */
  int16_t SamplePoint1;
  int16_t SamplePoint2;
<#if MC.M1_DISCONTINUOUS_PWM == true || MC.M2_DISCONTINUOUS_PWM == true>
  int16_t stored_val;
</#if><#-- MC.M1_DISCONTINUOUS_PWM == true || MC.M2_DISCONTINUOUS_PWM == true -->
  uint16_t hAux;
  uint8_t maxVal;
  uint8_t midVal;
  uint8_t minVal;
  uint8_t max_bad_flag;
  uint8_t min_bad_flag;

  aCCRval[0] = (int16_t)pHandle->_Super.CntPhA;
  aCCRval[1] = (int16_t)pHandle->_Super.CntPhB;
  aCCRval[2] = (int16_t)pHandle->_Super.CntPhC;
  
<#if MC.M1_DISCONTINUOUS_PWM == true || MC.M2_DISCONTINUOUS_PWM == true>
  if(pHandle->_Super.DPWM_Mode == true)
  {
    /* Check if the stator flux localized in boundary regions */
    if ((aCCRval[pHandle->_Super.midDuty] - aCCRval[pHandle->_Super.lowDuty]) <= (int16_t)pHandle->pParams_str->TMin)
    {
      stored_val = (int16_t)pHandle->_Super.HighDutyStored;
      /* Bundary 2 and 3 */
      pHandle->_Super.CntPhA += (uint16_t)stored_val;
      pHandle->_Super.CntPhB += (uint16_t)stored_val;
      pHandle->_Super.CntPhC += (uint16_t)stored_val;
      
      aCCRval[0] = (int16_t)pHandle->_Super.CntPhA;
      aCCRval[1] = (int16_t)pHandle->_Super.CntPhB;
      aCCRval[2] = (int16_t)pHandle->_Super.CntPhC;
    }
    else 
    {
      /* Other cases nothing to do */
    }
  }
  else
  {
    /* DPWM is not used nothing to do */
  }
</#if><#-- MC.M1_DISCONTINUOUS_PWM == true || MC.M2_DISCONTINUOUS_PWM == true -->
  
<#if MC.M1_OVERMODULATION == true>
  /* Sector dividing and setting the value index for max, mid and min */
  if(aCCRval[0] >= aCCRval[1])  /* A >= B */
  {
    if(aCCRval[1] >= aCCRval[2]) /* B >= C */
    {
      /* bSector = 1 */
      maxVal = 0;
      midVal = 1;
      minVal = 2;
    }
    else  /* B < C */
    {
      if(aCCRval[2] >= aCCRval[0])  /* C >= A */
      {
        /* bSector = 5 */
        maxVal = 2;
        midVal = 0;
        minVal = 1;
      }
      else /* C < A */
      {
        /* bSector = 6 */
        maxVal = 0;
        midVal = 2;
        minVal = 1;
      }
    }
    
  }
  else  /* A < B */
  {
    if(aCCRval[1] >= aCCRval[2]) /* B >= C */
    {
       if(aCCRval[2] >= aCCRval[0])  /* C >= A */
      {
        /* bSector = */
        maxVal = 1;
        midVal = 2;
        minVal = 0;
      }
      else /* C < A */
      {
       /* bSector = 2 */
        maxVal = 1;
        midVal = 0;
        minVal = 2;
      }    
    }
    else  /* B < C */
    {
      /* bSector = 4 */
      maxVal = 2;
      midVal = 1;
      minVal = 0;
    }
    
  }
<#else><#-- MC.M1_OVERMODULATION == false -->
  maxVal = (uint8_t)pHandle->_Super.highDuty;
  midVal = (uint8_t)pHandle->_Super.midDuty;
  minVal = (uint8_t)pHandle->_Super.lowDuty;
</#if><#-- MC.M1_OVERMODULATION == true -->
  pHandle->iflag=0x00;


  /* Phase-shift and set iflag */
  submax_mid = aCCRval[maxVal] - aCCRval[midVal];
  submax_mid_deltmin = submax_mid - (int16_t)pHandle->pParams_str->TMin;
  submid_min = aCCRval[midVal] - aCCRval[minVal];
  submid_min_deltmin = submid_min - (int16_t)pHandle->pParams_str->TMin;
  pHandle->aShiftval[0]=0;
  pHandle->aShiftval[1]=0;
  pHandle->aShiftval[2]=0;
  max_bad_flag = 0;
  min_bad_flag = 0;
  
  if(submax_mid_deltmin > 0)
  {
    pHandle->iflag |= ALFLAG[maxVal];
  }
  else
  {
    if ((1 - submax_mid_deltmin + aCCRval[maxVal] + (int16_t) pHandle->pParams_str->hTADConv)
       > (int16_t)(pHandle->Half_PWMPeriod))
    {
      pHandle->iflag &= ~ALFLAG[maxVal];
      max_bad_flag = 1U;
    }
    else
    {
      pHandle->iflag |= ALFLAG[maxVal];
      pHandle->aShiftval[maxVal] = 1U - (uint16_t)submax_mid_deltmin;
    }
  }
  
  if(submid_min_deltmin > 0)
  {
    pHandle->iflag |= ALFLAG[minVal];
  }
  else
  {
    if ((submid_min_deltmin - 1 + aCCRval[minVal]) < 0)
    {
      pHandle->iflag &= ~ALFLAG[minVal];
      min_bad_flag = 1;
    }
    else
    {
      pHandle->iflag |= ALFLAG[minVal];
      pHandle->aShiftval[minVal] = (uint16_t)submid_min_deltmin - 1U;
    }
  }
  
  if ((0U == max_bad_flag) && (0U == min_bad_flag))
  {
    SamplePoint1 = (int16_t)aCCRval[midVal] - (int16_t)pHandle->pParams_str->TSample;
    SamplePoint2 = aCCRval[midVal] + (int16_t)pHandle->pParams_str->TMin - (int16_t)pHandle->pParams_str->TSample;
  }
  else if ((1U == max_bad_flag) && (1U == min_bad_flag))
  {
    SamplePoint1 = (int16_t)pHandle->Half_PWMPeriod / 2;
    SamplePoint2 = SamplePoint1 + (int16_t)pHandle->pParams_str->TSample + (int16_t)pHandle->pParams_str->hTADConv;
  }
  else if (1U == max_bad_flag)
  {
    SamplePoint1 = aCCRval[midVal] - (int16_t)pHandle->pParams_str->TSample;
    SamplePoint2 = aCCRval[midVal] + (int16_t)pHandle->pParams_str->hTADConv;
  }
  else
  {
    SamplePoint2 = aCCRval[midVal] + (int16_t)pHandle->pParams_str->TMin - (int16_t)pHandle->pParams_str->TSample;
    SamplePoint1 = aCCRval[midVal];
  }
  
  if ((SamplePoint2-SamplePoint1) < (int16_t)pHandle->pParams_str->hTADConv)
  {
    pHandle->iflag &=  ALFLAG[maxVal];
    pHandle->iflag &= ~ALFLAG[minVal];  
    SamplePoint1 = (int16_t)pHandle->Half_PWMPeriod /2 ;
    SamplePoint2 = SamplePoint1 + (int16_t)pHandle->pParams_str->TSample + (int16_t)pHandle->pParams_str->hTADConv;
  }
  else
  {
    /* Nothing to do */
  }

  /* Saturate sampling point */
  if ((SamplePoint2 >= (int16_t)(pHandle->Half_PWMPeriod)) || (SamplePoint2 <= 0))
  {
    pHandle->iflag &=  ALFLAG[maxVal];  
    SamplePoint2 = aCCRval[midVal] + (int16_t)pHandle->pParams_str->hTADConv;
  }
  else
  {
    /* Nothing to do */
  }
  if ((SamplePoint1 >= (int16_t)pHandle->Half_PWMPeriod) || (SamplePoint1 <= 0))
  {
    pHandle->iflag &= ~ALFLAG[minVal];  
    SamplePoint1 = aCCRval[midVal];
  }
  else
  {
    /* Nothing to do */
  }

  pHandle->CntSmp1 = SamplePoint1;
  pHandle->CntSmp2 = SamplePoint2;
  
  /* Critical section start */
  LL_DMA_DisableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);
  
  pHandle->DmaBuffCCR_latch[0] = pHandle->_Super.CntPhA + pHandle->aShiftval[0];
  pHandle->DmaBuffCCR_latch[1] = pHandle->_Super.CntPhB + pHandle->aShiftval[1];
  pHandle->DmaBuffCCR_latch[2] = pHandle->_Super.CntPhC + pHandle->aShiftval[2];
  /* Second half PWM period CCR value transfered by DMA */
  pHandle->DmaBuffCCR_latch[3]= pHandle->_Super.CntPhA - pHandle->aShiftval[0];
  pHandle->DmaBuffCCR_latch[4]= pHandle->_Super.CntPhB - pHandle->aShiftval[1];
  pHandle->DmaBuffCCR_latch[5]= pHandle->_Super.CntPhC - pHandle->aShiftval[2];

  if (pHandle->TCDoneFlag == true)
  {
    /* First half PWM period CCR value transfered by DMA */
    pHandle->DmaBuffCCR[0] = pHandle->DmaBuffCCR_latch[0];
    pHandle->DmaBuffCCR[1] = pHandle->DmaBuffCCR_latch[1];
    pHandle->DmaBuffCCR[2] = pHandle->DmaBuffCCR_latch[2];
    /* Second half PWM period CCR value transfered by DMA */
    pHandle->DmaBuffCCR[3]= pHandle->DmaBuffCCR_latch[3];
    pHandle->DmaBuffCCR[4]= pHandle->DmaBuffCCR_latch[4];
    pHandle->DmaBuffCCR[5]= pHandle->DmaBuffCCR_latch[5];
<#if CondFamily_STM32F4>
     
    /* Last DMA TC IT of the PWM cycle occured which re-enabled DMA transfer, this means that a first 
       CCR value (DmaBuffCCR[0] from previous PWM cycle) has been loaded by the DMA as 
       it is configured in direct mode. Then the stream needs to be disabled and enabled to use 
       the correct CCR value */
    LL_DMA_DisableStream(DMAx, pHandle->pParams_str->DMAChannelX);
    while (LL_DMA_IsEnabledStream(DMAx, pHandle->pParams_str->DMAChannelX) == true)
    {
      /* Nothing to do */
    }
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH_CCR);
    LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMAChannelX);

    /* Last DMA TC IT of the PWM cycle occured which re-enabled DMA transfer, this means that the first 
       sampling point value (from previous PWM cycle) has been loaded by the DMA as 
       it is configured in direct mode. Then the stream needs to be disabled and enabled to use 
       the correct sampling point value value */
    LL_DMA_DisableStream(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
    while (LL_DMA_IsEnabledStream(DMAx, pHandle->pParams_str->DMASamplingPtChannelX) == true)
    {
      /* Nothing to do */
    }
    pHandle->DmaBuffCCR_ADCTrig[0] = pHandle->CntSmp2;
    LL_TIM_DisableDMAReq_CC4(TIMx);
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, DMA_TRANSFER_LENGTH_SAMPLING_POINT);
    LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
    LL_TIM_EnableDMAReq_CC4(TIMx);
    LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)pHandle->CntSmp1);
    LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_RISING);
  }
  else
  {
    /* Do nothing, it will be applied during DMA transfer complete IRQ */
  }
  
  /* Critical section end */
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);
<#else><#-- CondFamily_STM32F4 == false -->
    
    /* Reset first sampling as it has already been written by DMA */
    LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t) pHandle->CntSmp1);
  }
  else
  {
    /* Do nothing, it will be applied during DMA transfer complete IRQ */
  }
  /* Critical section end */
  LL_DMA_EnableIT_TC(DMAx, pHandle->pParams_str->DMAChannelX);

  LL_ADC_REG_SetSequencerChannels(ADC1, __LL_ADC_DECIMAL_NB_TO_CHANNEL (pHandle->pParams_str->IChannel));
  LL_ADC_SetSamplingTimeCommonChannels (ADC1, pHandle->pParams_str->ISamplingTime);
  LL_ADC_REG_SetTriggerSource(ADC1, LL_ADC_REG_TRIG_EXT_TIM1_CH4);
  
  pHandle->DmaBuffCCR_ADCTrig[0] = SamplePoint2;
  pHandle->DmaBuffCCR_ADCTrig[2] = SamplePoint1;
</#if><#-- CondFamily_STM32F4 -->
  
  /* Check software error */
  if (pHandle->FOCDurationFlag == true)
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
  else
  {
    /* Nothing to do */
  }
  
  return (hAux);
}


/**
  * @brief  It contains the TIMx Update event interrupt
  * @param pHdl: handler of the current instance of the PWM component
  */
__weak void *R1_TIM1_UP_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
<#if CondFamily_STM32F0 == true>
         
  if (pHandle->TCDoneFlag ==true)
  {
    LL_ADC_REG_StartConversion(ADC1);
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
    pHandle->FOCDurationFlag = true;
    pHandle->TCDoneFlag = false;
  }
  else
  {
    /* Nothing to do */
  }
  
</#if><#--  CondFamily_STM32F0 == true -->
  return (&(pHandle->_Super.Motor));
}


/**
  * @brief  It contains the TIMx Update event interrupt
  * @param pHdl: handler of the current instance of the PWM component
  */
__weak void *R1_TIM8_UP_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
   
  return (&(pHandle->_Super.Motor));
}


/**
  * @brief  This function handles motor DMAx TC interrupt request. 
  *         Required only for R1 with rep rate > 1
  * @param pHdl: handler of the current instance of the PWM component
  */
__weak void *R1_DMAx_TC_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
  DMA_TypeDef *DMAx = pHandle->pParams_str->DMAx;
<#if CondFamily_STM32F4 == true>
  TIM_TypeDef *TIMx = pHandle->pParams_str->TIMx;
</#if><#-- CondFamily_STM32F4 == true -->

  LL_DMA_ClearFlag_HT(DMAx, pHandle->pParams_str->DMAChannelX);
  pHandle->TCCnt++;
  if (pHandle->TCCnt == pHandle->pParams_str->RepetitionCounter)
  {
    /* First half PWM period CCR value transfered by DMA */
    pHandle->DmaBuffCCR[0] = pHandle->DmaBuffCCR_latch[0];
    pHandle->DmaBuffCCR[1] = pHandle->DmaBuffCCR_latch[1];
    pHandle->DmaBuffCCR[2] = pHandle->DmaBuffCCR_latch[2];
    /* Second half PWM period CCR value transfered by DMA */
    pHandle->DmaBuffCCR[3]= pHandle->DmaBuffCCR_latch[3];
    pHandle->DmaBuffCCR[4]= pHandle->DmaBuffCCR_latch[4];
    pHandle->DmaBuffCCR[5]= pHandle->DmaBuffCCR_latch[5];

<#if CondFamily_STM32F4>
    LL_DMA_DisableStream(DMAx, pHandle->pParams_str->DMAChannelX);
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH_CCR);
    LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMAChannelX);
    
    pHandle->DmaBuffCCR_ADCTrig[0] = pHandle->CntSmp2;
    LL_DMA_DisableStream(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
    LL_TIM_DisableDMAReq_CC4(TIMx);
    LL_DMA_ClearFlag_HT(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
    LL_DMA_ClearFlag_TC(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMASamplingPtChannelX, DMA_TRANSFER_LENGTH_SAMPLING_POINT);
    LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMASamplingPtChannelX);
    LL_TIM_EnableDMAReq_CC4(TIMx);
    
</#if><#-- CondFamily_STM32F4 -->
    pHandle->TCCnt = 0;
    pHandle->TCDoneFlag =true;
  }
  else
  {
<#if CondFamily_STM32F4>
    LL_DMA_DisableStream(DMAx, pHandle->pParams_str->DMAChannelX);
    LL_DMA_ClearFlag_HT(DMAx, pHandle->pParams_str->DMAChannelX);
    LL_DMA_SetDataLength(DMAx, pHandle->pParams_str->DMAChannelX, DMA_TRANSFER_LENGTH_CCR);
    LL_DMA_EnableStream(DMAx, pHandle->pParams_str->DMAChannelX);
</#if><#-- CondFamily_STM32F4 -->
  }

  return (&(pHandle->_Super.Motor));
}

__weak void *R1_DMAx_HT_IRQHandler(PWMC_R1_Handle_t *pHandle)
{
<#if CondFamily_STM32F4>
  TIM_TypeDef * TIMx = pHandle->pParams_str->TIMx;
  ADC_TypeDef * ADCx = pHandle->pParams_str->ADCx;
  
  if (pHandle->TCDoneFlag ==true)
  {      
    LL_TIM_OC_SetCompareCH4(TIMx, (uint32_t)pHandle->CntSmp1);
    LL_ADC_INJ_StartConversionExtTrig(ADCx, LL_ADC_INJ_TRIG_EXT_RISING);
    
    pHandle->FOCDurationFlag = true;
    pHandle->TCDoneFlag = false;
  }
  else
  {
    /* Nothing to do */
  }
 </#if><#-- CondFamily_STM32F4 -->
  return (&(pHandle->_Super.Motor));

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

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
