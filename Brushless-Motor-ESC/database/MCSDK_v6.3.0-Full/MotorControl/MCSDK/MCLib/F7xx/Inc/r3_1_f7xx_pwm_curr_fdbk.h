/**
  ******************************************************************************
  * @file    r3_1_f7xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_1_F7XX_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup R3_1_F7XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_1_F7XX_PWMCURRFDBK_H
#define __R3_1_F7XX_PWMCURRFDBK_H


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

#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/* Exported types ------------------------------------------------------- */

/*
  * R3_1_F7XX_pwm_curr_fdbk component parameters definition.
  */
typedef const struct
{
  TIM_TypeDef * TIMx;                       /* Timer used for PWM generation. */
  ADC_TypeDef * ADCx;                       /* ADC peripheral to be used. */
  uint32_t ADCConfig[6] ;                  /* Stores ADC sequence for the 6 sectors. */
  uint16_t Tafter;                          /* Sum of dead time plus max
                                                 value between rise time and noise time
                                                 expressed in number of TIM clocks. */
  uint16_t Tbefore;                         /* Sampling time expressed in
                                                 number of TIM clocks.*/
  uint16_t Tsampling;                       /* Sampling time expressed in
                                                 number of TIM clocks.*/
  uint16_t Tcase2;                          /* Sampling time expressed in
                                                 number of TIM clocks.*/
  uint16_t Tcase3;                          /* Sampling time expressed in
                                                 number of TIM clocks.*/

  uint8_t  RepetitionCounter;               /* Expresses the number of PWM
                                                 periods to be elapsed before compare
                                                 registers are updated again. In
                                                 particular:
                                                 RepetitionCounter= (2* #PWM periods)-1*/

} R3_1_Params_t;


/*
  * @brief  This structure is used to handle an instance of the
  *         R3_1_F7XX_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /* Base component handler. */
  uint32_t PhaseAOffset;    /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;    /* Offset of Phase B current sensing network. */
  uint32_t PhaseCOffset;    /* Offset of Phase C current sensing network. */
  uint32_t ADC_ExternalTriggerInjected;  /* External ADC trigger source. */
  uint32_t ADCTriggerEdge;               /* External ADC trigger edge. */
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts. */
  uint8_t  CalibSector;     /* Space vector sector number during calibration. */
  volatile uint8_t PolarizationCounter; /* Number of conversions performed during the calibration phase. */
  R3_1_Params_t const *pParams_str;

} PWMC_R3_1_Handle_t;


/* Exported functions ------------------------------------------------------- */

/*  Initializes peripherals for current reading and PWM generation
 *  in three shunts configuration using STM32F302x8 *****/
void R3_1_Init( PWMC_R3_1_Handle_t * pHandle );

/* Disables PWM generation on the proper Timer peripheral acting on MOE bit.
 */
void R3_1_SwitchOffPWM( PWMC_Handle_t * pHdl );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_1_SwitchOnPWM( PWMC_Handle_t * pHdl );

/*
  * Turns on low sides switches.
  */
void R3_1_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void R3_1_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format. Specific to overmodulation.
  */
void R3_1_GetPhaseCurrents_OVM( PWMC_Handle_t * pHdl, ab_t * pStator_Currents );

/*
  * Stores into handler component variables the offset voltage on Ia and
  * Ib current feedback analog channels when no current is flowing into the
  * motor.
  */
void R3_1_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/*
  * Configures the ADC for the current sampling during calibration.
  */
uint16_t R3_1_SetADCSampPointCalibration( PWMC_Handle_t * pHdl);

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  */
uint16_t R3_1_SetADCSampPointSectX( PWMC_Handle_t * pHdl);

/*
  * Configures the ADC for the current sampling related to sector (X = [1..6] ) in case of overmodulation.
  */
uint16_t R3_1_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHdl);

/*
  * Contains the TIMx Update event interrupt.
  */
void * R3_1_TIMx_UP_IRQHandler( PWMC_R3_1_Handle_t * pHdl );

/*
  * Sets the calibrated offset.
  */
void R3_1_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets.
  */
void R3_1_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Sets the PWM mode for R/L detection.
 */
void R3_1_RLDetectionModeEnable( PWMC_Handle_t * pHdl );

/*
  * Disables the PWM mode for R/L detection.
  */
void R3_1_RLDetectionModeDisable( PWMC_Handle_t * pHdl );

/*
  * Sets the PWM dutycycle for R/L detection.
  */
uint16_t R3_1_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty );

/*
 * Turns on low sides switches and start ADC triggering.
 */
void R3_1_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl );

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

#endif /*__R3_1_F7XX_PWMNCURRFDBK_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
