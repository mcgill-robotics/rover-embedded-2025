/**
  ******************************************************************************
  * @file    r3_2_l4xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          r3_4_f30x_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup R3_2_L4XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_2_L4XX_PWMNCURRFDBK_H
#define __R3_2_L4XX_PWMNCURRFDBK_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "pwm_curr_fdbk.h"

/* Exported defines --------------------------------------------------------*/
#define GPIO_NoRemap_TIM1 ((uint32_t)(0))
#define SHIFTED_TIMs      ((uint8_t) 1)
#define NO_SHIFTED_TIMs   ((uint8_t) 0)

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup pwm_curr_fdbk
  * @{
  */

/** @addtogroup R3_2_pwm_curr_fdbk
  * @{
  */

/* Exported types ------------------------------------------------------- */

/*
  * @brief  R3_2_L4XX_pwm_curr_fdbk component parameters definition
  */
typedef const struct
{
  TIM_TypeDef * TIMx;              /* Timer used for PWM generation.*/
  uint32_t ADCConfig1[6];                 /* Values of JSQR for first ADC for 6 sectors. */
  uint32_t ADCConfig2[6];                 /* Values of JSQR for second ADC for 6 sectors. */
  ADC_TypeDef * ADCDataReg1[6];  /* Contains the Address of ADC read value for one phase
                                             and all the 6 sectors. */
  ADC_TypeDef * ADCDataReg2[6];  /* Contains the Address of ADC read value for one phase
                                             and all the 6 sectors. */

  /* PWM generation parameters --------------------------------------------------*/  

  uint16_t hTafter;                       /* Sum of dead time plus max
                                             value between rise time and noise time
                                             express in number of TIM clocks.*/
  uint16_t Tsampling;                     /* Sampling time express in
                                             number of TIM clocks.*/
  uint16_t hTbefore;                      /* Sampling time express in
                                             number of TIM clocks.*/
  uint16_t Tcase2;                        /* Sampling time express in
                                             number of TIM clocks.*/
  uint16_t Tcase3;                        /* Sampling time express in
                                             number of TIM clocks.*/
  
  /* DAC settings --------------------------------------------------------------*/
  uint16_t hDAC_OCP_Threshold;           /* Value of analog reference expressed
                                            as 16bit unsigned integer.
                                            Ex. 0 = 0V ; 65536 = VDD_DAC.*/
  uint16_t hDAC_OVP_Threshold;           /* Value of analog reference expressed
                                            as 16bit unsigned integer.
                                            Ex. 0 = 0V ; 65536 = VDD_DAC.*/

  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  bFreqRatio;                     /* Used in case of dual MC to
                                              synchronize TIM1 and TIM8. It has
                                              effect only on the second instanced
                                              object and must be equal to the
                                              ratio between the two PWM frequencies
                                              (higher/lower). Supported values are
                                              1, 2 or 3 */
  uint8_t  bIsHigherFreqTim;               /* When bFreqRatio is greater than 1
                                              this param is used to indicate if this
                                              instance is the one with the highest
                                              frequency. Allowed value are: HIGHER_FREQ
                                              or LOWER_FREQ */

  /* PWM Driving signals initialization ----------------------------------------*/
  uint8_t  RepetitionCounter;              /* It expresses the number of PWM
                                              periods to be elapsed before compare
                                              registers are updated again. In
                                              particular:
                                              RepetitionCounter= (2* #PWM periods)-1*/
  
} R3_2_Params_t;

/*
  * @brief  Handles an instance of the R3_2_L4XX_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;    /* Base component handler. */
  uint32_t PhaseAOffset;   /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;   /* Offset of Phase B current sensing network. */
  uint32_t PhaseCOffset;   /* Offset of Phase C current sensing network. */
  uint32_t ADC_ExternalTriggerInjected;   /* External trigger selection for ADC peripheral.*/
  volatile uint32_t ADCTriggerEdge;       /* Trigger edge selection for ADC peripheral. Specific to F4XX, F7XX and L4XX.*/
  uint16_t Half_PWMPeriod;    /* Half PWM Period in timer clock counts. */
  uint8_t  CalibSector;       /* Space vector sector number during calibration. Specific to F4XX, F7XX and L4XX. */
  volatile uint8_t PolarizationCounter; /* Number of conversions performed during the calibration phase. */
  R3_2_Params_t *pParams_str;
} PWMC_R3_2_Handle_t;

/* Exported functions ------------------------------------------------------- */

/*
  * Initializes TIMx, ADC, GPIO, DMA1 and NVIC for current reading
  * in three shunt topology using STM32F30X and two ADCs.
  */
void R3_2_Init( PWMC_R3_2_Handle_t * pHandle );

/*
  * Stores into the handler the voltage present on Ia and Ib current
  * feedback analog channels when no current is flowing into the motor.
  */
void R3_2_CurrentReadingCalibration( PWMC_Handle_t * pHdl );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void R3_2_GetPhaseCurrents( PWMC_Handle_t * pHdl, ab_t * Iab );

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format. Specific to overmodulation.
  */
void R3_2_GetPhaseCurrents_OVM( PWMC_Handle_t * pHdl, ab_t * Iab );

/*
  * Turns on low sides switches.
  */
void R3_2_TurnOnLowSides( PWMC_Handle_t * pHdl, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOnPWM( PWMC_Handle_t * pHdl );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOffPWM( PWMC_Handle_t * pHdl );

/*
  * Configures the ADC for the current sampling during calibration.
  */
uint16_t R3_2_SetADCSampPointCalibration( PWMC_Handle_t * pHdl);

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  */
uint16_t R3_2_SetADCSampPointSectX( PWMC_Handle_t * pHdl);

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ) in case of overmodulation.
  */
uint16_t R3_2_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHdl);

/*
  * Contains the TIMx Update event interrupt.
  */
void * R3_2_TIMx_UP_IRQHandler( PWMC_R3_2_Handle_t * pHdl );

/*
  * Set the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeEnable( PWMC_Handle_t * pHdl );

/*
  * Disables the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeDisable( PWMC_Handle_t * pHdl );

/*
  * Set the PWM dutycycle for R/L detection.
  */
uint16_t R3_2_RLDetectionModeSetDuty( PWMC_Handle_t * pHdl, uint16_t hDuty );

/*
 * Turns on low sides switches and start ADC triggering.
 */
void R3_2_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHdl );

/*
 * Sets ADC sampling points.
 */
void RLSetADCSampPoint( PWMC_Handle_t * pHdl );

/*
  * Stores in the handler the calibrated offsets.
  */
void R3_2_SetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

/*
  * Reads the calibrated offsets stored in the handler.
  */
void R3_2_GetOffsetCalib(PWMC_Handle_t *pHdl, PolarizationOffsets_t *offsets);

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

#endif /*__R3_2_L4XX_PWMNCURRFDBK_H*/

/************************ (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
