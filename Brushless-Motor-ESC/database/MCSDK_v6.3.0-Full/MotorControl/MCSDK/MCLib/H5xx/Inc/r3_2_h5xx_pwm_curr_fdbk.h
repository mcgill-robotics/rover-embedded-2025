/**
  ******************************************************************************
  * @file    r3_2_h5xx_pwm_curr_fdbk.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          R3_2_H5XX_pwm_curr_fdbk component of the Motor Control SDK.
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
  * @ingroup R3_2_H5XX_pwm_curr_fdbk
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __R3_2_PWM_CURR_FDBK_H
#define __R3_2_PWM_CURR_FDBK_H

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

/** @addtogroup R3_2_pwm_curr_fdbk
  * @{
  */

#define NONE    ((uint8_t)(0x00))
#define EXT_MODE  ((uint8_t)(0x01))
#define INT_MODE  ((uint8_t)(0x02))

/* Exported types ------------------------------------------------------- */
/*
  * @brief  R3_2_H5XX_pwm_curr_fdbk component parameters definition
  */
typedef const struct
{
  TIM_TypeDef * TIMx;                   /* Contains the pointer to the timer
                                            used for PWM generation. It must
                                            equal to TIM1 if M1, to TIM8 otherwise */
  ADC_TypeDef *ADCDataReg1[6];
  ADC_TypeDef *ADCDataReg2[6];
  uint32_t ADCConfig1[6];           /* Values of JSQR for first ADC for 6 sectors. */ 
  uint32_t ADCConfig2[6];           /* Values of JSQR for second ADC for 6 sectors. */ 


/* PWM generation parameters --------------------------------------------------*/

  uint16_t Tw;                    /*!< Used for switching the context
                                       in dual MC. It contains biggest delay
                                       (expressed in counter ticks) between
                                       the counter crest and ADC latest trigger. Specific to F4XX and F7XX.
                                       */
  uint16_t hTafter;                  /*!< Sum of dead time plus max
                                          value between rise time and noise time
                                          express in number of TIM clocks. Specific to F4XX and F7XX.*/
  uint16_t hTbefore;                   /* Sampling time expressed in
                                          number of TIM clocks.*/
  uint16_t hDeadTime;                /*!< Dead time in number of TIM clock
                                          cycles. If CHxN are enabled, it must
                                          contain the dead time to be generated
                                          by the microcontroller, otherwise it
                                          expresses the maximum dead time
                                          generated by driving network. Specific to F4XX and F7XX. */

  uint16_t Tsampling;                 /* Sampling time expressed in
                                          number of TIM clocks.*/
  uint16_t Tbefore;                   /* Sampling time expressed in
                                          number of TIM clocks.*/
  uint16_t Tcase2;                    /* Sampling time expressed in
                                          number of TIM clocks.*/
  uint16_t Tcase3;                    /* Sampling time expressed in
                                          number of TIM clocks.*/

  /* Dual MC parameters --------------------------------------------------------*/
  uint8_t  bFreqRatio;              /* Used in case of dual MC to
                                        synchronize TIM1 and TIM8. It has
                                        effect only on the second instanced
                                        object and must be equal to the
                                        ratio between the two PWM frequencies
                                        (higher/lower). Supported values are
                                        1, 2 or 3 */
  uint8_t  bIsHigherFreqTim;        /* When bFreqRatio is greater than 1
                                        this param is used to indicate if this
                                        instance is the one with the highest
                                        frequency. Allowed values are: HIGHER_FREQ
                                        or LOWER_FREQ */

/* PWM Driving signals initialization ----------------------------------------*/
  uint8_t  RepetitionCounter;           /* Expresses the number of PWM
                                            periods to be elapsed before compare
                                            registers are updated again.
                                            In particular:
                                            RepetitionCounter= (2* #PWM periods)-1*/

} R3_2_Params_t;

/*
  * @brief  Handles an instance of the R3_2_H5XX_pwm_curr_fdbk component.
  */
typedef struct
{
  PWMC_Handle_t _Super;     /* Base component handler   */
  uint32_t PhaseAOffset;    /* Offset of Phase A current sensing network. */
  uint32_t PhaseBOffset;    /* Offset of Phase B current sensing network. */
  uint32_t PhaseCOffset;    /* Offset of Phase C current sensing network. */
  uint16_t ADC_ExternalPolarityInjected; /* External trigger selection for ADC peripheral. */
  uint32_t ADCTriggerEdge;  /*!< Trigger edge selection. Specific to F4XX, F7XX and L4XX. */
  uint16_t Half_PWMPeriod;  /* Half PWM Period in timer clock counts. */
  uint8_t  CalibSector;     /*!< Space vector sector number during calibration. Specific to F4XX, F7XX and L4XX. */
  volatile uint8_t PolarizationCounter;
  uint8_t PolarizationSector; /* Sector selected during calibration phase. */
  R3_2_Params_t const *pParams_str;
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
void R3_2_CurrentReadingCalibration( PWMC_Handle_t * pHandle );

/*
  * Stores into the handle the voltage present on Ia and
  * Ib current feedback analog channels when no current is flowing into the
  * motor.
  */
void R3_2_CurrentReadingPolarization(PWMC_Handle_t *pHdl);

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format.
  */
void R3_2_GetPhaseCurrents( PWMC_Handle_t * pHandle,ab_t* Iab);

/*
  * Computes and stores in the handler the latest converted motor phase currents in ab_t format. Specific to overmodulation.
  */
void R3_2_GetPhaseCurrents_OVM( PWMC_Handle_t * pHandle,ab_t* Iab);

/*
  * Turns on low sides switches.
  */
void R3_2_TurnOnLowSides( PWMC_Handle_t * pHandle, uint32_t ticks );

/*
  * Enables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOnPWM( PWMC_Handle_t * pHandle );

/*
  * Disables PWM generation on the proper Timer peripheral acting on MOE bit.
  */
void R3_2_SwitchOffPWM( PWMC_Handle_t * pHandle );

/* 
  * Configures the ADC for the current sampling during calibration.
  */
uint16_t R3_2_SetADCSampPointCalibration( PWMC_Handle_t * pHandle );

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ).
  */
uint16_t R3_2_SetADCSampPointSectX( PWMC_Handle_t * pHandle );

/*
  * Configures the ADC for the current sampling related to sector X (X = [1..6] ) in case of overmodulation.
  */
uint16_t R3_2_SetADCSampPointSectX_OVM( PWMC_Handle_t * pHandle );

/*
  *  Contains the TIMx Update event interrupt.
  */
void *R3_2_TIMx_UP_IRQHandler( PWMC_R3_2_Handle_t * pHandle);

/*
  * Sets the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeEnable( PWMC_Handle_t * pHandle  );

/*
  * Disables the PWM mode for R/L detection.
  */
void R3_2_RLDetectionModeDisable( PWMC_Handle_t * pHandle  );

/*
  * Sets the PWM dutycycle for R/L detection.
  */
uint16_t R3_2_RLDetectionModeSetDuty( PWMC_Handle_t * pHandle , uint16_t hDuty );

/*
 *  Turns on low sides switches and start ADC triggering.
 */
void R3_2_RLTurnOnLowSidesAndStart( PWMC_Handle_t * pHandle  );

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

#endif /*__R3_2_PWM_CURR_FDBK_H*/

/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/