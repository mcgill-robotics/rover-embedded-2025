/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motorControl.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */


/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */



/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAULT_pitch_Pin GPIO_PIN_13
#define FAULT_pitch_GPIO_Port GPIOC
#define FAULT_pitch_EXTI_IRQn EXTI15_10_IRQn
#define FAULT_roll_Pin GPIO_PIN_14
#define FAULT_roll_GPIO_Port GPIOC
#define FAULT_roll_EXTI_IRQn EXTI15_10_IRQn
#define FAULT_gripper_Pin GPIO_PIN_15
#define FAULT_gripper_GPIO_Port GPIOC
#define PWM_pitch_Pin GPIO_PIN_0
#define PWM_pitch_GPIO_Port GPIOC
#define ADC_pitch_Pin GPIO_PIN_1
#define ADC_pitch_GPIO_Port GPIOC
#define ADC_roll_Pin GPIO_PIN_2
#define ADC_roll_GPIO_Port GPIOC
#define ADC_gripper_Pin GPIO_PIN_3
#define ADC_gripper_GPIO_Port GPIOC
#define Encoder_A_pitch_Pin GPIO_PIN_0
#define Encoder_A_pitch_GPIO_Port GPIOA
#define Encoder_B_pitch_Pin GPIO_PIN_1
#define Encoder_B_pitch_GPIO_Port GPIOA
#define DIR_pitch_Pin GPIO_PIN_2
#define DIR_pitch_GPIO_Port GPIOA
#define LED_pitch_Pin GPIO_PIN_3
#define LED_pitch_GPIO_Port GPIOA
#define LED_roll_Pin GPIO_PIN_4
#define LED_roll_GPIO_Port GPIOA
#define LED_gripper_Pin GPIO_PIN_5
#define LED_gripper_GPIO_Port GPIOA
#define Encoder_A_gripper_Pin GPIO_PIN_6
#define Encoder_A_gripper_GPIO_Port GPIOA
#define Encoder_B_gripper_Pin GPIO_PIN_7
#define Encoder_B_gripper_GPIO_Port GPIOA
#define DIR_gripper_Pin GPIO_PIN_1
#define DIR_gripper_GPIO_Port GPIOB
#define PWM_gripper_Pin GPIO_PIN_2
#define PWM_gripper_GPIO_Port GPIOB
#define ToF_int_Pin GPIO_PIN_15
#define ToF_int_GPIO_Port GPIOB
#define ToF_int_EXTI_IRQn EXTI15_10_IRQn
#define PWM_roll_Pin GPIO_PIN_6
#define PWM_roll_GPIO_Port GPIOC
#define DIR_roll_Pin GPIO_PIN_7
#define DIR_roll_GPIO_Port GPIOC
#define LED_comms_Pin GPIO_PIN_8
#define LED_comms_GPIO_Port GPIOC
#define VBUS_sense_Pin GPIO_PIN_9
#define VBUS_sense_GPIO_Port GPIOA
#define Lazer_ctrl_Pin GPIO_PIN_10
#define Lazer_ctrl_GPIO_Port GPIOA
#define Limit_switch_6_Pin GPIO_PIN_10
#define Limit_switch_6_GPIO_Port GPIOC
#define Limit_switch_6_EXTI_IRQn EXTI15_10_IRQn
#define Limit_switch_5_Pin GPIO_PIN_11
#define Limit_switch_5_GPIO_Port GPIOC
#define Limit_switch_5_EXTI_IRQn EXTI15_10_IRQn
#define Limit_switch_4_Pin GPIO_PIN_12
#define Limit_switch_4_GPIO_Port GPIOC
#define Limit_switch_4_EXTI_IRQn EXTI15_10_IRQn
#define Limit_switch_3_Pin GPIO_PIN_2
#define Limit_switch_3_GPIO_Port GPIOD
#define Limit_switch_3_EXTI_IRQn EXTI2_IRQn
#define Limit_switch_2_Pin GPIO_PIN_4
#define Limit_switch_2_GPIO_Port GPIOB
#define Limit_switch_2_EXTI_IRQn EXTI4_IRQn
#define Limit_switch_1_Pin GPIO_PIN_5
#define Limit_switch_1_GPIO_Port GPIOB
#define Limit_switch_1_EXTI_IRQn EXTI9_5_IRQn
#define Encoder_A_roll_Pin GPIO_PIN_6
#define Encoder_A_roll_GPIO_Port GPIOB
#define Encoder_B_roll_Pin GPIO_PIN_7
#define Encoder_B_roll_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define NB_MOTORS 3
extern Motor * all_motors_list[NB_MOTORS];

//extern volatile uint8_t systick_10ms_flag;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
