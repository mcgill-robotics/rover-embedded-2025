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
#define CO2_SDA_Pin GPIO_PIN_0
#define CO2_SDA_GPIO_Port GPIOF
#define MOIS3_Pin GPIO_PIN_1
#define MOIS3_GPIO_Port GPIOF
#define MOIS4_Pin GPIO_PIN_0
#define MOIS4_GPIO_Port GPIOC
#define MOIS1_Pin GPIO_PIN_2
#define MOIS1_GPIO_Port GPIOC
#define MOIS2_Pin GPIO_PIN_3
#define MOIS2_GPIO_Port GPIOC
#define PH1_Pin GPIO_PIN_1
#define PH1_GPIO_Port GPIOA
#define PH2_Pin GPIO_PIN_2
#define PH2_GPIO_Port GPIOA
#define PH3_Pin GPIO_PIN_3
#define PH3_GPIO_Port GPIOA
#define PH4_Pin GPIO_PIN_4
#define PH4_GPIO_Port GPIOA
#define SER5_Pin GPIO_PIN_5
#define SER5_GPIO_Port GPIOA
#define SERV4_Pin GPIO_PIN_6
#define SERV4_GPIO_Port GPIOA
#define SERV3_Pin GPIO_PIN_7
#define SERV3_GPIO_Port GPIOA
#define CO2_SCL_Pin GPIO_PIN_4
#define CO2_SCL_GPIO_Port GPIOC
#define SER2_Pin GPIO_PIN_1
#define SER2_GPIO_Port GPIOB
#define SER1_Pin GPIO_PIN_2
#define SER1_GPIO_Port GPIOB
#define ToF_SCL_Pin GPIO_PIN_6
#define ToF_SCL_GPIO_Port GPIOC
#define ToF_SDA_Pin GPIO_PIN_7
#define ToF_SDA_GPIO_Port GPIOC
#define USR_LED_Pin GPIO_PIN_9
#define USR_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
