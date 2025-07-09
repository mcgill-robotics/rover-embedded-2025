/*
 * encoder_conf.h
 *
 *  Created on: Jul 5, 2025
 *      Author: vince
 */

#ifndef INC_ENCODER_CONF_H_
#define INC_ENCODER_CONF_H_

#include "stm32f4xx_hal.h"

struct MotorEncoder {
	TIM_HandleTypeDef *htim;
	volatile int32_t count;
	volatile int32_t min_limit_counts;
	volatile int32_t max_limit_counts;
	volatile int32_t total_counts_range;
	volatile float angle_range;
	volatile uint8_t is_homed;
	volatile float angle;

	GPIO_TypeDef* lower_limit_port;
	volatile uint8_t lower_limit_pin;
	GPIO_TypeDef* upper_limit_port;
	volatile uint16_t upper_limit_pin;
	volatile uint8_t revolute; // if revolute joint, only use lower limit
	volatile uint8_t lower_limit_active;
	volatile uint8_t upper_limit_active;
};

void MotorEncoder_Init(struct MotorEncoder* menc);

int32_t ReadEncoderCounts(TIM_HandleTypeDef *htim);

void NewPosition(struct MotorEncoder* menc);

void HandleUpperLimit(struct MotorEncoder* menc);

void HandleLowerLimit(struct MotorEncoder* menc);

#endif /* INC_ENCODER_CONF_H_ */
