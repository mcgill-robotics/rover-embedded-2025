#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

void set_motor_speed(int n){
	TIM8->CCR2 = n;
}
void set_motor_direction(int n){

	if (n == 1) {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}
}
#endif
