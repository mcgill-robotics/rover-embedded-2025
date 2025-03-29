#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

void set_motor_speed(int n){
	TIM8->CCR2 = n;
}
void set_motor_direction(int n){

	/*for (int i = 100 ; i>0 ; i--) {
				set_motor_speed(i);
				delay_us(100);
	}*/
	if (n == 1) {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
	}
	/*for (int i = 0 ; i<100 ; i++) {
					set_motor_speed(i);
					delay_us(100);
	}*/
}
#endif
