#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "encoder.h"
#include "pid.h"

int direction = 1;

void stop_motor(){
	set_motor_speed(0);
	int counts = get_counts();
	setPIDGoalA(count_to_angle(counts));
}

void set_motor_speed(int n){
	TIM8->CCR1 = n;
}

void set_motor_direction(int n){
	if (n) {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}
	else {
		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}
	direction = n;
}
#endif
