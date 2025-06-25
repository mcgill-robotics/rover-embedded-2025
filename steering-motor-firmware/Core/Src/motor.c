#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "encoder.h"
#include "pid.h"

int direction = 1;
int actual_direction = 1;
//0 ccw
//1 cw

void stop_motor(){
	int counts = get_counts();
	setPIDGoalA(counts);
	set_motor_speed(0);
}

void set_motor_speed(int n){
	TIM8->CCR1 = n;
}

int get_actual_motor_direction(){
	return actual_direction;
}

void set_actual_motor_direction(int n){
	actual_direction = n;
}


void set_motor_direction(int n){
	/*for (int i = 100 ; i>0 ; i--) {
				set_motor_speed(i);
				delay_us(100);
	}*/
	if (n) {
//		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
	}
	else {
//		HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
//		HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
	}
	/*for (int i = 0 ; i<100 ; i++) {
					set_motor_speed(i);
					delay_us(100);
	}*/
	direction = n;
}
#endif
