#include "main.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "CAN_processing.h"

int direction = 1;

// Change based on what motor is being controlled!
int STEERING_ID = RF_STEER;

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
