#include "main.h"
#include "encoder.h"
#include "pid.h"
#include "motor.h"
#include "CAN_processing.h"
#include "math.h"

int direction = 1;
int power_limit = 4499;

// Change based on what motor is being controlled!
int STEERING_ID = RF_STEER;

void stop_motor(){
	set_motor_speed_raw(0);
	int counts = get_counts();
	setPIDGoalA(count_to_angle(counts));
}

void set_motor_speed_percent(float n){
	set_motor_speed_raw(power_limit*(n/100.0f));
}

void set_motor_speed_raw(int n){
	n = abs(n);
	if (n > power_limit){
		n = power_limit;
	}
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
