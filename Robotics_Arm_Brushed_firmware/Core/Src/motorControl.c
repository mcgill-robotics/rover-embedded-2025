/*
 *
 * This file contains motor control functions for the arm brushed motors.
 *
 */


#include <stdint.h>
#include <stdbool.h>


#include "CAN_processing.h"
#include "pid.h"
#include "encoder.h"
#include "motorControl.h"
#include "main.h"

//#include "uart_debugging.h"




int direction = 1;
int power_limit = 4499;

// Change based on what motor is being controlled!
//int STEERING_ID = RF_STEER;

//int GRIPPER_ID = GRIPPER;
//int PITCH_ID = PITCH;
//int ROLL_ID = ROLL;


void motor_struct_init(Motor* motor, TIM_TypeDef * pwm, TIM_TypeDef * encoder, MotorID motorID){
	motor->motorID = motorID;
	motor->ENCODER_type = encoder;
	motor->PWM_type = pwm;
}


void stop_motor(Motor * motor){
	set_motor_speed_raw(motor, 0);
	int counts = get_counts();
	setPIDGoalA(count_to_angle(counts));
}

void set_motor_speed_percent(Motor * motor, float n){
	set_motor_speed_raw(motor, power_limit*(n/100.0f));
}

void set_motor_speed_raw(Motor * motor, int n){
	n = abs(n);
	if (n > power_limit){
		n = power_limit;
	}
	motor->PWM_type->CCR1 = n;
}


void set_motor_direction(int n){
	if (n) {
		//HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);
	}
	else {
		//HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_SET);
	}
	direction = n;
}
