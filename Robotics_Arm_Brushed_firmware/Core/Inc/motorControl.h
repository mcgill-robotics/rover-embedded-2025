#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdlib.h>
#include "CAN_processing.h"
#include "encoder.h"

extern TIM_TypeDef* GRIPPER_PWM_TIMER;
extern TIM_TypeDef* PITCH_PWM_TIMER;
extern TIM_TypeDef* ROLL_PWM_TIMER;


// Struct for motor
typedef struct {
	TIM_TypeDef* 			PWM_type;
	TIM_TypeDef* 			ENCODER_type;
	Motor_Encoding_Struct*	Motor_Encoding_Struct;
	MotorID					motorID;
	GPIO_TypeDef* 			DIR_port;
	uint16_t				DIR_pin;


} Motor;

extern Motor gripper_motor;
extern Motor pitch_motor;
extern Motor roll_motor;

void motor_struct_init(Motor * motor, TIM_TypeDef * pwm,
		TIM_TypeDef * encoder, Motor_Encoding_Struct * motor_encoding_stuct, MotorID motorID, GPIO_TypeDef* DIR_port,
		uint16_t DIR_pin);


void stop_motor();
void set_motor_speed_percent(Motor * motor, float n);
void set_motor_speed_raw(Motor * motor, int n);
void set_motor_direction(Motor * motor, int n);


#endif
