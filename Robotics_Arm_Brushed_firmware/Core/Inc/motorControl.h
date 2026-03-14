#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdlib.h>

extern TIM_TypeDef* GRIPPER_PWM_TIMER;
extern TIM_TypeDef* PITCH_PWM_TIMER;
extern TIM_TypeDef* ROLL_PWM_TIMER;



// Struct for motor
typedef struct {
	TIM_TypeDef* 	PWM_type;
	TIM_TypeDef* 	ENCODER_type;
	MotorID			motorID;
} Motor;

extern Motor gripper_motor;
extern Motor pitch_motor;
extern Motor roll_motor;

void motor_struct_init(Motor * motor, TIM_TypeDef * pwm, TIM_TypeDef * encoder, MotorID motorID);


void stop_motor();
void set_motor_speed_percent(Motor * motor, float n);
void set_motor_speed_raw(Motor * motor, int n);
void set_motor_direction(int n);


#endif
