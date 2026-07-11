#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <stdlib.h>
#include "stm32g4xx_hal.h"
//#include "main.h"
//#include "CAN_processing.h"
//#include "encoder.h"

extern TIM_TypeDef* GRIPPER_PWM_TIMER;
extern TIM_TypeDef* PITCH_PWM_TIMER;
extern TIM_TypeDef* ROLL_PWM_TIMER;


typedef enum {
    PID = 0,
    CALIBRATION = 1,
	LEAVE_LIMIT = 2
} SteeringState;

typedef enum {
	GRIPPER = 0,
	PITCH = 1,
	ROLL = 2
} MotorName;

typedef struct {
	int 			ENCODER_MAX_COUNTS;
	int 			MAX_ROTATION_ANGLE;
	int				MIN_ROTATION_ANGLE;
	int 			nb_full_rotations;
	int				curr_counts;
	int 			need_debounce;
	int				oldAngleError;

} Motor_Encoding_Struct;


// Struct for motor
typedef struct {
	TIM_TypeDef* 			PWM_type;
	TIM_TypeDef* 			ENCODER_type;
	Motor_Encoding_Struct*	Motor_Encoding_Struct;
	MotorName				motorName;
	GPIO_TypeDef* 			DIR_port;
	uint16_t				DIR_pin;
	SteeringState			steering_state;
	int						kPw; // proportional gain (how far away from goal)
	int 					kDw; //derivative gain (smoothing)

} Motor;


void motor_struct_init(Motor * motor, TIM_TypeDef * pwm,
		TIM_TypeDef * encoder, Motor_Encoding_Struct * motor_encoding_stuct,
		MotorName motorName, GPIO_TypeDef* DIR_port, uint16_t DIR_pin, int kPw, int kDw);


void stop_motor();
void set_motor_speed_percent(Motor * motor, float n);
void set_motor_speed_raw(Motor * motor, int n);
void set_motor_direction(Motor * motor, int n);


#endif
