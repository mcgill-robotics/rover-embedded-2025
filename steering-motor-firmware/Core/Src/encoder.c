
#include <stdlib.h>
#include "math.h"
#include "main.h"
#include "encoder.h"
#include "CAN_processing.h"
#include "pid.h"
#include "calibration.h"

int counts;
int need_debounce = 0;
int limit_calls = 0;

int is_debouncing(){
	return need_debounce;
}

void set_debounce(int debounce_state){
	need_debounce = debounce_state;
}

void set_counts(int n){
	counts = ((n%MAX_COUNTS)+MAX_COUNTS)%MAX_COUNTS;
}

int get_counts(){
	return counts;
}

float count_to_angle(int n){
	int new_n = abs(n%MAX_COUNTS);
	float angle=((float)n/(float)MAX_COUNTS)*360;
	if (angle<0){
		return 360+angle;
	}
	return angle;
}

int angle_to_count(double n){
	if (!calibrationMode){
		if (n < 0){
			n=0;
		} else if (n > 180){
			n = 180;
		}
	}
	float new_n = fabs(fmod(n,360));
	int c = MAX_COUNTS;
	return (int) ((new_n/(360))*MAX_COUNTS);
}

void calibrate_encoder(){
	if (!is_debouncing()){
		limit_calls++;
		set_debounce(1);
		if (STEERING_ID == LF_STEER || STEERING_ID == LB_STEER) {
			TIM2->CNT = angle_to_count(LIMIT_SWITCH_RESET_ANGLE_LEFT);
			setPIDGoalA(90); // move wheels back to the middle!
		}
		if (STEERING_ID == RF_STEER || STEERING_ID == RB_STEER) {
			TIM2->CNT =  angle_to_count(LIMIT_SWITCH_RESET_ANGLE_RIGHT);
			setPIDGoalA(90); // ...
		}
	}
//	set_counts(angle_to_count(LIMIT_SWITCH_RESET_ANGLE));
}
