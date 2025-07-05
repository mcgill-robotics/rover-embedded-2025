
#include <stdlib.h>
#include "math.h"
#include "main.h"
#include "encoder.h"

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
	float new_n = fabs(fmod(n,360));
	int c = MAX_COUNTS;
	return (int) ((new_n/(360))*MAX_COUNTS);
}

void calibrate_encoder(){
	if (!is_debouncing()){
		limit_calls++;
		set_debounce(1);
		TIM2->CNT = LIMIT_SWITCH_RESET_ANGLE;
	}
//	set_counts(angle_to_count(LIMIT_SWITCH_RESET_ANGLE));
}
