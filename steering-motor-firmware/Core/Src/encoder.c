
#include <stdlib.h>
#include "math.h"
#include "main.h"
#include "encoder.h"
#include "CAN_processing.h"
#include "pid.h"
#include "calibration.h"

int counts;
int need_debounce = 0;
int debounce_buffer = 0; // 32 bits buffer to fill with switch state

int is_debouncing(){
	return need_debounce;
}

void set_debounce(int debounce_state){
	need_debounce = debounce_state;
}

void set_counts(int n){
//	counts = ((n%MAX_COUNTS)+MAX_COUNTS)%MAX_COUNTS;
	counts = n;
}

int get_counts(){
	return counts;
}

float count_to_angle(int n){
	int no_offset = n-(LIMIT_SWITCH_RESET_COUNTS-MAX_COUNTS/2);
//	int new_n = abs(no_offset%MAX_COUNTS);
	float angle=((float)no_offset/(float)MAX_COUNTS)*360;
	return angle;
}

int angle_to_count(double n){
	float new_n = fabs(fmod(n,360));
	int offset = (LIMIT_SWITCH_RESET_COUNTS-MAX_COUNTS/2);
	return (int) ((new_n/(360))*MAX_COUNTS)+offset;
}

void reset_debounce_buffer(){
	debounce_buffer = 0;
}

// scan limit switch and return if considered pressed
int scan_switch(){
	int current_switch_reading = HAL_GPIO_ReadPin(LIMIT_GPIO_Port, LIMIT_Pin);
	debounce_buffer = (debounce_buffer<<1) | current_switch_reading;
	return debounce_buffer == 0xFFFFFFFF;
}


int try_calibrate_encoder(){
	// return 1 if calibrated
	if (scan_switch()){
		TIM2->CNT = angle_to_count(LIMIT_SWITCH_RESET_COUNTS);
		set_counts((int16_t) TIM2->CNT);
		steering_state = PID;
		return 1;
	}
	return 0;
//	if (!is_debouncing()){
//		set_debounce(1);
//		TIM2->CNT = angle_to_count(LIMIT_SWITCH_RESET_COUNTS);
//		if (steering_state == CALIBRATION){
//			setPIDGoalA(90); // move wheels back to the middle!
//		}
//		steering_state = PID;
//	}
}
