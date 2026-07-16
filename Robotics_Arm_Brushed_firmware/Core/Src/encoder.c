#include <stdlib.h>
#include "math.h"
#include "main.h"
#include "encoder.h"
#include "CAN_processing.h"
#include "pid.h"
#include "motorControl.h"
//#include "calibration.h"



//implement for all 3 encoders???? separately???



//int counts;
//int need_debounce = 0;
int debounce_buffer = 0; // 32 bits buffer to fill with switch state


void motor_encoding_struct_init(Motor_Encoding_Struct * encoding, int encoder_max_counts,
		int lm_sw_reset_counts){
	encoding->ENCODER_MAX_COUNTS = encoder_max_counts;
	encoding->LMSW_RESET_COUNTS = lm_sw_reset_counts;
	encoding->curr_counts = 0;
	encoding->need_debounce = 0;
	encoding->oldAngleError = 0;
}


//
//int is_debouncing(Motor_Encoding_Struct * encoding){
//	return encoding->need_debounce;
//}
//* integrated into main loop

//void set_debounce(Motor_Encoding_Struct * encoding, int debounce_state){
//	encoding->need_debounce = debounce_state;
//}
//* integrated into main loop

void set_counts(Motor_Encoding_Struct * encoding, int n){
//	counts = ((n%MAX_COUNTS)+MAX_COUNTS)%MAX_COUNTS;
	encoding->curr_counts = n;
}

int get_counts(Motor_Encoding_Struct * encoding){
	return encoding->curr_counts;
}

float count_to_angle(Motor_Encoding_Struct * encoding, int n){
	//remove the offset from the angle to get the raw count
	//int no_offset = n-(encoding->LMSW_RESET_COUNTS - encoding->ENCODER_MAX_COUNTS/2);
//	int new_n = abs(no_offset%MAX_COUNTS);
	int no_offset = n;

	float angle=((float)no_offset/(float) encoding->ENCODER_MAX_COUNTS)*360;
	return angle;
}

int angle_to_count(Motor_Encoding_Struct * encoding, double n){
	//float new_n = fabs(fmod(n,360));
	//int offset = (encoding->LMSW_RESET_COUNTS - encoding->ENCODER_MAX_COUNTS/2);

	int offset = 0;

	return (int) ((n/(360)) * encoding->ENCODER_MAX_COUNTS) + offset;
}

//void reset_debounce_buffer(){
//	debounce_buffer = 0;
//}
//*integrated into main loop


// scan limit switch and return if considered pressed
//int scan_switch(){
//	int current_switch_reading = HAL_GPIO_ReadPin(LIMIT_GPIO_Port, LIMIT_Pin);
//	debounce_buffer = (debounce_buffer<<1) | current_switch_reading;
//	return debounce_buffer == 0xFFFFFFFF;
//}
//*integrated into main



int lmsw_pitch_up_recalibrate(Motor * motor){
	//reset the maximum number of counts
	//set the current number of counts to the MAX
	HAL_GPIO_TogglePin(LED_pitch_GPIO_Port, LED_pitch_Pin);

	motor->Motor_Encoding_Struct->LMSW_RESET_COUNTS = motor->Motor_Encoding_Struct->curr_counts;
	set_motor_speed_raw(motor, 0);
	setPIDGoalA(motor, count_to_angle(motor->Motor_Encoding_Struct, motor->Motor_Encoding_Struct->curr_counts));



	return 1;
}

int lmsw_pitch_down_recalibrate(Motor * motor){
	//reset to minimum number of counts
	//TODO: ITS NOT SUPPOSED TO BE 0, WHATS THE ACTUAL MIN VALUE????
	int offset = motor->Motor_Encoding_Struct->LMSW_RESET_COUNTS - motor->Motor_Encoding_Struct->ENCODER_MAX_COUNTS/2;
	set_counts(motor->Motor_Encoding_Struct, offset);
	return 1;
}

int lmsw_roll_recalibrate(Motor * motor){
	//reset to minimum number of counts
		//reset counts to 0 -> offset-ed
		int offset = motor->Motor_Encoding_Struct->LMSW_RESET_COUNTS - motor->Motor_Encoding_Struct->ENCODER_MAX_COUNTS/2;
		set_counts(motor->Motor_Encoding_Struct, offset);
		return 1;
}

int lmsw_gripper_recalibrate(Motor * motor){
	//reset calibration & set current number of counts to MAX
	motor->ENCODER_type->CNT = motor->Motor_Encoding_Struct->LMSW_RESET_COUNTS;
	set_counts(motor->Motor_Encoding_Struct, (uint16_t) motor->ENCODER_type->CNT);
	return 1;
}

//int try_calibrate_encoder(){
//	// return 1 if calibrated
//	//if (scan_switch()){
//		TIM2->CNT = LIMIT_SWITCH_RESET_COUNTS;
//		set_counts((uint16_t) TIM2->CNT);
//		return 1;
//	//}
//	//return 0;

//	if (!is_debouncing()){
//		set_debounce(1);
//		TIM2->CNT = angle_to_count(LIMIT_SWITCH_RESET_COUNTS);
//		if (steering_state == CALIBRATION){
//			setPIDGoalA(90); // move wheels back to the middle!
//		}
//		steering_state = PID;
//	}
//}





