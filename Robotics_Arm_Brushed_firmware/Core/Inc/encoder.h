#ifndef ENCODER_H
#define ENCODER_H

//Find the one specific with gear ratio of motors (i.e. more than 1 MAX_COUNT)
//#define MAX_COUNTS 33024
// calculated as: 16*516*4.0. (gear ratio* gear ratio * pulses)
//^ now given in motor struct in main upon initialization

//reset angles for left and for right motors
#define LIMIT_SWITCH_RESET_COUNTS 50000 //(depends how angles defined -- corresponds to 180)


typedef struct {
	int 		ENCODER_MAX_COUNTS;
	int 		LMSW_RESET_COUNTS;
	int			curr_counts;

} Motor_Encoding_Struct;

void motor_encoding_struct_init(Motor_Encoding_Struct * encoding, int encoder_max_counts, int lm_sw_reset_counts);


//int is_debouncing();
//void set_debounce(int debounce_state);
void set_counts(Motor_Encoding_Struct * encoding, int n);
int get_counts();
float count_to_angle(Motor_Encoding_Struct * encoding, int n);
int angle_to_count(Motor_Encoding_Struct * encoding, double n);
//int try_calibrate_encoder();
//void reset_debounce_buffer();

#endif
