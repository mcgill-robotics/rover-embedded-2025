/*
 * systick.c
 */

#include "main.h"
#include "pid.h"
#include "encoder.h"
#include "calibration.h"

int systick_counts = 0;

void SysTickFunction(void) {
	/*
	 * THIS IS CALLED EVERY 1ms
	 */


	/*TODO
	 *
	// poll limit switch after interrupt triggered
	if (is_debouncing()){
		if (try_calibrate_encoder()){
			// reset to stop polling and set switch to non pressed state
			set_debounce(0);
			reset_debounce_buffer();
			// align wheel if initial calibration
			if (steering_state == CALIBRATION){
				setPIDGoalA(90);
			}
			steering_state = LEAVE_LIMIT;
		}
	}
	switch (steering_state) {
		case (PID):
			updatePID();
			break;
		case(CALIBRATION):
			set_calibration_motor_movement();
			break;
		case(LEAVE_LIMIT):
			leave_limit_switch();
			break;
	}
	set_counts((uint16_t) TIM2->CNT);
//	if (is_debouncing()){
//		if(systick_counts++==100){
//			systick_counts=0;
//			set_debounce(0);
//		}
//	}
 *
 *
 *
 */
}
