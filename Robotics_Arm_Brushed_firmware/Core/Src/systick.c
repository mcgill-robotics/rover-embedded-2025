/*
 * systick.c
 */

#include "main.h"
#include "pid.h"
#include "encoder.h"
#include "calibration.h"
//
//int systick_counts = 0;
//
//
//void SysTickFunction(void) {
//	/*
//	 * THIS IS CALLED EVERY 1ms
//	 */
//
//	for (int i = 0; i < NB_MOTORS; i++){
//
//		Motor * motor =  all_motors_list[i];
//
////		int counter = 0;
////		if (motor->motorName == 1){ //pitch
////			if (counter == 1000){
////				HAL_GPIO_TogglePin(LED_pitch_GPIO_Port, LED_pitch_Pin);
////				counter = 0;
////			}
////			else counter ++;
////		}
//
//		// poll limit switch after interrupt triggered
//		//if (is_debouncing(motor->Motor_Encoding_Struct)){
//			//if (try_calibrate_encoder()){
//				// reset to stop polling and set switch to non pressed state
//	//			set_debounce(motor, 0);
//	//			reset_debounce_buffer();
//				// align wheel if initial calibration
////				if (motor->steering_state == CALIBRATION){
////					setPIDGoalA(motor, 90);
////				}
////				motor->steering_state = LEAVE_LIMIT;
//			//}
//		//}
//
//		//normal systick loop execution
//		switch (motor->steering_state) {
//			case (PID):
//				updatePID(motor);// TODO FIX
//				break;
//			case(CALIBRATION):
//				set_calibration_motor_movement(motor);
//				break;
//			case(LEAVE_LIMIT):
//				//leave_limit_switch(); // TODO FIX
//				break;
//		}
//		set_counts(motor->Motor_Encoding_Struct, (uint16_t) TIM2->CNT);
//
//		//	if (is_debouncing()){
//	//		if(systick_counts++==100){
//	//			systick_counts=0;
//	//			set_debounce(0);
//	//		}
//	//	}
//
//	}
//
//}
