/*
 * systick.c
 */

#include "main.h"
#include "pid.h"
#include "encoder.h"

int systick_counts = 0;

void SysTickFunction(void) {
	/*
	 * PID LOOP: THIS IS CALLED EVERY 1ms
	 */
	updatePID();
	set_counts((int16_t) TIM2->CNT);
	if (is_debouncing()){
		if(systick_counts++==100){
			systick_counts=0;
			set_debounce(0);
		}
	}
}
