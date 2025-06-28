/*
 * systick.c
 */

#include "main.h"
#include "pid.h"
#include "encoder.h"
#include "stdio.h"
#include "motor.h"

void SysTickFunction(void) {
	/*
	 * PID LOOP: THIS IS CALLED EVERY 1ms
	 */
	updatePID();
	set_counts((int16_t) TIM2->CNT);
}
