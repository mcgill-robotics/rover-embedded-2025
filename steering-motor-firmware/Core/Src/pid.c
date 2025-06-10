/*
 * pid.c
 */

#ifndef PID_H
#define PID_H

#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "math.h"
#include <stdatomic.h>

int angleError = 0;
int angleCorrection = 0;
int oldAngleError = 0;

#define MAX_COUNTS 33024
#define HALF_COUNTS 16512


float kPw = 1;
float kDw = 8;

// FIGURE OUT HOW LOCK FREE ATOMIC INTS WORK
// This is probably the problem behind the goalAngle not updating
static volatile atomic_int goalAngle = ATOMIC_VAR_INIT (0);

void resetPID() {
	/*
	 * For assignment 3.1: This function does not need to do anything
	 * For assignment 3.2: This function should reset all the variables you define in this file to help with PID to their default
	 *  values. You should also reset your motors and encoder counts (if you tell your rat to turn 90 degrees, there will be a big
	 * difference in encoder counts after it turns. If you follow that by telling your rat to drive straight without first
	 * resetting the encoder counts, your rat is going to see a huge angle error and be very unhappy).
	 *
	 * You should additionally set your distance and error goal values (and your oldDistanceError and oldAngleError) to zero.
	 */
}

void updatePID() {
	/*
	 * This function will do the heavy lifting PID logic. It should do the following: read the encoder counts to determine an error,
	 * use that error along with some PD constants you determine in order to determine how to set the motor speeds, and then actually
	 * set the motor speeds.
	 *
	 * For assignment 3.1: implement this function to get your rat to drive forwards indefinitely in a straight line. Refer to pseudocode
	 * example document on the google drive for some pointers
	 *
	 * TIPS (assignment 3.1): Create kPw and kDw variables, use a variable to store the previous error for use in computing your
	 * derivative term. You may get better performance by having your kDw term average the previous handful of error values instead of the
	 * immediately previous one, or using a stored error from 10-15 cycles ago (stored in an array?). This is because systick calls so frequently
	 * that the error change may be very small and hard to operate on.
	 *
	 * For assignment 3.2: implement this function so it calculates distanceError as the difference between your goal distance and the average of
	 * your left and right encoder counts. Calculate angleError as the difference between your goal angle and the difference between your left and
	 * right encoder counts. Refer to stocked example document on the google drive for some pointers.
	 */
	angleError = atomic_load(&goalAngle) - get_counts();
	if (abs(angleError) > HALF_COUNTS) { // the units need to be fixed but this gets the best path
		if (angleError > 0) {
			angleError = angleError - MAX_COUNTS;
		}
		else {
			angleError = angleError + MAX_COUNTS;
		}
	}


    angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
//	printf("correction %d\r\n", angleCorrection);
	if (angleCorrection < 0){
		set_motor_direction(0);
	} else{
		set_motor_direction(1);
	}
	if (abs(angleCorrection) > 100) {
		angleCorrection = 100;
	}
	oldAngleError = angleError;
	if (abs(angleError) <  100){
		set_motor_speed(0);
		return;
	}
	if (goalAngle == 0) {
		if (abs(angleError) < 300) {
			set_motor_speed(0);
			return;
		}
	}
	set_motor_speed(angleCorrection);

}

void setPIDGoalD(int16_t distance) {
	/*
	 * For assignment 3.1: this function does not need to do anything.
	 * For assignment 3.2: this function should set a variable that stores the goal distance.
	 */
}

void setPIDGoalA(double angle) {
//	printf("set goal %f\r\n", angle);
	int counts = angle_to_count(angle);
	atomic_store(&goalAngle, angle_to_count(counts));
//	printf("goal %d\r\n", goalAngle);
}

int8_t PIDdone(void) { // There is no bool type in C. True/False values are represented as 1 or 0.
	/*
	 * For assignment 3.1: this function does not need to do anything (your rat should just drive straight indefinitely)
	 * For assignment 3.2:This function should return true if the rat has achieved the set goal. One way to do this by having updatePID() set some variable when
	 * the error is zero (realistically, have it set the variable when the error is close to zero, not just exactly zero). You will have better results if you make
	 * PIDdone() return true only if the error has been sufficiently close to zero for a certain number, say, 50, of SysTick calls in a row.
	 */

	return 1;
}

#endif
