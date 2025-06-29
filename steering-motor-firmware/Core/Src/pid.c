/*
 * pid.c
 */

#ifndef PID_H
#define PID_H

#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "math.h"
#include "pid.h"
#include <stdatomic.h>
#include <stdlib.h>

// PID parameters


int angleError = 0;
int angleCorrection = 0;
int oldAngleError = 0;

static volatile atomic_int goalAngle = ATOMIC_VAR_INIT (0);


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
	// Find optimal direction
	if (abs(angleError) > MAX_COUNTS/2) {
		if (angleError > 0) {
			angleError = angleError - MAX_COUNTS;
		}
		else {
			angleError = angleError + MAX_COUNTS;
		}
	}
    angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
    // Set direction based on allowed error
	if (angleCorrection < 0){
		set_motor_direction(0);
	} else{
		set_motor_direction(1);
	}
	if (abs(angleCorrection) > 100) {
		angleCorrection = 100;
	}
	oldAngleError = angleError;
	// Stop if within error
	if (abs(angleError) <  ALLOWED_ERROR){
		set_motor_speed(0);
		return;
	}

	// Handle strange oscillations near 0 degrees with higher allowed error
	if (goalAngle == 0) {
		if (abs(angleError) < ALLOWED_ERROR_ZERO) {
			set_motor_speed(0);
			return;
		}
	}
	set_motor_speed(angleCorrection);

}

void setPIDGoalA(double angle) {
	atomic_store(&goalAngle, angle_to_count(angle));
}

#endif
