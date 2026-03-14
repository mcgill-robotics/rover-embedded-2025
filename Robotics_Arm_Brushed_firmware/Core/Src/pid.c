/*
 * pid.c
 */

#ifndef PID_H
#define PID_H

#include "main.h"
#include "encoder.h"
#include "motorControl.h"
#include "math.h"
#include "pid.h"
#include <stdatomic.h>
#include <stdlib.h>
//#include "TestList.h"
//#include "Calibration.h"
// PID parameters





int angleError = 0;
int angleCorrection = 0;
int oldAngleError = 0;
int currentGoal =0;
int calibrationMode = 0; // if calibrating, lower the speed!
// In Calibration.c, calibrationMode is set to one at the start
// of the program. Once the limit switch is hit,
// then it is set back to zero and the motor goes full speed
// again. What's important is that the joystick can't
// be moved while calibrating!!!!!!!!!!!!!!!!
// I will mention this to whoever is controlling the steering.

static volatile atomic_int goalAngle = ATOMIC_VAR_INIT (0);

// PID implementation
int updatePIDImpl(int goal) {

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



	/* TODO

	currentGoal = goal;
	//return 1 when goal reached
	angleError = goal - get_counts();
	// Find optimal direction
//	if (abs(angleError) > MAX_COUNTS/2) {
//		if (angleError > 0) {
//			angleError = angleError - MAX_COUNTS;
//		}
//		else {
//			angleError = angleError + MAX_COUNTS;
//		}
//	}


	// --> define kPw and kDw for each individual motor !
    angleCorrection = kPw * angleError + kDw * (angleError - oldAngleError);
    // Set direction based on allowed error
	if (angleCorrection < 0){
		set_motor_direction(0);
	} else{
		set_motor_direction(1);
	}
	oldAngleError = angleError;
	// Stop if within error
	if (abs(angleError) <  ALLOWED_ERROR){
		set_motor_speed_raw(0);
		//reset error once goal reached
		oldAngleError = 0;
		return 1;
	}

	// Handle strange oscillations near 0 degrees with higher allowed error
//	if (goalAngle == 0) {
//		if (abs(angleError) < ALLOWED_ERROR_ZERO) {
//			set_motor_speed(0);
//			return;
//		}
//	}
	set_motor_speed_raw(angleCorrection);


	*/
	return 0;
}

// normal pid with goal from can
int updatePID(){
	return updatePIDImpl(atomic_load(&goalAngle));
}

//run pid with overriden goal(to ignore goal from can when moving away from limit switch)
int updatePIDOverrideGoal(int override){
	return updatePIDImpl(override);
}

// use PID to move away from limit switch a little bit after switch is triggered
void leave_limit_switch(){
	/* TODO
	if(updatePIDOverrideGoal(angle_to_count(170))){

		//steering_state = PID; // TEMPORARILY REMOVED FOR TESTING; ADD BACK
		// prevent continuously moving to the limit
		if(atomic_load(&goalAngle) > angle_to_count(170)){
			stop_motor();
		}
	}
	*/

}

void setPIDGoalA(double angle) {
	/* TODO
	atomic_store(&goalAngle, angle_to_count(angle));
	*/
}




#endif
