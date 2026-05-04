#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "main.h"

typedef enum {
    PID = 0,
    CALIBRATION = 1,
	LEAVE_LIMIT = 2
} SteeringState;

extern SteeringState steering_state;


// Basically calibrationMode determines if
// the motor is going to go super slowly or not.
// see pid.c for more details.


void CalibrateMotor();

void set_calibration_motor_movement();


#endif
