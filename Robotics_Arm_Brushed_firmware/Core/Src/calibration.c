#include "main.h"
#include "encoder.h"
#include "pid.h"
#include "motorControl.h"
#include "CAN_processing.h"
#include "calibration.h"

//SteeringState steering_state = PID;

// Hi, this is going to be the calibration sequence for the motors. It's going to
// consist of a couple steps.

// 1. Depending on what side of the rover the motor is on, it will turn the wheel
// at 10% speed (maximum 180 degrees) until the limit switch is hit.

// After the limit switch is hit, that point will be the new 180/0 degree point (again,
// depending on what side of the rover the board is on).

//  2. The motor will then move to the new 90 degree point at 100% speed.

// This function is going to trigger on startup (see call in main loop)

void CalibrateMotor(Motor * motor) {
	motor->motor_state = CALIBRATION;
}
 // Note: Part 2. of the calibration sequence as described above is implemented
// in Calibrate_endoder in encoder.c

// Another Note: The speed decrease is accomplished through
// a new vairable "calibrationMode" which is declared/used
// in pid.c, set to one in CalibrateMotor() (above), then
// set back to zero in the limit switch interrupt

void set_calibration_motor_movement(Motor * motor){
	set_motor_direction(motor, 1);
	set_motor_speed_percent(motor, 80);
}
