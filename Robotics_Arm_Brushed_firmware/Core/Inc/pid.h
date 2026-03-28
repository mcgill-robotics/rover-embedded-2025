/*
 * pid.h
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "motorControl.h"
//#define kPw 50 defined in motor struct
//#define kDw 5  // defined in motor struct

#define ALLOWED_ERROR 20
#define ALLOWED_ERROR_ZERO 300


int updatePID(Motor * motor);
void setPIDGoalA(Motor * motor, double angle);
void leave_limit_switch();

#endif /* INC_PID_H_ */
