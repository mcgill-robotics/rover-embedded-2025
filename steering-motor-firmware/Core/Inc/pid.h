/*
 * pid.h
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define kPw 1
#define kDw 8
#define ALLOWED_ERROR 100
#define ALLOWED_ERROR_ZERO 300


void updatePID(void);
void setPIDGoalA(double angle);

#endif /* INC_PID_H_ */
