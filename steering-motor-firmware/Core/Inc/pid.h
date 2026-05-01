/*
 * pid.h
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#define kPw 50
#define kDw 5
#define ALLOWED_ERROR 20
#define ALLOWED_ERROR_ZERO 300


int updatePID(void);
void setPIDGoalA(double angle);
void leave_limit_switch();

#endif /* INC_PID_H_ */
