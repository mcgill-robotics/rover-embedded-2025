#ifndef MOTOR_H
#define MOTOR_H

extern int power_limit;

void set_motor_speed(int n);
void set_motor_direction(int n);
void stop_motor();

#endif
