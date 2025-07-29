#ifndef MOTOR_H
#define MOTOR_H

extern int power_limit;

void set_motor_speed_raw(int n);
void set_motor_direction(int n);
void stop_motor();
void set_motor_speed_percent(float n);

#endif
