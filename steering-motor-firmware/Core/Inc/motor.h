#ifndef MOTOR_H
#define MOTOR_H

void set_motor_speed(int n);
void set_motor_direction(int n);
void stop_motor();
int get_actual_motor_direction();
void set_actual_motor_direction(int n);

#endif
