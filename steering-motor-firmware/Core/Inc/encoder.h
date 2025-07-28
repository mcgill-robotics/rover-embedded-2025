#ifndef ENCODER_H
#define ENCODER_H

#define MAX_COUNTS 33024//16*516*4.0.
// reset angles for left and for right motors
#define LIMIT_SWITCH_RESET_ANGLE_RIGHT 180
#define LIMIT_SWITCH_RESET_ANGLE_LEFT 180

int is_debouncing();
void set_debounce(int debounce_state);
void set_counts(int n);
int get_counts();
float count_to_angle(int n);
int angle_to_count(double n);
void calibrate_encoder();

#endif
