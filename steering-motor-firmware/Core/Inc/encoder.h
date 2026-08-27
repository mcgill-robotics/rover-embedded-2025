#ifndef ENCODER_H
#define ENCODER_H

#define MAX_COUNTS 33024//16*516*4.0.
// reset angles for left and for right motors
#define LIMIT_SWITCH_RESET_COUNTS 50000

int is_debouncing();
void set_debounce(int debounce_state);
void set_counts(int n);
int get_counts();
float count_to_angle(int n);
int angle_to_count(double n);
int try_calibrate_encoder();
void reset_debounce_buffer();

#endif
