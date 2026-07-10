#include "rosjam.h"
#include "servo.h"
#include <stdint.h>

double pan_angle = 90;
double tilt_angle = 90;

double min(double a, double b) {
    return (a < b) ? a : b;
}

double max(double a, double b) {
    return (a > b) ? a : b;
}


int process_servo_uart(char *buffer, int length, ProcessResult* res) {
    int comma_count = 0;
    int comma_index = -1;
    int index = 0;

    char pan_angle_string[50];
    char tilt_angle_string[50];
    double new_pan_angle;
    double new_tilt_angle;

    while(index < length) {
        char incoming = buffer[index];

        if (incoming == '\n') {
            if (comma_index != -1&&comma_count==1) {
                create_substring(buffer, pan_angle_string, 0, comma_index - 1);
                create_substring(buffer, tilt_angle_string, comma_index + 1, index - 1);
                // uses atof so invalid input just turns into 0, so no change to output happens
                new_pan_angle = string_to_float(pan_angle_string); 
                new_tilt_angle = string_to_float(tilt_angle_string);

                set_pan(new_pan_angle);
                set_tilt(new_tilt_angle);
                *res = PROC_OK;
                return index+1;
            }
            *res = PROC_INVALID;
            return index+1;
        } else {
            if (incoming == ',') {
                comma_index = index;
                comma_count++;
            }
            buffer[index] = incoming;
            index++;
        }
    }
    *res = PROC_NEED_MORE;
    return index;
}

void create_substring(char *buffer, char *destination, int start, int end) {
    for (int i = start; i <= end; i++) {
        destination[i - start] = buffer[i];
    }
    destination[end - start + 1] = '\0';
}

void init_servos(void) {
    uint32_t psc = (CLK_FREQ / SERVO_TICK_RATE) - 1;
    uint32_t arr = SERVO_PWM_PERIOD - 1;

    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_PRESCALER(&htim2, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    write_servo(PAN_SERVO, (uint16_t) pan_angle);
    write_servo(TILT_SERVO, (uint16_t) tilt_angle);
}

int buffer_index = 0;
int comma_index = -1;

void process_servo(void) {
    char buffer[100];

    char pan_angle_string[50];
    char tilt_angle_string[50];
    double new_pan_angle;
    double new_tilt_angle;

    while(has_data()) {
        char incoming = read_char();

        // Check in case of overflow
        if (buffer_index >= 100) {
            comma_index = -1;
            buffer_index = 0;
        } else if (incoming == '\n') {
            if (comma_index != -1) {
                create_substring(buffer, pan_angle_string, 0, comma_index - 1);
                create_substring(buffer, tilt_angle_string, comma_index + 1, buffer_index - 1);

                new_pan_angle = string_to_float(pan_angle_string);
                new_tilt_angle = string_to_float(tilt_angle_string);

                set_pan(new_pan_angle);
                set_tilt(new_tilt_angle);
            }
            comma_index = -1;
            buffer_index = 0;
        } else {
            if (incoming == ',') comma_index = buffer_index;
            buffer[buffer_index] = incoming;
            buffer_index++;
        }
    }
}

void write_servo(TIM_HandleTypeDef *htim, double angle) {
    double pulse = MIN_PERIOD + (angle * (MAX_PERIOD - MIN_PERIOD) / MAX_ANGLE);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint16_t) pulse);
}

void set_pan(double angle) {
    pan_angle = max(MIN_ANGLE, min((pan_angle + angle), MAX_PAN_ANGLE));
    double servo_angle = (pan_angle / PAN_GEAR_RATIO);
    write_servo(PAN_SERVO, servo_angle);
}

void set_tilt(double angle) {
    tilt_angle = max(MIN_ANGLE, min((tilt_angle + angle), MAX_TILT_ANGLE));
    write_servo(TILT_SERVO, tilt_angle);
}