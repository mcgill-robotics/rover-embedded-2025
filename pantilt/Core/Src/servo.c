#include "rosjam.h"
#include "servo.h"
#include <stdint.h>

int32_t pan_angle = 90;
int32_t tilt_angle = 90;

int32_t min(int32_t a, int32_t b) {
    return (a < b) ? a : b;
}

int32_t max(int32_t a, int32_t b) {
    return (a > b) ? a : b;
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

void process_servo(void) {
    char buffer[100];
    int comma_index = -1;
    int index = 0;

    char pan_angle_string[50];
    char tilt_angle_string[50];
    float new_pan_angle;
    float new_tilt_angle;

    while(has_data()) {
        char incoming = read_char();

        // Check in case of overflow
        if (index >= 100) {
            comma_index = -1;
            index = 0;
        } else if (incoming == '\n') {
            if (comma_index != -1) {
                create_substring(buffer, pan_angle_string, 0, comma_index - 1);
                create_substring(buffer, tilt_angle_string, comma_index + 1, index - 1);

                new_pan_angle = string_to_float(pan_angle_string);
                new_tilt_angle = string_to_float(tilt_angle_string);

                set_pan(new_pan_angle);
                set_tilt(new_tilt_angle);
            }
            comma_index = -1;
            index = 0;
        } else {
            if (incoming == ',') comma_index = index;
            buffer[index] = incoming;
            index++;
        }
    }
}

void process_servo_uart(char *buffer, int length) {
    int comma_index = -1;
    int index = 0;

    char pan_angle_string[50];
    char tilt_angle_string[50];
    float new_pan_angle;
    float new_tilt_angle;

    while(index < length) {
        char incoming = buffer[index];

        if (incoming == '\n') {
            if (comma_index != -1) {
                create_substring(buffer, pan_angle_string, 0, comma_index - 1);
                create_substring(buffer, tilt_angle_string, comma_index + 1, index - 1);

                new_pan_angle = string_to_float(pan_angle_string);
                new_tilt_angle = string_to_float(tilt_angle_string);

                set_pan(new_pan_angle);
                set_tilt(new_tilt_angle);
            }

            return;
        } else {
            if (incoming == ',') comma_index = index;
            buffer[index] = incoming;
            index++;
        }
    }
}

void write_servo(TIM_HandleTypeDef *htim, uint16_t angle) {
    uint16_t pulse = MIN_PERIOD + (angle * (MAX_PERIOD - MIN_PERIOD) / MAX_ANGLE);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse);
}

void set_pan(float angle) {
    int32_t int_angle = (int32_t) (angle * PAN_GEAR_RATIO);
    pan_angle = max(MIN_ANGLE, min((pan_angle + int_angle), MAX_ANGLE));
    write_servo(PAN_SERVO, (uint16_t) pan_angle);
}

void set_tilt(float angle) {
    int32_t int_angle = (int32_t) angle;
    tilt_angle = max(MIN_ANGLE, min((tilt_angle + int_angle), MAX_ANGLE));
    write_servo(TILT_SERVO, (uint16_t) tilt_angle);
}