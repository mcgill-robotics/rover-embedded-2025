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

void write_servo(TIM_HandleTypeDef *htim, uint16_t angle) {
    uint16_t pulse = MIN_PERIOD + (angle * (MAX_PERIOD - MIN_PERIOD) / MAX_ANGLE);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, pulse);
}

// PWM_2 signal
void set_pan(float angle) {
    int32_t int_angle = (int32_t) (angle * PAN_GEAR_RATIO);
    pan_angle = max(MIN_ANGLE, min((pan_angle + int_angle), MAX_ANGLE));
    write_servo(PAN_SERVO, (uint16_t) pan_angle);
}

// PWM_1 signal
void set_tilt(float angle) {
    int32_t int_angle = (int32_t) angle;
    tilt_angle = max(MIN_ANGLE, min((tilt_angle + int_angle), MAX_ANGLE));
    write_servo(TILT_SERVO, (uint16_t) tilt_angle);
}