#include "servo.h"
#include <stdint.h>

void init_servos(void) {
    uint32_t psc = (CLK_FREQ / SERVO_TICK_RATE) - 1;
    uint32_t arr = SERVO_PWM_PERIOD - 1;

    __HAL_TIM_SET_PRESCALER(&htim1, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim1, arr);
    __HAL_TIM_SET_PRESCALER(&htim2, psc);
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

    set_pan(90);
    set_tilt(90);
}

// PWM_1 signal
void set_pan(uint16_t angle) {
    uint16_t pulse = MIN_PERIOD + (angle * (MAX_PERIOD - MIN_PERIOD) / MAX_ANGLE);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
}

// PWM_2 signal
void set_tilt(uint16_t angle) {
    uint16_t pulse = MIN_PERIOD + (angle * (MAX_PERIOD - MIN_PERIOD) / MAX_ANGLE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}