#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include <stdint.h>

#define CLK_FREQ         16000000   // 16 MHz clock
#define SERVO_TICK_RATE  1000000    // 1 us tick
#define SERVO_PWM_PERIOD 20000      // 20 ms PWM period (50 Hz)

#define MIN_PERIOD 500  // 500 us period (0 degrees)
#define MAX_PERIOD 2500 // 2.5 ms period (180 degrees)
#define MIN_ANGLE  0
#define MAX_ANGLE  180

#define PAN_GEAR_RATIO 2.0

extern TIM_HandleTypeDef htim1; // PWM_2
extern TIM_HandleTypeDef htim2; // PWM_1

#define PAN_SERVO &htim1
#define TILT_SERVO &htim2

int32_t min(int32_t a, int32_t b);
int32_t max(int32_t a, int32_t b);

void init_servos(void);
void write_servo(TIM_HandleTypeDef *htim, uint16_t angle);
void set_pan(float angle);
void set_tilt(float angle);

#endif /* __SERVO_H */