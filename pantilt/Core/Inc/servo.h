#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

#define CLK_FREQ 16000000       // 16 MHz clock
#define SERVO_TICK_RATE 1000000 // 1 us tick
#define SERVO_PWM_PERIOD 20000  // 20 ms PWM period (50 Hz)

#define MIN_PERIOD 1000 // 1 ms pulse
#define MAX_PERIOD 2000 // 2 ms pulse
#define MAX_ANGLE  180

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void init_servos(void);
void set_pan(uint16_t angle);
void set_tilt(uint16_t angle);

#endif /* __SERVO_H */