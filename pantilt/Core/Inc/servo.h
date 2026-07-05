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
#define MAX_ANGLE  270
#define MAX_PAN_ANGLE  360
#define MAX_TILT_ANGLE  270


#define PAN_GEAR_RATIO 2.0

extern TIM_HandleTypeDef htim1; // PWM_2
extern TIM_HandleTypeDef htim2; // PWM_1

#define PAN_SERVO &htim1
#define TILT_SERVO &htim2

typedef enum ProcessResult {
	PROC_OK,
	PROC_INVALID,
	PROC_NEED_MORE
} ProcessResult;

double min(double a, double b);
double max(double a, double b);

void process_servo(void);
void create_substring(char *buffer, char *destination, int start, int end);
int process_servo_uart(char *buffer, int length, ProcessResult* res);
void init_servos(void);
void write_servo(TIM_HandleTypeDef *htim, double angle);
void set_pan(double angle);
void set_tilt(double angle);

#endif /* __SERVO_H */