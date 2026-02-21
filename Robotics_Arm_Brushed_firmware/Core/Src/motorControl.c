/*
 *
 * This file contains motor control functions for the arm brushed motors.
 *
 */

/*
#include <stdint.h>
#include <stdbool.h>

#include "CAN_processing.h"
#include "pid.h"
#include "encoder.h"
//#include "uart_debugging.h"


//Ramp up "torque" parameters
#define RAMP_MIN_MS_RUN      100   // small tweaks while already running
#define RAMP_MIN_MS_STARTUP  200   // zero → set-point, or after stop

//Startup Watchdog parameters
#define START_WD_WINDOW_MS   5000   // length of rolling window
#define START_WD_THRESHOLD   3     // # kicks that trigger a fault


// Variable declarations
static float g_lastCommandedSpeed = 0; // Previous speed setpoint given to the esc
int s_previousDirection = 0 ; // 0 means idle, 1 means forward, -1 means backward
int DELTA_SPEED_THRESH = 200; // Threshold to clip differing speed commands
const float SPEED_ZERO_THR = 50.0f; // Threshold to consider "almost zero" / unreliable speed feedback point
const float MAX_SPEED_THR = 3200.0f;
const int WAIT_AFTER_STOP = 250; // amount in ms motor will wait after it has issued a stopMotor command
const int SAFE_STOP_SPEED_THRESHOLD = 400;

//Motor parameters --> Get these from the profiled motor!!
static float g_maxTorque   = 0.30f;    // [N·m]  conservative value
const float g_startupTorque = 0.15f;   // typical open-loop pull-in
static float g_inertia     = 0.00001242f; // [kg·m^2]
static float g_speedThresh = 50.0f;    // threshold below which we treat speed as zero

static StartWatchdog s_startWd = { .firstTick = 0, .attempts = 0 };




*/
