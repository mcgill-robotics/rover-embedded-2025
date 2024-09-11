/**
 * @file          hardware_pins.h
 * @author        Steve Ding, Oliver Philbin-Briscoe, Colin Gallacher
 * @version       V0.1.0
 * @date          11-March-2021
 * @brief         hardware pin definitions
 *
 * @attention     For prototype example only
 */

#ifndef HARDWARE_PINS_H
#define HARDWARE_PINS_H

#include <stdint.h>

const uint8_t ENCPIN1_1 = 0;
const uint8_t ENCPIN1_2 = 1;

const uint8_t ENCPIN2_1 = 2;
const uint8_t ENCPIN2_2 = 3;

const uint8_t ENCPIN3_1 = 4;
const uint8_t ENCPIN3_2 = 5;

const uint8_t PWMPIN1 = 13; // enable pin
const uint8_t DIRPIN1 = 16; // phase pin
const uint8_t nSLEEP1 = -1;

const uint8_t PWMPIN2 = 14;
const uint8_t DIRPIN2 = 17;
const uint8_t nSLEEP2 = -1;

const uint8_t PWMPIN3 = 15;
const uint8_t DIRPIN3 = 18;
const uint8_t nSLEEP3 = -1;

const uint8_t LED_O = -1;
const uint8_t LED_B = -1;

const uint8_t CURRENT_SENSE_A = 19;
const uint8_t CURRENT_SENSE_B = 20;
const uint8_t CURRENT_SENSE_C = 21;

const uint8_t LIM_1 = 9;
const uint8_t LIM_2 = 10;
const uint8_t LIM_3 = 11;
const uint8_t LIM_4 = 12;
const uint8_t LIM_5 = 22;
const uint8_t LIM_6 = 23;

const float wrist_pitch_max_angle = 30.0;
const float wrist_pitch_min_angle = -30.0;
#endif