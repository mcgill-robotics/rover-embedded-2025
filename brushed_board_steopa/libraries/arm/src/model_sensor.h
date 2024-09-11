/**
 * @file          model_sensor_sensors.h
 * @author        Steve Ding, Oliver Philbin-Briscoe, Colin Gallacher
 * @version       V1.1.0
 * @date          26-Janurary-2023
 * @brief         generic sensor object
 *
 * @attention     For prototype example only
 *				  V1.1.0 - added offset calculations for current and inkwell sensors
 *
 */

#ifndef MODEL_SENSOR_H
#define MODEL_SENSOR_H

#include "Arduino.h"
#include <ADC.h>
#include "hardware_pins.h"

class model_sensor
{
public:
	void initialize_sensor(uint8_t pin);

	void reset_sensor(void);

	float read_sensor_value(void);

	float get_current(void);

	int16_t get_current_16(void);

	float get_raw_voltage(void);

private:
	ADC *_adc;

	int32_t _sensor_value;
	uint8_t _pin;
	float _current;
	float _raw_voltage;
	int16_t _current16;

	int32_t _sensor_offset;

	void _initialize_adc(void);

	void _update_sensor_value(void);
};

#endif