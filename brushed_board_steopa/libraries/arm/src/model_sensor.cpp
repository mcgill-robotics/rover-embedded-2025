/**
 * @file          model_sensor.cpp
 * @author        Vincent Boucher, Steve Ding
 * @version       V1.0.0
 * @date          30-May-2023
 * @brief         current sensor object
 *
 * @attention     For prototype example only
 *				  V1.0.0 - created
 *
 */
#include "model_sensor.h"

void model_sensor::initialize_sensor(uint8_t pin)
{
	_sensor_value = 0;
	_sensor_offset = 0;

	_pin = pin;

	_initialize_adc();

	pinMode(pin, INPUT);

	delay(10);

	for (uint8_t i = 0; i < 40; i++)
	{
		_update_sensor_value();
		_sensor_offset = _sensor_offset + _sensor_value;
	}

	_sensor_offset = _sensor_offset / 40;

	_sensor_offset = 1862 - _sensor_offset;
}

void model_sensor::reset_sensor(void)
{
	_sensor_value = 0;
}

float model_sensor::read_sensor_value(void)
{
	_update_sensor_value();
	float sensorValue = _sensor_value + _sensor_offset;
	_current16 = sensorValue - 1862;
	_raw_voltage = sensorValue * (3.3 / 4095.0);
	_current = (_raw_voltage - 1.5) * 2;
	return _current;
}

float model_sensor::get_current(void)
{
	return _current;
}

int16_t model_sensor::get_current_16(void)
{
	return _current16;
}

float model_sensor::get_raw_voltage(void)
{
	return _raw_voltage;
}

void model_sensor::_update_sensor_value(void)
{
	// if (_pin == CURRENT_SENSE_A1 ||
	// 		_pin == CURRENT_SENSE_B1 ||
	// 		_pin == CURRENT_SENSE_A2 ||
	// 		_pin == CURRENT_SENSE_B2) {
	//     _sensor_value = _adc->adc1->analogRead(_pin);
	// }else{
	_sensor_value = _adc->adc0->analogRead(_pin);
	// }
}

void model_sensor::_initialize_adc(void)
{

	_adc = new ADC();

	_adc->adc0->setAveraging(4);
	_adc->adc0->setResolution(12);
	_adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
	_adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);

	_adc->adc1->setAveraging(4);
	_adc->adc1->setResolution(12);
	_adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
	_adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
}