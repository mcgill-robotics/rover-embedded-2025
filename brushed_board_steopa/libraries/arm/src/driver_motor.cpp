/**
 * @file          driver_motor.cpp
 * @author        Vincent Boucher, Steve Ding
 * @version       V1.0.0
 * @date          30-May-2023
 * @brief         motor object
 *
 * @attention     For prototype example only
 *				  V1.0.0 - created
 *
 */
#include "driver_motor.h"
#include "pid.h"

/// @brief Initializes the motor object
/// @param direction Positive direction for motor
/// @param port Port used for motor connection
/// @param maxTorque Maximum torque motor can output
void driver_motor::initialize_motor(uint8_t direction_motor, uint8_t motor_pwn_pin, uint8_t motor_dir_pin, uint8_t motor_fault_pin, float max_torque_motor, float torque_constant_motor)
{
	_motor_max_torque = max_torque_motor;
	_torque_constant_motor = torque_constant_motor;
	_target_torque = 0.0;
	_target_angle_ps = 0.0; // The desired angle to turn to (sent from software)
	_output_motor = 0;
	_gear_ratio = 1.0;			  // default for no effect
	_angle_full_turn_es = 360.0f; // encoder angle corresponding to a full turn (considering gear ratio)

	// Linear joint by default.
	_is_circular_joint = false;
	// Foward direction logic level
	_forward_dir = direction_motor;

	_ctrl_period = 0.0;
	_sampling_period = 0.0;

	_motor_dir_pin = motor_dir_pin;
	_motor_fault_pin = motor_fault_pin;
	_motor_pwm_pin = motor_pwn_pin;

	// kalman params
	_kalman_gain = 0;
	_current_estimate = 0;

	_err_estimate = _ERR_ESTIMATE_INIT;
	_last_estimate = _LAST_ESTIMATE_INIT;
	_err_measure = _ERR_MEASURE_INIT;
	_q = _Q_INIT;

	_error = 0.0;
	_previous_error = 0.0;

	_error_int = 0.0;
	_error_dir = 0.0;

	// initializes pins and resolution
	// _pwm_set_resolution(_PWM_BIT_RESOLUTION);

	pinMode(_motor_pwm_pin, OUTPUT);
	pinMode(_motor_dir_pin, OUTPUT);
	pinMode(_motor_fault_pin, OUTPUT);

	// TODO: is this not necessary for pid?
	// Sets frequency of pwm
	_pwm_setup(_PWM_FREQUENCY);

	// 24V motor therefore max is 24 and min is -24
	pid_instance = new PID(0.1, 24, -24, 2.0, 0, 0);
}

/// @brief Sets the torque and writes the PWM value to the motor
/// @param torque Torque value to be written to the motor
void driver_motor::set_target_torque(float torque)
{
	_target_torque = torque;
}

float driver_motor::get_target_torque(void)
{
	return _target_torque;
}

float driver_motor::get_output_motor(void)
{
	return _output_motor;
}

void driver_motor::set_target_angle_ps(float angle_ps)
{
	if (!_is_circular_joint)
	{
		if (angle_ps > _max_angle_ps)
		{
			_target_angle_ps = _max_angle_ps;
		}
		else if (angle_ps < _min_angle_ps)
		{
			_target_angle_ps = _min_angle_ps;
		}
		else
		{
			_target_angle_ps = angle_ps;
		}
	}
	else
	{
		_target_angle_ps = angle_ps;
	}
	return;
}

float driver_motor::get_target_angle_ps(void)
{
	return _target_angle_ps;
}

void driver_motor::set_control_period(float period)
{
	_ctrl_period = period;
	_sampling_period = _ctrl_period * 1e-6;
}

float driver_motor::get_current_angle_es()
{
	this->_encoder->poll_encoder_angle();
	_current_angle_es = this->_encoder->get_angle_multi();
	return _current_angle_es;
}

float driver_motor::get_current_angle_ps()
{
	this->_encoder->poll_encoder_angle();
	float temp = this->_encoder->get_angle_multi();
	if (_is_circular_joint)
	{
		temp = fmod(temp, _angle_full_turn_es);
	}
	return temp / _gear_ratio;
}

void driver_motor::closed_loop_control_tick()
{
	// IMPORTANT: es means encoder space
	// setpoint_es is the desired angle after gear ratio translation
	float setpoint_es = _target_angle_ps * _gear_ratio;

	// angle_multi_es is the current angle after gear ratio translation
	float angle_multi_es = get_current_angle_es();

	// For later, diff can influence PID coefficients
	float diff = setpoint_es - angle_multi_es;

	// Determine whether to move forward or backwards
	float forward_distance = (setpoint_es > angle_multi_es) ? (setpoint_es - angle_multi_es) : (_angle_full_turn_es - (angle_multi_es - setpoint_es));	// CCW
	float backward_distance = (setpoint_es > angle_multi_es) ? (_angle_full_turn_es - (setpoint_es - angle_multi_es)) : (angle_multi_es - setpoint_es); // CW

	// CAPPING ANGLE BY JOINT LIMITS, only for linear joints
	if (!_is_circular_joint)
	{
		if (setpoint_es > _max_angle_es)
		{
			setpoint_es = _max_angle_es;
		}
		else if (setpoint_es < _min_angle_es)
		{
			setpoint_es = _min_angle_es;
		}
	}

	// Feed to PID and determine error
	float pid_output = 0.0;
	if (_is_circular_joint)
	{
		// Backwards
		if (backward_distance < forward_distance - 10.0) // TODO: handle hysterics? 10.0 was used previously
		{
			// set_direction(!_forward_dir); // set direction to backwards
			if (setpoint_es < angle_multi_es)
			{
				pid_output = pid_instance->calculate(setpoint_es, angle_multi_es);
			}
			else
			{
				pid_output = pid_instance->calculate(setpoint_es + _angle_full_turn_es, angle_multi_es);
			}
		}
		// Forwards
		else
		{
			// set_direction(_forward_dir); // set direction to forwards
			if (setpoint_es > angle_multi_es)
			{
				pid_output = pid_instance->calculate(setpoint_es, angle_multi_es);
			}
			else
			{
				pid_output = pid_instance->calculate(setpoint_es - _angle_full_turn_es, angle_multi_es);
			}
		}
	}
	else // linear joint
	{
		pid_output = pid_instance->calculate(setpoint_es, angle_multi_es);
	}

	// Set direction
	if (pid_output > 0) // pos
	{
		set_direction(_forward_dir); // set direction to forwards
	}
	else // neg
	{
		set_direction(!_forward_dir); // set direction to backwards
	}

	// Output to motor
	// 255 is the max PWM value, 24 is the max voltage
	float pwm_output = abs(pid_output) * _PWM_OUTPUT_RESOLUTION / 24.0;

	this->_pwm_write_duty(pwm_output);
	return;
}

void driver_motor::torque_control(float motor_cur)
{

	bool direction = LOW;
	float motor_current;

	if (_forward_dir)
	{
		motor_current = motor_cur;
	}
	else
	{
		motor_current = -1 * motor_cur;
	}

	_kalman_gain = _err_estimate / (_err_estimate + _err_measure);
	_current_estimate = _last_estimate + _kalman_gain * (motor_current - _last_estimate);

	_err_estimate = (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
	_last_estimate = _current_estimate;

	_error = _target_torque - _current_estimate * _torque_constant_motor;
	_error_int += _error * _sampling_period;

	// Bound the integral error such that it cannot saturate the motors
	// The typical error is less than 1
	if (_error_int > _maxError)
	{
		_error_int = _maxError;
	}
	else if (_error_int < -_maxError)
	{
		_error_int = -_maxError;
	}

	// Calculate output
	_output_motor = _error * _KP + _error_int * _KI + _error_dir * _KD;

	if (_output_motor == _output_motor) // TODO: why are we comparing it to itself? A: Its broken
	{
		_output_motor *= _PWM_OUTPUT_RESOLUTION / _motor_max_torque;
	}
	else
	{
		_output_motor = 0.0;

		_error = 0.0;
		_previous_error = 0.0;

		_error_int = 0.0;
		_error_dir = 0.0;
	}

	// Set direction
	if (_forward_dir == 0)
	{
		direction = !direction;
	}

	if (_output_motor <= 0)
	{
		digitalWrite(_motor_dir_pin, direction);
	}
	else
	{
		digitalWrite(_motor_dir_pin, !direction);
	}

	// output torque
	_output_motor = abs(_output_motor);
	if (_output_motor > _PWM_OUTPUT_RESOLUTION)
	{
		_pwm_write_duty(_PWM_OUTPUT_RESOLUTION);
	}
	else
	{
		_pwm_write_duty((uint32_t)_output_motor);
	}
}

/// @brief Sets the resolution of the motor
/// @param resolution new resolution of the motor
void driver_motor::_pwm_set_resolution(uint16_t resolution)
{
	analogWriteResolution(resolution);
}

/// @brief Sets the PWM frequency of the motor
/// @param pwmPin Pin used for PWM modulation to the motor
/// @param pwmFreq Frequency to be set for PWM
void driver_motor::_pwm_setup(float pwmFreq)
{
	analogWriteFrequency(_motor_pwm_pin, pwmFreq);
}

/// @brief Writes the PWM value to the motor
/// @param pwmPin Pin used for PWM modulation to the motor
/// @param pwmDuty Duty cyle value to be written to PWM
void driver_motor::_pwm_write_duty(uint32_t pwmDuty)
{
	analogWrite(_motor_pwm_pin, pwmDuty);
}

void driver_motor::set_gear_ratio(float gear_ratio)
{
	_gear_ratio = gear_ratio;
	_angle_full_turn_es = 360.0f * _gear_ratio;
}

void driver_motor::set_is_circular_joint(bool is_circular_joint)
{
	_is_circular_joint = is_circular_joint;
}

/// @brief Define the forward logic level
/// @param forward_dir
void driver_motor::set_forward_dir(uint8_t forward_dir)
{
	_forward_dir = forward_dir;
}

void driver_motor::set_angle_limit_ps(float max_angle, float min_angle)
{
	_max_angle_ps = max_angle;
	_min_angle_ps = min_angle;
	_max_angle_es = max_angle * _gear_ratio;
	_min_angle_es = min_angle * _gear_ratio;
}

void driver_motor::set_direction(uint8_t direction)
{
	digitalWrite(_motor_dir_pin, direction);
}

void driver_motor::move_manual(float speed)
{
	uint32_t duty = (uint32_t)(abs(speed) * 255.0);
	if (speed < 0.0f)
	{
		digitalWrite(_motor_dir_pin, !_forward_dir);
		_pwm_write_duty(duty);
	}
	else
	{
		digitalWrite(_motor_dir_pin, _forward_dir);
		_pwm_write_duty(duty);
	}
}
