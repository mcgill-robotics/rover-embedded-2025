/*
 * stspin948.c
 *
 *  Created on: Jun 28, 2025
 *      Author: vince
 */

#include "stspin948.h"

static uint32_t get_us() {
	return (uint32_t) ((uint64_t) DWT->CYCCNT * 1000000 / HAL_RCC_GetHCLKFreq());
}

float angular_error_continuous(float setpoint_deg,
		float position_continuous_deg) {
	float setpoint_wrapped = fmodf(setpoint_deg, 360.0f);
	if (setpoint_wrapped < 0.0f)
		setpoint_wrapped += 360.0f;

	float position_wrapped = fmodf(position_continuous_deg, 360.0f);
	if (position_wrapped < 0.0f)
		position_wrapped += 360.0f;

	float error = setpoint_wrapped - position_wrapped;

	// Wrap to (-180, 180]
	if (error > 180.0f)
		error -= 360.0f;
	if (error <= -180.0f)
		error += 360.0f;

	return error;
}

void STSPIN948_Init(BrushedDriver *driverInstance) {
	if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
	driverInstance->last_us = get_us();
	HAL_TIM_PWM_Start(driverInstance->config->pwm_a_inst,
			driverInstance->config->pwm_a_channel);
	HAL_TIM_PWM_Start(driverInstance->config->pwm_a_inst,
			driverInstance->config->pwm_a_channel);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

//DAC Outputs don't change
	HAL_DAC_Start(driverInstance->config->dacRef_b_inst,
			driverInstance->config->dacRef_b_channel);
	HAL_Delay(3);
	HAL_DAC_Start(driverInstance->config->dacRef_a_inst,
			driverInstance->config->dacRef_a_channel);
	HAL_Delay(3);
	HAL_DAC_SetValue(driverInstance->config->dacRef_a_inst,
			driverInstance->config->dacRef_a_channel, DAC_ALIGN_12B_R,
			driverInstance->config->dacRef_a);
	HAL_DAC_SetValue(driverInstance->config->dacRef_b_inst,
			driverInstance->config->dacRef_b_channel, DAC_ALIGN_12B_R,
			driverInstance->config->dacRef_b);

	STSPIN948_SetOutputs(driverInstance);
	STSPIN948_ReadInputs(driverInstance);
}

void STSPIN948_ReadInputs(BrushedDriver *driverInstance) {

	// update driver enable inputs
	driverInstance->enFault_a = HAL_GPIO_ReadPin(
			driverInstance->config->enFault_a_port,
			driverInstance->config->enFault_a_pin);
	driverInstance->enFault_b = HAL_GPIO_ReadPin(
			driverInstance->config->enFault_b_port,
			driverInstance->config->enFault_b_pin);

// update currents
	driverInstance->cur_a = (((float) driverInstance->raw_cur.u16[0] - 2047.0)
			/ 4095.0) * driverInstance->config->cur_a_factor;
	driverInstance->cur_b = ((float) driverInstance->raw_cur.u16[1] / 4095.0)
			* driverInstance->config->cur_b_factor;
}

void STSPIN948_SetOutputs(BrushedDriver *driverInstance) {
	__HAL_TIM_SET_COMPARE(driverInstance->config->pwm_a_inst,
			driverInstance->config->pwm_a_channel, driverInstance->pwm_a);
	__HAL_TIM_SET_COMPARE(driverInstance->config->pwm_b_inst,
			driverInstance->config->pwm_b_channel, driverInstance->pwm_b);
	HAL_GPIO_WritePin(driverInstance->config->phase_a_port,
			driverInstance->config->phase_a_pin, !driverInstance->phase_a);
	HAL_GPIO_WritePin(driverInstance->config->phase_b_port,
			driverInstance->config->phase_b_pin, !driverInstance->phase_b);
}

void STSPIN948_SetPwmValues(BrushedDriver *driverInstance, uint32_t pwm_a,
		uint32_t pwm_b) {

	if (pwm_a > MAX_PWM_VALUE) {
		driverInstance->pwm_a = MAX_PWM_VALUE;
	} else {
		driverInstance->pwm_a = pwm_a;
	}
	if (pwm_b > MAX_PWM_VALUE) {
		driverInstance->pwm_b = MAX_PWM_VALUE;
	} else {
		driverInstance->pwm_b = pwm_b;
	}
}

void STSPIN948_SetDirections(BrushedDriver *driverInstance, uint8_t phase_a,
		uint8_t phase_b) {
	if (phase_a > 1) {
		driverInstance->phase_a = 1;
	} else {
		driverInstance->phase_a = phase_a;
	}
	if (phase_b > 1) {
		driverInstance->phase_b = 1;
	} else {
		driverInstance->phase_b = phase_a;
	}
}

void STSPIN948_CalculatePID(BrushedDriver *driverInstance, float pos_a,
		float pos_b) {
	float dt = (get_us() - driverInstance->last_us) / 1000000.0;
	driverInstance->pid_a.error = angular_error_continuous(
			driverInstance->pid_a.setpoint, pos_a);
	driverInstance->pid_b.error = driverInstance->pid_b.setpoint - pos_b;

	driverInstance->pid_a.integral += driverInstance->pid_a.error * dt;
	driverInstance->pid_b.integral += driverInstance->pid_b.error * dt;

//	if (driverInstance->pid_a.integral > INTEGRAL_MAX)
//		driverInstance->pid_a.integral = INTEGRAL_MAX;
//	else if (driverInstance->pid_a.integral < -INTEGRAL_MAX)
//		driverInstance->pid_a.integral = -INTEGRAL_MAX;
//
//	if (driverInstance->pid_b.integral > INTEGRAL_MAX)
//		driverInstance->pid_b.integral = INTEGRAL_MAX;
//	else if (driverInstance->pid_b.integral < -INTEGRAL_MAX)
//		driverInstance->pid_b.integral = -INTEGRAL_MAX;

	driverInstance->pid_a.derivative = (driverInstance->pid_a.error
			- driverInstance->pid_a.prev_error) * dt;
	driverInstance->pid_b.derivative = (driverInstance->pid_b.error
			- driverInstance->pid_b.prev_error) * dt;

	driverInstance->pid_a.output = driverInstance->config->kp_a
			* driverInstance->pid_a.error;  // proportional
	driverInstance->pid_a.output += driverInstance->config->ki_a
			* driverInstance->pid_a.integral;  //integral
	driverInstance->pid_a.output += driverInstance->config->kd_a
			* driverInstance->pid_a.derivative;  //derivative

	driverInstance->pid_b.output = driverInstance->config->kp_b
			* driverInstance->pid_b.error;  // proportional
	driverInstance->pid_b.output += driverInstance->config->ki_b
			* driverInstance->pid_b.integral;  //integral
	driverInstance->pid_b.output += driverInstance->config->kd_b
			* driverInstance->pid_b.derivative;  //derivative

	STSPIN948_SetPwmValues(driverInstance,
			(uint32_t) fabsf(driverInstance->pid_a.output),
			(uint32_t) fabsf(driverInstance->pid_b.output));
	STSPIN948_SetDirections(driverInstance,
			(uint8_t) (driverInstance->pid_a.output >= 0),
			(uint8_t) (driverInstance->pid_b.output >= 0));

	driverInstance->pid_a.prev_error = driverInstance->pid_a.error;
	driverInstance->pid_b.prev_error = driverInstance->pid_b.error;
	driverInstance->last_us = get_us();
}

void STSPIN948_ResetPID(BrushedDriver *driverInstance) {
	driverInstance->pid_a.derivative = 0;
	driverInstance->pid_a.integral = 0;
	driverInstance->pid_a.error = 0;
	driverInstance->pid_a.prev_error = 0;
	driverInstance->pid_a.output = 0;

	driverInstance->pid_b.derivative = 0;
	driverInstance->pid_b.integral = 0;
	driverInstance->pid_b.error = 0;
	driverInstance->pid_b.prev_error = 0;
	driverInstance->pid_b.output = 0;
}
