/*
 * pid.c
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#include "pid.h"
#include "timer_utils.h"
#include "foc_utils.h"
#include <stdint.h>


/*
 * @brief Create and initialize PID structure
 * @params[in] float kp
 * @params[in] float ki
 * @params[in] float kd
 * @retval PID structure
 */
PID_t PID_Init()
{
	PID_t PID_dev =
	{
		.kp = 1.0f,
		.ki = 0.1f,
		.kd = 0.01f,
		.integral = 0,
		.last_error = 0,
		.last_derivative = 0,
		.lower_bound = -10,
		.upper_bound = 10,
		.timestamp_us = 0,
		.mode = P
	};

	return PID_dev;
}

/*
 * @brief Computes PID output for input & setpoint
 * @param[in] float setpoint
 * @param[in] float input value
 * @retval float PID output
 */
float PID_Compute(PID_t* PID_dev, float setpoint, float input)
{
	uint32_t now_us = micros();

	float error = setpoint - input;

	/* Save time delta as uint32_t first to handle overflows naturally */
	uint32_t dt_us = now_us - PID_dev->timestamp_us;
	float dt = dt_us * 0.000001;

	/* Proportional term calculation */
	float p_term = PID_dev->kp * error;

	/* If time delta is unreasonable, only return proportional term since it is not time-based */
	if(PID_dev->mode == P || dt <= 0 || dt > 0.2)
	{
		PID_dev->timestamp_us = now_us;
		return p_term;
	}

	/* Integral term calculation */
	float i_term = 0;

	if(PID_dev->mode == PI)
	{
		/* Accumulate integral value using Riemann midpoint rule */
		PID_dev->integral += PID_dev->ki * dt * 0.5f * (error + PID_dev->last_error);
		PID_dev->integral = _constrain(PID_dev->integral, PID_dev->lower_bound, PID_dev->upper_bound);
		i_term = PID_dev->integral;
	}

	/* Derivative term calculation */
	float d_term = 0;

	if(PID_dev->mode == PID)
	{
		/* If dt is too small, set to a reasonable value to avoid division by extremely small numbers */
		if(dt < 0.00001f)
		{
			dt = 0.00001f;
		}

		/* Check error difference & apply simple LPF */
		d_term = (error - PID_dev->last_error) / dt;
		d_term = PID_LPF_ALPHA * d_term + (1 - PID_LPF_ALPHA) * PID_dev->last_derivative;
		PID_dev->last_derivative = d_term;
	}

	PID_dev->last_error = error;
	PID_dev->timestamp_us = now_us;

	return p_term + i_term + d_term;
}

/*
 * @brief Configure PID parameters
 * @param[in] float kp
 * @param[in] float ki
 * @param[in] float kd
 * @retval None
 */
void PID_ConfigureParams(PID_t* PID_dev, float kp, float ki, float kd)
{
	PID_dev->kp = kp;
	PID_dev->ki = ki;
	PID_dev->kd = kd;
}

/*
 * @brief Set the controller mode (which terms to apply)
 * 		  0 = P, 1 = PI, 2 = PID (default)
 * @param[in] uint8_t mode
 * @retval None
 */
void PID_SetMode(PID_t* PID_dev, uint8_t mode)
{
	if(mode == 0 || mode == 1 || mode == 2)
	{
		PID_dev->mode = mode;
	}
}
