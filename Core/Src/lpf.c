/*
 * lp_filter.c
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#include "lpf.h"
#include "timer_utils.h"

/*
 * Static variables
 */
static uint32_t LPF_prev_timestamp = 0;
static float LPF_prev_data = 0;

/*
 * @brief Exponential moving average LPF
 * @param[in] float data
 * @retval float filtered data
 */
float LowPassFilter(float x)
{
	/* Get current time & calculate time difference */
	uint32_t now_us = micros();
	float dt = (now_us - LPF_prev_timestamp) * 0.000001f;

	/* Check if dt is reasonable; if not, discard value & return original x */
	if(dt < 0)
	{
		dt = 0.0001;
	}
	else if(dt > 0.2)
	{
		LPF_prev_data = x;
		LPF_prev_timestamp = now_us;
		return x;
	}

	/* Calculate alpha value & filtered datapoint */
	float alpha = TIME_CONSTANT / (TIME_CONSTANT + dt);
	float y = alpha * LPF_prev_data + (1.0f - alpha) * x;

	/* Update timestamps & previous data */
	LPF_prev_data = y;
	LPF_prev_timestamp = now_us;

	return y;
}
