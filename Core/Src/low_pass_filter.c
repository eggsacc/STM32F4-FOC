/*
 * lp_filter.c
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#include <low_pass_filter.h>
#include "timer_utils.h"

/*
 * @brief Exponential moving average LPF
 * @param[in] LPF_t lpf
 * @param[in] float data
 * @retval float filtered_data
 */
float LPF_Compute(LPF_t* lpf_dev, float x)
{
	/* Get current time & calculate time difference */
	uint32_t now_us = micros();
	float dt = (now_us - lpf_dev->prev_us) * 0.000001f;

	/* Check if dt is reasonable; if not, discard value & return original x */
	if(dt < 0)
	{
		dt = 0.0001;
	}
	else if(dt > 0.2)
	{
		lpf_dev->prev_data = x;
		lpf_dev->prev_us = now_us;
		return x;
	}

	/* Calculate alpha value & filtered datapoint */
	float alpha = lpf_dev->time_const / (lpf_dev->time_const + dt);
	float y = alpha * lpf_dev->prev_data + (1.0f - alpha) * x;

	/* Update timestamps & previous data */
	lpf_dev->prev_data = y;
	lpf_dev->prev_us = now_us;

	return y;
}

/*
 * @brief LPF struct initialization function
 */
LPF_t LPF_Init()
{
	LPF_t lpf_dev = {
			.prev_data = 0,
			.prev_us = 0,
			.time_const = 0.1
	};

	return lpf_dev;
}

/*
 * @brief Configure filter time constant
 * @param[in] LPF_t* lpf
 * @param[in] float time_const
 * @retval -
 */
void LPF_ConfigureTimeConst(LPF_t* lpf_dev, float time_const)
{
	lpf_dev->time_const = time_const;
}
