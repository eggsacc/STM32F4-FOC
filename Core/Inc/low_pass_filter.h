/*
 * lpf.h
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#ifndef INC_LOW_PASS_FILTER_H_
#define INC_LOW_PASS_FILTER_H_

#include <stdint.h>

/*
 * LPF struct
 */
typedef struct
{
	float time_const;
	float prev_data;
	uint32_t prev_us;
}LPF_t;

/*
 * Fucntions
 */
float LPF_Compute(LPF_t* lpf, float x);
LPF_t LPF_Init();

#endif /* INC_LOW_PASS_FILTER_H_ */
