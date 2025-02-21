/*
 * lpf.h
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#ifndef INC_LPF_H_
#define INC_LPF_H_

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
float LPF_Filter(LPF_t* lpf, float x);
LPF_t LPF_Init();

#endif /* INC_LPF_H_ */
