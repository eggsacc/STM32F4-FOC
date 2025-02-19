/*
 * pid.h
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

#define LPF_ALPHA 0.1
/*
 * PID controller typedef
 */
typedef struct
{
	float kp;
	float ki;
	float kd;

	float integral;
	float last_error;
	float last_derivative;

	float lower_bound;
	float upper_bound;

	uint32_t timestamp_us;

	/* Selector: 0 = P, 1 = PI, 2 = PID (default) */
	uint8_t selector;
}PID_t;

/*
 * Prototypes
 */
float PID_Compute(float setpoint, float input);
void PID_ConfigureParams(float kp, float ki, float kd);

#endif /* INC_PID_H_ */
