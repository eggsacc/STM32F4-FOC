/*
 * pid.h
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

#define PID_LPF_ALPHA 0.1
/*
 * PID controller typedef
 */

typedef enum
{
	P,
	PI,
	PID
}PID_mode;

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

	/* Mode: 0 = P, 1 = PI, 2 = PID (default) */
	PID_mode mode;
}PID_t;

/*
 * Prototypes
 */
PID_t PID_Init();
float PID_Compute(PID_t* PID_dev, float setpoint, float input);
void PID_ConfigureParams(PID_t* PID_dev, float kp, float ki, float kd);
void PID_SetMode(PID_t* PID_dev, uint8_t mode);

#endif /* INC_PID_H_ */
