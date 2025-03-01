/*
 * foc_driver.c
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#include <low_pass_filter.h>
#include "foc_hw.h"
#include "foc_core.h"
#include "pid.h"

/*
 * @brief Open-loop velocity control
 * @param[in] BLDCMotor* motor
 * @param[in] float target_velocity (rads/sec)
 * @warning Ensure DWT_Init() is called in main() to initialize timer first.
 */
void OLVelocityControl(BLDCMotor* motor, float target_velocity)
{
	/* Check if motor timer initialized properly */
	if(motor->timer == NULL)
	{
		return;
	}

	/* Track current micros */
	uint32_t now_us = micros();

	/* Time difference since last call */
	uint32_t time_elapsed_us = now_us - motor->vars->prev_us;
	float time_elapsed_s = time_elapsed_us  / 1000000.0f;
	time_elapsed_s = time_elapsed_s > 0.5 ? 0.0005 : time_elapsed_s;

	/* Update virtual shaft angle, and calculate phase voltages */
	motor->vars->shaft_angle = _normalizeAngle(motor->vars->shaft_angle + float_to_fix16(target_velocity * time_elapsed_s));
	motor->dq->Uq = motor->voltage_limit;
	SetTorque(motor);

	/* Update timestamp */
	motor->vars->prev_us = now_us;
}

/*
 * @brief Closed loop position control
 * @param[in] BLDCMotor* motor
 * @param[in] float target_pos (in radians)
 * @note Returns if sensor not attached
 * @note Uses P control of PID
 */
void CLPositionControl(BLDCMotor* motor, float target_pos)
{
	/* Check if sensor is attached to motor */
	if(motor->sensor == NULL)
	{
		return;
	}

	/* Sets the PID controller to P-mode */
	if(motor->pid->mode != 0)
	{
		motor->pid->mode = 0;
	}

	/* Electrical angle calculated based on sensor angle * pole pairs */
	motor->vars->shaft_angle = (float)(motor->sensor_dir * AS5600_ReadAngle(motor->sensor));
	/* KP sets max torque when shaft is 45deg (0.7854 rad) off from target pos */
	motor->dq->Uq = PID_Compute(motor->pid, target_pos, motor->sensor_dir * motor->vars->shaft_angle);
	SetTorque(motor);
}

/*
 * @brief Closed loop velocity control
 * @param[in] BLDCmotor
 * @param[in] target velocity
 * @retval -
 * @note Returns if sensor not attached
 * @note Uses PI control of PID
 */
void CLVelocityControl(BLDCMotor* motor, float target_velocity)
{
	/* Check if sensor is attached */
	if(motor->sensor == NULL)
	{
		return;
	}

	/* Set PID controller to PI-mode */
	if(motor->pid->mode != 1)
	{
		motor->pid->mode = 1;
	}

	/* Compute torque based on velocity error */
	motor->dq->Uq = PID_Compute(motor->pid, target_velocity, LPF_Compute(motor->lpf, AS5600_GetVelocity(motor->sensor)));
	motor->vars->shaft_angle = AS5600_ReadAngle(motor->sensor);
	SetTorque(motor);
}
