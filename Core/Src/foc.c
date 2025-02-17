/*
 * foc.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

/*
 * Includes
 */
#include <stm32f4xx_hal.h>
#include <stdlib.h>
#include <math.h>
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f411xe.h"
#include "foc_utils.h"
#include "foc.h"
#include "AS5600.h"
#include "timer_utils.h"

/*
 * Extern variable
 */
static uint8_t usb_tx_buffer[64];

/*
 * @brief Starts PWM channels 1, 2, 3 of specified timer.
 * @param[in] TIM_HandleTypeDef timer
 */
void PWM_Start_3_Channel(TIM_HandleTypeDef* timer)
{
	HAL_TIM_PWM_Start(timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(timer, TIM_CHANNEL_3);
}

/*
 * @scope static
 * @brief Set pwm duty cycle of timer channels
 * @param[in] BLDCMotor* motor
 */
__STATIC_INLINE void SetPWM(BLDCMotor* motor)
{
	motor->timer->Instance->CCR1 = _constrain(motor->phaseVs->Ua / motor->supply_voltage, 0.0f, 1.0f) * 256;
	motor->timer->Instance->CCR2 = _constrain(motor->phaseVs->Ub / motor->supply_voltage, 0.0f, 1.0f) * 256;
	motor->timer->Instance->CCR3 = _constrain(motor->phaseVs->Uc / motor->supply_voltage, 0.0f, 1.0f) * 256;
}

/*
 * @brief Initializes motor object.
 *        Creates all relevant structs and returns main motor struct.
 *
 * @param[in] TIM_HandleTypeDef* timer
 * @param[in] float supply_voltage
 * @param[in] uint8_t pole_pairs
 *
 * @note
 * - Sensor pointer is NULL by default (no sensor)
 * - BLDCMotor voltage limit set to supply voltage / 2 by default
 *
 * @retval BLDCMotor motor
 */
void MotorInit(BLDCMotor* motor, TIM_HandleTypeDef* timer, float supply_voltage, uint8_t pole_pairs)
{
	/* Create & initialize structs */
	static Var_t motor_vars = {
			.shaft_angle = 0,
			.prev_us = 0
	};

	static DQval_t motor_dq = {
			.Ud = 0,
			.Uq = 0
	};
	static PhaseV_t motor_pv = {
			.Ua = 0,
			.Ub = 0,
			.Uc = 0
	};

	/* Timer utility init */
	DWT_Init();

	/* BLDCMotor struct initialization */
	motor->dqVals = &motor_dq;
	motor->phaseVs = &motor_pv;
	motor->vars = &motor_vars;
	motor->sensor = NULL;
	motor->timer = timer;
	motor->sensor_dir = 0;
	motor->pole_pairs = pole_pairs;
	motor->supply_voltage = supply_voltage;
	motor->voltage_limit = supply_voltage / 2;
}

/*
 * @brief Inverse Clarke & Park transformations to calculate phase voltages
 * @param[in] BLDCMotor* motor
 * @note Calls setpwm()
 */
void SetTorque(BLDCMotor* motor) {
	/* Constrain Uq to within voltage range */
	motor->dqVals->Uq = _constrain(motor->dqVals->Uq, -motor->voltage_limit, motor->voltage_limit);
    /* Normalize electric angle */
    float el_angle = _normalizeAngle(_electricalAngle(motor->vars->shaft_angle, motor->pole_pairs));

	/* Inverse park transform */
	float Ualpha = -(motor->dqVals->Uq) * _sin(el_angle);
	float Ubeta = motor->dqVals->Uq * _cos(el_angle);

	/* Inverse Clarke transform */
	motor->phaseVs->Ua = Ualpha + motor->supply_voltage / 2.0f;
	motor->phaseVs->Ub = (_SQRT3 * Ubeta - Ualpha) / 2.0f + motor->supply_voltage / 2.0f;
	motor->phaseVs->Uc = (- Ualpha - _SQRT3 * Ubeta) / 2.0f + motor->supply_voltage / 2.0f;

	SetPWM(motor);
}

/*
 * @brief Links a AS5600 sensor to a motor object
 * @param[in] BLDCMotor* motor
 * @param[in] AS5600* sensor
 * @param[in] I2C_HandleTypeDef *i2c_handle
 */
void LinkSensor(BLDCMotor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle)
{
	uint8_t init_stat = AS5600_Init(sensor, i2c_handle, 1);

	/* Check if sensor link successful */
	if(init_stat != 0)
	{
		motor->sensor = NULL;
		return;
	}

	motor->sensor = sensor;
}

/*
 * @brief Automatically determine the pole pair & sensor direction of BLDC motor
 * @param[in] BLDCmotor* motor
 * @warning Does nothing if sensor or timer not attached to motor.
 */
void BLDC_AutoCalibrate(BLDCMotor* motor)
{
	/* Check if encoder & timer attached */
	if(motor->sensor == NULL || motor->timer == NULL)
	{
		return;
	}

	/* Mechanical angles */
	float angle_a = 0;
	float angle_b = 0;
	float total = 0;

	/* Set some current & electrical angle to 0 */
	motor->dqVals->Uq = motor->supply_voltage / 3;
	motor->pole_pairs = 1;
	motor->vars->shaft_angle = 0;
	SetTorque(motor);
	AS5600_ZeroAngle(motor->sensor);

	/* Take 3 repeated mechanical angle readings */
	for(int i = 0; i < 3; i++)
	{
		motor->vars->shaft_angle = _PI;
		SetTorque(motor);
		HAL_Delay(1500);
		angle_a = AS5600_ReadNormalizedAngle(motor->sensor);
		HAL_Delay(500);

		motor->vars->shaft_angle = _3PI_2;
		SetTorque(motor);
		HAL_Delay(1500);
		angle_b = AS5600_ReadNormalizedAngle(motor->sensor);
		HAL_Delay(500);

		total += angle_b - angle_a;
	}

	/* Calculate average mechanical angle delta */
	float avg_delta = total / 3;

	/* Set sensor angle inverter */
	motor->sensor_dir = avg_delta > 0 ? 1 : -1;

	if(avg_delta != 0)
	{
		uint8_t pole_pairs = round(_PI_2 / _abs(avg_delta));

		/* Check if pole pair calculation is reasonable */
		if(pole_pairs >= 5 || pole_pairs <= 14)
		{
			motor->pole_pairs = pole_pairs;
		}
	}

	/* Set motor back to 0 angle */
	motor->vars->shaft_angle = 0;
	SetTorque(motor);

	HAL_Delay(1000);

	AS5600_ZeroAngle(motor->sensor);

	/* Set 0 torque */
	motor->dqVals->Uq = 0;
	SetTorque(motor);
}

/*
 * @brief Debug motor parameters
 * @params[in] BLDCMotor* motor
 */
void MotorDebug(BLDCMotor* motor)
{
	sprintf(usb_tx_buffer, "Sensor dir: %d,\nPole pairs: %d,\nSensor: %p,\nTimer: %p\n",
							motor->sensor_dir,
							motor->pole_pairs,
							motor->sensor,
							motor->timer);
	CDC_Transmit_FS(usb_tx_buffer, strlen((const char*)usb_tx_buffer));
}

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
	time_elapsed_s = time_elapsed_s > 0.5 ? 0.001 : time_elapsed_s;

	/* Update virtual shaft angle, and calculate phase voltages */
	motor->vars->shaft_angle = _normalizeAngle(motor->vars->shaft_angle + target_velocity * time_elapsed_s);
	motor->dqVals->Uq = motor->voltage_limit;
	SetTorque(motor);

	/* Update timestamp */
	motor->vars->prev_us = micros();
}

/*
 * @brief Closed loop position control
 * @param[in] BLDCMotor* motor
 * @param[in] float target_pos (in radians)
 * @note Does nothing if no sensor is attached to the motor.
 */
void CLPositionControl(BLDCMotor* motor, float target_pos)
{
	/* Check if sensor is attached to motor */
	if(motor->sensor == NULL)
	{
		return;
	}

	/* Electrical angle calculated based on sensor angle * pole pairs */
	motor->vars->shaft_angle = (float)(motor->sensor_dir * AS5600_ReadAngle(motor->sensor));
	/* KP sets max torque when shaft is 45deg (0.7854 rad) off from target pos */
	motor->dqVals->Uq = KP * (target_pos - (motor->sensor_dir * motor->vars->shaft_angle));
	SetTorque(motor);
}
