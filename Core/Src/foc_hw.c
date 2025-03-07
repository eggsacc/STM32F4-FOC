/*
 * foc.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

/*
 * Includes
 */
#include <low_pass_filter.h>
#include "main.h"
#include "foc_hw.h"
#include <math.h>
#include "stm32f4xx_hal_tim.h"
#include "stm32f411xe.h"
#include "foc_utils.h"
#include "AS5600.h"
#include "timer_utils.h"
#include "pid.h"

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
	fix16_t ARR = int_to_fix16(motor->timer->Instance->ARR);
	motor->timer->Instance->CCR1 = fix16_to_int(fix16_mul(fix16_div(motor->pv->Ua, motor->supply_voltage), ARR));
	motor->timer->Instance->CCR2 = fix16_to_int(fix16_mul(fix16_div(motor->pv->Ub, motor->supply_voltage), ARR));
	motor->timer->Instance->CCR3 = fix16_to_int(fix16_mul(fix16_div(motor->pv->Uc, motor->supply_voltage), ARR));
}

/*
 * @scope Static
 * @brief Initializers for motor sub-structs
 * @retval Struct_t struct
 */
static Var_t Var_t_Init()
{
	Var_t vars = {
		.shaft_angle = 0,
		.prev_us = 0
	};

	return vars;
}

static DQ_t DQ_t_Init()
{
	DQ_t dq = {
		.Ud = 0,
		.Uq = 0
	};

	return dq;
}

static PV_t PV_t_Init()
{
	PV_t pv = {
		.Ua = 0,
		.Ub = 0,
		.Uc = 0
	};

	return pv;
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
 * - BLDCMotor voltage is set to 12V bu default.
 * - Voltage limit is 3V by default.
 *
 * @retval BLDCMotor motor
 */
void BLDCMotor_Init(BLDCMotor* motor, Var_t* var, DQ_t* dq, PV_t* pv, PID_t* pid, LPF_t* lpf, Ctrl_t* ctrl, TIM_HandleTypeDef* timer, uint8_t pole_pairs)
{
	/* De-reference & initialize sub-structs */
	*var = Var_t_Init();
	*dq = DQ_t_Init();
	*pv = PV_t_Init();
	*pid = PID_Init();
	*lpf = LPF_Init();
	*ctrl = none;

	/* BLDCMotor struct initialization */
	motor->vars = var;
	motor->dq = dq;
	motor->pv = pv;
	motor->sensor = NULL;
	motor->timer = timer;
	motor->pid = pid;
	motor->lpf = lpf;
	motor->control = ctrl;
	motor->sensor_dir = 1;
	motor->pole_pairs = pole_pairs;
	motor->target_velocity = 0;
	motor->target_pos = 0;
	motor->supply_voltage = int_to_fix16(12);
	motor->voltage_limit = int_to_fix16(3);
}

/*
 * @brief Inverse Clarke & Park transformations to calculate phase voltages
 * @param[in] BLDCMotor* motor
 * @note Calls setpwm()
 */
void SetTorque(BLDCMotor* motor) {
	/* Constrain Uq to within voltage range */
	motor->dq->Uq = _constrain(motor->dq->Uq, -motor->voltage_limit, motor->voltage_limit);
    /* Normalize electric angle */
	/* Note that _normalizeAngle() works with floats, not fix16 */
    fix16_t el_angle = _normalizeAngle(_electricalAngle(motor->vars->shaft_angle, motor->pole_pairs));

	/* Inverse park transform */
	fix16_t Ualpha = fix16_mul(motor->dq->Uq, _cos(el_angle));
	fix16_t Ubeta = fix16_mul(motor->dq->Uq, _sin(el_angle));

	fix16_t half_supply_v = motor->supply_voltage >> 1;
	fix16_t sqrt3_beta = fix16_mul(FIX16_SQRT3, Ubeta);

	/* Inverse Clarke transform */
	motor->pv->Ua = Ualpha + half_supply_v;
	/* (_SQRT3 * Ubeta - Ualpha) / 2.0f */
//	motor->pv->Ub = ((sqrt3_beta - Ualpha) >> 1) + half_supply_v;
	motor->pv->Ub = fix16_mul(FIX16_HALF, (sqrt3_beta - Ualpha)) + half_supply_v;
	/* (- Ualpha - _SQRT3 * Ubeta) / 2.0f */
//	motor->pv->Uc = ((-Ualpha - sqrt3_beta) >> 1) + half_supply_v;
	motor->pv->Uc = fix16_mul(FIX16_HALF, (-Ualpha - sqrt3_beta)) + half_supply_v;
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

	motor->dq->Uq = motor->voltage_limit >> 1;
	motor->vars->shaft_angle = FIX16_PI_2;
	SetTorque(motor);
	HAL_Delay(1500);
	AS5600_ZeroAngle(sensor);
	motor->dq->Uq = 0;
	motor->vars->shaft_angle = 0;
	SetTorque(motor);
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
	motor->dq->Uq = motor->supply_voltage >> 2;
	motor->pole_pairs = 1;
	motor->vars->shaft_angle = 0;
	SetTorque(motor);
	HAL_Delay(1500);
	AS5600_ZeroAngle(motor->sensor);

	/* Take 3 repeated mechanical angle readings */
	for(int i = 0; i < 3; i++)
	{
		motor->vars->shaft_angle = FIX16_PI;
		SetTorque(motor);
		HAL_Delay(1500);
		angle_a = AS5600_ReadAngle(motor->sensor);
		HAL_Delay(500);

		motor->vars->shaft_angle = FIX16_3PI_2;
		SetTorque(motor);
		HAL_Delay(1500);
		angle_b = AS5600_ReadAngle(motor->sensor);
		HAL_Delay(500);

		total += angle_b - angle_a;
	}

	/* Calculate average mechanical angle delta */
	float avg_delta = total / 5;

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
	motor->dq->Uq = 0;
	SetTorque(motor);
}

/*
 * @brief Debug motor parameters
 * @params[in] BLDCMotor* motor
 */
//void MotorDebug(BLDCMotor* motor)
//{
//	sprintf(usb_tx_buffer, "Sensor dir: %d,\nPole pairs: %d,\nSensor: %p,\nTimer: %p\n",
//							motor->sensor_dir,
//							motor->pole_pairs,
//							motor->sensor,
//							motor->timer);
//	CDC_Transmit_FS(usb_tx_buffer, strlen((const char*)usb_tx_buffer));
//}


