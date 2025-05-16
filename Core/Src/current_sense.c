/*
 * current_sense.c
 *
 *  Created on: Feb 23, 2025
 *      Author: yizha
 */

#include "current_sense.h"

uint16_t ADC_buff[4] = {0};

/*
 * @brief periodic interrupt callback function to update motor struct ADC values from ADC buffer
 */
void BLDC_UpdateMotorADC_DMA()
{
	if(BLDCMotorArray[0] != NULL)
	{
		BLDCMotorArray[0]->vars.phase_current_buff[0] = ADC_buff[0];
		BLDCMotorArray[0]->vars.phase_current_buff[1] = ADC_buff[1];
	}
	if(BLDCMotorArray[1] != NULL)
	{
		BLDCMotorArray[1]->vars.phase_current_buff[0] = ADC_buff[2];
		BLDCMotorArray[1]->vars.phase_current_buff[1] = ADC_buff[3];
	}
}

/*
 * @brief Calibrate current sense amplifier gain offset
 * @param[in] BLDCMotor* motor
 * @retval -
 */
static void CS_CalibrateAmpOffset(BLDCMotor* motor)
{
	/* Set torque to 0 */
	motor->dq.Uq = 0;
	SetTorque(motor);

	uint32_t offset_ia = 0;
	uint32_t offset_ib = 0;

	HAL_ADC_Start_DMA(&hadc1, ADC_buff, 4);

	/* Disable all PWM channels during calibration */
	motor->timer->Instance->CCR1 = 0;
	motor->timer->Instance->CCR2 = 0;
	motor->timer->Instance->CCR3 = 0;

	HAL_Delay(5);

	/* Run calibration for defined number of iterations */
	for(int i = 0; i < CS_OFFSET_CALIBRATION_TRIALS; i++)
	{
		offset_ia += motor->vars.phase_current_buff[0];
		offset_ib += motor->vars.phase_current_buff[1];
		HAL_Delay(1);
	}

	/* Get average offset */
	motor->cs_offset_ia = offset_ia / CS_OFFSET_CALIBRATION_TRIALS;
	motor->cs_offset_ib = offset_ib / CS_OFFSET_CALIBRATION_TRIALS;
}

/*
 * @brief Initialize current sense parameters in motor struct
 * @param[in] BLDCMotor* motor
 * @retval -
 */
void CS_Init(BLDCMotor* motor)
{
	float adc_raw_to_float = ADC_VREF / (float)(ADC_RESOLUTION - 1);
	float v_to_i_ratio = 1.0f / SHUNT_RESISTANCE / AMP_GAIN;
	motor->cs_gain_a = v_to_i_ratio * adc_raw_to_float;
	motor->cs_gain_b = v_to_i_ratio * adc_raw_to_float;
	motor->cs_gain_c = v_to_i_ratio * adc_raw_to_float;
	CS_CalibrateAmpOffset(motor);
}

/*
 * @brief Compute phase currents and store it back in motor struct
 * @param[in] BLDCMotor* motor
 * @note Only meaasures phase A, B current, phase C current is deduced not measured.
 */
void CS_SamplePhaseCurrents(BLDCMotor* motor)
{
	motor->pi.Ia = (float)(motor->vars.phase_current_buff[0] - motor->cs_offset_ia) * motor->cs_gain_a;
	motor->pi.Ib = (float)(motor->vars.phase_current_buff[1] - motor->cs_offset_ib) * motor->cs_gain_b;
	motor->pi.Ic = motor->pi.Ia + motor->pi.Ib;
}

static float CS_ComputeQD(BLDCMotor* motor)
{
	float Ia = motor->pi.Ia;
	float Ib = _1_SQRT3 * motor->pi.Ia + _2_SQRT3 * motor->pi.Ib;

	float ct = _cos(_normalizeAngle(_electricalAngle(motor->vars.shaft_angle, motor->pole_pairs)));
	float st = _sin(_normalizeAngle(_electricalAngle(motor->vars.shaft_angle, motor->pole_pairs)));

	float Iq = Ib * ct - Ia * st;
	return Iq;
}

float CS_SetTorque(BLDCMotor* motor)
{
	CS_SamplePhaseCurrents(motor);
	float Iq = CS_ComputeQD(motor);
	//motor->dq.Uq = PID_Compute(&(motor->pid), motor->target_current, Iq);
	//motor->vars.shaft_angle = AS5600_GetAngle(motor->sensor);
	//SetTorque(motor);
	return Iq;
}
