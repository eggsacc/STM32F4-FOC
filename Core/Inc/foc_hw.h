/*
 * foc.h
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_HW_H_
#define INC_FOC_HW_H_

/*
 * Includes
 */
#include <low_pass_filter.h>
#include "foc_utils.h"
#include "AS5600.h"
#include "pid.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f411xe.h"
#include "timer_utils.h"

/*
 * Typedef structures
 *
 * The library works by allocating (static) memory to each struct, then passing the struct pointers
 * around to update / use the stored values.
 *
 * The BLDCMotor structure contains motor parameters and pointers to all 3 other structures: Var_t, QDval_t, and PhaseV_t.
 * Also includes pointers to the sensor struct and timer handle struct.
 */
typedef struct
{
	float shaft_angle;
	uint32_t prev_us;
	uint16_t phase_current_buff[2];
} Var_t;

typedef struct
{
	float Uq;
	float Ud;
} DQ_t;

typedef struct
{
	float Ua;
	float Ub;
	float Uc;
} PV_t;

typedef struct
{
	float Ia;
	float Ib;
	float Ic;
} PI_t;


typedef enum
{
	error,
	none,
	open_loop_velocity,
	closed_loop_position_no_cs,
	closed_loop_velocity_no_cs,
	closed_loop_position,
	closed_loop_velocity
}Ctrl_t;

typedef struct
{
	/* Variables */
	int8_t sensor_dir;
	uint8_t pole_pairs;

	float voltage_limit;
	float supply_voltage;

	/* Target variables for serial commander */
	float target_velocity;
	float target_pos;
	float target_current;

	/* Current sense parameters */
	float cs_gain_a;
	float cs_gain_b;
	float cs_gain_c;
	uint16_t cs_offset_ia;
	uint16_t cs_offset_ib;

	/* Sub-structs */
	Var_t vars;
	DQ_t dq;
	PV_t pv;
	PI_t pi;
	PID_t pid;
	LPF_t lpf;
	Ctrl_t control;
	AS5600* sensor;
	TIM_HandleTypeDef* timer;
} BLDCMotor;

/*
 * Motors array
 */
extern BLDCMotor* BLDCMotorArray[2];

/*
 * Public functions
 */
void PWM_Start_3_Channel(TIM_HandleTypeDef* timer);
void BLDCMotor_Init(BLDCMotor* motor, TIM_HandleTypeDef* timer, uint8_t pole_pairs);
void LinkSensor(BLDCMotor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle);
//void BLDC_AutoCalibrate(BLDCMotor* motor);
void MotorDebug(BLDCMotor* motor);
void SetTorque(BLDCMotor* motor);
void BLDC_UpdateMotorADC_DMA();

#endif /* INC_FOC_HW_H_ */
