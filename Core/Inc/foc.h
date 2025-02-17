/*
 * foc.h
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

/*
 * Includes
 */
#include "foc_utils.h"
#include "AS5600.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f411xe.h"
#include "timer_utils.h"

/*
 * Constants
 */
#define KP 7.639437268

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
} Var_t;

typedef struct
{
	float Uq;
	float Ud;
} DQval_t;

typedef struct
{
	float Ua;
	float Ub;
	float Uc;
} PhaseV_t;

typedef struct
{
	/* Variables */
	int8_t sensor_dir;
	uint8_t pole_pairs;
	float voltage_limit;
	float supply_voltage;

	/* Pointers to structs */
	Var_t* vars;
	DQval_t* dqVals;
	PhaseV_t* phaseVs;
	AS5600* sensor;
	TIM_HandleTypeDef* timer;
} BLDCMotor;

/*
 * Public functions
 */
void PWM_Start_3_Channel(TIM_HandleTypeDef* timer);
void MotorInit(BLDCMotor* motor, TIM_HandleTypeDef* timer, float supply_voltage, uint8_t pole_pairs);
void LinkSensor(BLDCMotor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle);
void BLDC_AutoCalibrate(BLDCMotor* motor);
void MotorDebug(BLDCMotor* motor);

/*
 * Control functions
 */
void SetTorque(BLDCMotor* motor);
void OLVelocityControl(BLDCMotor* motor, float target_velocity);
void CLPositionControl(BLDCMotor* motor, float target_pos);


#endif /* INC_FOC_H_ */
