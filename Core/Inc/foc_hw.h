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
#include "fix16.h"

#define BLDC_Init(m, timer, pole_pairs) \
	Var_t m##_var;																							\
	DQ_t m##_dq;																							\
	PV_t m##_pv;\
	PID_t m##_pid;\
	LPF_t m##_lpf;\
	Ctrl_t m##_ctrl;\
	BLDCMotor_Init(&(m), &(m##_var), &(m##_dq), &(m##_pv), &(m##_pid), &(m##_lpf), &(m##_ctrl), &(timer), (pole_pairs))\


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
	fix16_t shaft_angle;
	uint32_t prev_us;
} Var_t;

typedef struct
{
	fix16_t Uq;
	fix16_t Ud;
} DQ_t;

typedef struct
{
	fix16_t Ua;
	fix16_t Ub;
	fix16_t Uc;
} PV_t;

typedef enum
{
	none,
	open_loop_velocity,
	closed_loop_position,
	closed_loop_velocity,
}Ctrl_t;

typedef struct
{
	/* Variables */
	int8_t sensor_dir;
	uint8_t pole_pairs;
	fix16_t voltage_limit;
	fix16_t supply_voltage;

	/* Target variables for serial commander */
	float target_velocity;
	float target_pos;


	/* Pointers to structs */
	Var_t* vars;
	DQ_t* dq;
	PV_t* pv;
	AS5600* sensor;
	TIM_HandleTypeDef* timer;
	PID_t* pid;
	LPF_t* lpf;
	Ctrl_t* control;
} BLDCMotor;

/*
 * Public functions
 */
void PWM_Start_3_Channel(TIM_HandleTypeDef* timer);
void BLDCMotor_Init(BLDCMotor* motor, Var_t* var, DQ_t* dq, PV_t* pv, PID_t* pid, LPF_t* lpf, Ctrl_t* ctrl, TIM_HandleTypeDef* timer, uint8_t pole_pairs);
void LinkSensor(BLDCMotor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle);
void BLDC_AutoCalibrate(BLDCMotor* motor);
void MotorDebug(BLDCMotor* motor);
void SetTorque(BLDCMotor* motor);

#endif /* INC_FOC_HW_H_ */
