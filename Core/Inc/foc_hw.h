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

#define BLDC_Init(m, timer, pole_pairs) \
	Var_t m##_var;																							\
	DQ_t m##_dq;																							\
	PV_t m##_pv;\
	PID_t m##_pid;\
	LPF_t m##_lpf;\
	BLDCMotor_Init(&(m), &(m##_var), &(m##_dq), &(m##_pv), &(m##_pid), &(m##_lpf), &(timer), (pole_pairs))\


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
} DQ_t;

typedef struct
{
	float Ua;
	float Ub;
	float Uc;
} PV_t;

typedef enum
{
	open_loop_vel,
	closed_loop_pos,
	closed_loop_vel,

}Ctrl_t;
typedef struct
{
	/* Variables */
	int8_t sensor_dir;
	uint8_t pole_pairs;
	float voltage_limit;
	float supply_voltage;

	/* Pointers to structs */
	Var_t* vars;
	DQ_t* dq;
	PV_t* pv;
	AS5600* sensor;
	TIM_HandleTypeDef* timer;
	PID_t* pid;
	LPF_t* lpf;
} BLDCMotor;

/*
 * Public functions
 */
void PWM_Start_3_Channel(TIM_HandleTypeDef* timer);
void BLDCMotor_Init(BLDCMotor* motor, Var_t* var, DQ_t* dq, PV_t* pv, PID_t* pid, LPF_t* lpf, TIM_HandleTypeDef* timer, uint8_t pole_pairs);
void LinkSensor(BLDCMotor* motor, AS5600* sensor, I2C_HandleTypeDef *i2c_handle);
void BLDC_AutoCalibrate(BLDCMotor* motor);
void MotorDebug(BLDCMotor* motor);
void SetTorque(BLDCMotor* motor);

#endif /* INC_FOC_HW_H_ */
