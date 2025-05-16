/*
 * serialCommander.c
 *
 *  Created on: Feb 1, 2025
 *      Author: yizha
 */

#include "main.h"
#include "serial_commander.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct
{
	BLDCMotor* m0;
	BLDCMotor* m1;
	UART_HandleTypeDef* uart;
	char rx_buff[32];
	char tx_buff[32];
	uint8_t rx_flag;
} Serial_t;

static Serial_t serial_dev =
		{
				.m0 = NULL,
				.m1 = NULL,
				.rx_flag = 0,
				.uart = NULL
		};

/*
 * @brief Attach Serial commander to motor object
 * @param[in] BLDCMotor* m0
 * @param[in] BLDCMotor* m1
 * @param[in] UART handle
 * @retval -
 * @note Pass NULL for second motor ptr if only using 1 motor
 */
__INLINE void SerialCommander_Init(UART_HandleTypeDef* huart)
{
	serial_dev.uart = huart;
	serial_dev.m0 = BLDCMotorArray[0];
	serial_dev.m1 = BLDCMotorArray[1];
	HAL_UARTEx_ReceiveToIdle_DMA(serial_dev.uart, serial_dev.rx_buff, 32);
}

/*
 * @brief DMA receive to idle interrupt callback function
 * @note Sets the receive flag to 1 and append null terminator to end of string
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	/* Indicate incoming data */
	serial_dev.rx_flag = 1;

	/* Insert null terminator for C string */
	if(Size > 15)
	{
		serial_dev.rx_buff[15] = '\0';
	}
	else
	{
		serial_dev.rx_buff[Size] = '\0';
	}
}

/*
 * @brief Transmits a string over UART using DMA
 * @param[in] UART handle
 * @param[in] string
 * @retval -
 */
__INLINE void Serial_Print(const char* s)
{
	HAL_UART_Transmit(serial_dev.uart, s, strlen(s), HAL_MAX_DELAY);
}

/* @scope Static
 * @brief Return the Ctrl_t enum type corresponding to command string
 * @param[in] uint8_t motor_idx used for checking sensor availability
 * @param[in] const char* command string
 * @retval Ctrl_t ctrl
 * @note Returns Ctrl_t error if command not recognised
 */
static Ctrl_t SerialCommander_ParseControlType(uint8_t motor_idx, const char* type)
{
	BLDCMotor* motor = BLDCMotorArray[motor_idx];

	/* Motor not in use */
	if(!motor){
		return error;
	}

	/* All the closed-loop control types need to check if sensor is attached */
	if(!strcmp(type, "OL-vel")){
		return open_loop_velocity;
	}
	else if(!strcmp(type, "CL-pos") && motor->sensor != NULL){
		return closed_loop_position_no_cs;
	}
	else if(!strcmp(type, "CL-vel") && motor->sensor != NULL){
		return closed_loop_velocity_no_cs;
	}
	else if(!strcmp(type, "CL-pos-cs") && motor->sensor != NULL){
		return closed_loop_position;
	}
	else if(!strcmp(type, "CL-vel-cs") && motor->sensor != NULL){
		return closed_loop_velocity;
	}
	else{
		return error;
	}
}

/* @scope Static
 * @brief Parse command, directly modifying the command variables
 * @param[in] const char* command
 * @param[in] uint8_t motor_idx
 * @param[in] char* param
 * @param[in] uint32_t param_size
 * @param[in] float* val
 * @param[in] Ctrl_t* ctrl
 * @retval uint8_t state; 0 if success, 1 if error
 */
static uint8_t SerialCommander_ParseCommand(const char* cmd, uint8_t* motor_idx, char* param, uint32_t param_size, float* val, Ctrl_t* ctrl)
{
	char buff[32];

	/* Copy cmd to buffer & append null terminator */
	strncpy(buff, cmd, sizeof(buff));
	buff[sizeof(buff) - 1] = '\0';

	/* Generate token: motor ID */
	char* tok = strtok(buff, " ");
	if(!tok || tok[0] != 'm')
	{
		Serial_Print("Invalid motor ID\n");
		return 1;
	}

	*motor_idx = tok[1] - '0';
	if(*motor_idx < 0 || *motor_idx >= 2)
	{
		Serial_Print("Motor ID out of range\n");
		return 1;
	}

	/* Generate token: param */
	tok = strtok(NULL, " ");
	if(!tok)
	{
		Serial_Print("Invalid command: no param\n");
		return 1;
	}
	strncpy(param, tok, param_size);
	param[param_size - 1] = '\0';

	/* Generate token: value */
	tok = strtok(NULL, " ");
	if(!tok)
	{
		Serial_Print("Invalid command: no value\n");
		return 1;
	}

	/* Handle control parameter */
	if(!strcmp(param, "ctrl")){
		*ctrl = SerialCommander_ParseControlType(*motor_idx, (const char*)tok);
	}
	else{
		*ctrl = error;
		*val = atof(tok);
	}

	return 0;
}

/* @scope Static
 * @brief Set motor params from parsed command
 * @param[in] uint8_t motor_idx
 * @param[in] const char* param
 * @param[in] float val
 * @param[in] Ctrl_t ctrl
 * @retval uint8_t state; 0 if success, 1 if error
 */
static uint8_t SerialCommander_SetMotorParam(uint8_t motor_idx, const char* param, float val, Ctrl_t ctrl)
{
	/* Get motor reference & check if motor is in use */
	BLDCMotor* motor = BLDCMotorArray[motor_idx];
	if(!motor)
	{
		return 1;
	}

	/* Set param value */
	if(!strcmp(param, "vel")){
		motor->target_velocity = val;
	}
	else if(!strcmp(param, "pos")){
		motor->target_pos = val;
	}
	else if(!strcmp(param, "v_limit")){
		motor->voltage_limit = val;
	}
	else if(!strcmp(param, "pid_p")){
		motor->pid.kp = val;
	}
	else if(!strcmp(param, "pid_i")){
		motor->pid.ki = val;
	}
	else if(!strcmp(param, "pid_d")){
		motor->pid.kd = val;
	}
	else if(!strcmp(param, "current")){
		motor->target_current = val;
	}
	else if(!strcmp(param, "ctrl")){
		/* No update to control type if error parsing */
		if(ctrl != error){
			motor->control = ctrl;
		}
		else{
			return 1;
		}
	}
	else{
		return 1;
	}
	return 0;
}

/*
 * @brief Event updater; to be called repeatedly in main loop
 * @param[in] -
 * @retval -
 */
void SerialCommander_PollCommands()
{
	/* If data flag raised, parse buffer data */
	if(serial_dev.rx_flag == 1)
	{
		uint8_t motor_idx;
		char param[16];
		float val;
		Ctrl_t ctrl;

		uint8_t state = 1;

		/* Parse commands */
		state = SerialCommander_ParseCommand((const char*)serial_dev.rx_buff, &motor_idx, param, sizeof(param), &val, &ctrl);

		if(state){
			return;
		}

		/* Set new params */
		state = SerialCommander_SetMotorParam(motor_idx, (const char*)param, val, ctrl);

		/* Initiate next uart receive to idle event */
		HAL_UARTEx_ReceiveToIdle_DMA(serial_dev.uart, serial_dev.rx_buff, 32);
		serial_dev.rx_flag = 0;
	}
}
