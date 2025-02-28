/*
 * serialCommander.c
 *
 *  Created on: Feb 1, 2025
 *      Author: yizha
 */

#include "main.h"
#include "serial_commander.h"
#include <stdio.h>
#include <string.h>

/*
 * Static variables used
 */
static UART_HandleTypeDef* uart_dev;
static BLDCMotor* bldc_dev;
static uint8_t rx_buff[16];
static uint8_t tx_buff[32];
static uint8_t rx_flag = 0;

/*
 * @brief Attach Serial commander to motor object
 * @param[in] motor ptr
 * @param[in] UART handle
 * @retval -
 */
__INLINE void SerialCommander_Init(BLDCMotor* motor, UART_HandleTypeDef* huart)
{
	uart_dev = huart;
	bldc_dev = motor;
	HAL_UARTEx_ReceiveToIdle_DMA(uart_dev, rx_buff, 32);
}

/*
 * @brief DMA receive to idle interrupt callback function
 * @note Sets the receive flag to 1
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	/* Indicate incoming data */
	rx_flag = 1;

	/* Insert null terminator for C string */
	if(Size > 15)
	{
		rx_buff[15] = '\0';
	}
	else
	{
		rx_buff[Size] = '\0';
	}
}

/*
 * @brief Transmits a string over UART using DMA
 * @param[in] UART handle
 * @param[in] string
 * @retval -
 */
void Serial_Print(const char* s)
{
	HAL_UART_Transmit_DMA(uart_dev, s, strlen(s));
}

/*
 * @brief Event updater; to be called repeatedly in main loop
 * @param[in] -
 * @retval -
 */
void SerialCommander_EventUpdate()
{
	/* If data flag raised, parse buffer data */
	if(rx_flag)
	{
		char cmd;
		float val;

		/* If third character is not terminator */
		if(rx_buff[2] != '\0')
		{
			/* Parse data in format (command val_float) */
			sscanf(rx_buff, "%c %f", &cmd, &val);
		}
		else
		{
			Serial_Print("Invalid command\n");
		}

		switch (cmd)
		{
		case 'V':
			sprintf(&tx_buff, "Velocity: %f", val);
			Serial_Print((const char*)&tx_buff);
			break;

		case 'T':
			Serial_Print("T\n");
			break;

		default:
			Serial_Print("Invalid command\n");
		}

		/* Reset receive data flag & signal dma ready for next input */
		rx_flag = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(uart_dev, rx_buff, 32);
	}
}
