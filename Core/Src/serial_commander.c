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

typedef struct
{
	BLDCMotor* m0;
	BLDCMotor* m1;
	UART_HandleTypeDef* uart;
	uint8_t rx_buff[32];
	uint8_t tx_buff[32];
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
__INLINE void SerialCommander_Init(BLDCMotor* m0, BLDCMotor* m1, UART_HandleTypeDef* huart)
{
	serial_dev.uart = huart;
	serial_dev.m0 = m0;
	serial_dev.m1 = m1;
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
	HAL_UART_Transmit_DMA(serial_dev.uart, s, strlen(s));
}

/*
 * @brief Event updater; to be called repeatedly in main loop
 * @param[in] -
 * @retval -
 */
void SerialCommander_PollEvents()
{
	/* If data flag raised, parse buffer data */
	if(serial_dev.rx_flag == 1)
	{
		Serial_Print((char*)serial_dev.rx_buff);
		char cmd;
		float val;
		char motor[2];

		/* If third character is not terminator */
//		if(serial_dev.rx_buff[2] != '\0')
//		{
//			/* Parse data in format (command val_float) */
//			sscanf(serial_dev.rx_buff, "%c %f", &cmd, &val);
//		}
//		else
//		{
//			Serial_Print("Invalid command\n");
//		}
//
//		switch (cmd)
//		{
//		case 'V':
//			sprintf((char*)serial_dev.tx_buff, "%s Velocity: %f", val);
//			Serial_Print((const char*)serial_dev.tx_buff);
//			serial_dev.m0->target_velocity = val;
//			break;
//
//		case 'T':
//			sprintf((char*)serial_dev.tx_buff, "Velocity: %f", val);
//			Serial_Print((const char*)serial_dev.tx_buff);
//			Serial_Print("T\n");
//			break;
//
//		default:
//			Serial_Print("Invalid command\n");
//		}

		/* Reset receive data flag & signal dma ready for next input */
		serial_dev.rx_flag = 0;
		HAL_UARTEx_ReceiveToIdle_DMA(serial_dev.uart, serial_dev.rx_buff, 32);
	}
}
