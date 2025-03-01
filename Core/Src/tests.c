/*
 * tests.c
 *
 *  Created on: Mar 1, 2025
 *      Author: yizha
 */

#include "tests.h"
#include <string.h>
#include <stdio.h>

static uint8_t tx_buff[128];
extern UART_HandleTypeDef huart1;

void Run_trigonometric_tests()
{
	sprintf(tx_buff, "######## Trig functions test ########\n\r");
	HAL_UART_Transmit_DMA(&huart1, &tx_buff, strlen(tx_buff));
	HAL_Delay(10);

	for(fix16_t i = 0; i < FIX16_2PI; i += float_to_fix16(0.196))
	{
		sprintf(tx_buff, "built-in sin: %f, LUT _sin: %f\n", sin(fix16_to_float(i)), fix16_to_float(_sin(i)));
		HAL_UART_Transmit_DMA(&huart1, &tx_buff, strlen(tx_buff));
		HAL_Delay(10);
	}

	for(fix16_t i = 0; i < FIX16_2PI; i += float_to_fix16(0.196))
	{
		sprintf(tx_buff, "built-in cos: %f, LUT _cos: %f\n", cos(fix16_to_float(i)), fix16_to_float(_cos(i)));
		HAL_UART_Transmit_DMA(&huart1, &tx_buff, strlen(tx_buff));
		HAL_Delay(10);
	}

	sprintf(tx_buff, "######## Trig tests ended ########\n\r");
	HAL_UART_Transmit_DMA(&huart1, &tx_buff, strlen(tx_buff));
	HAL_Delay(10);
}
