/*
 * SerialCommander.h
 *
 *  Created on: Feb 1, 2025
 *      Author: yizha
 */

#ifndef INC_SERIAL_COMMANDER_H_
#define INC_SERIAL_COMMANDER_H_

#include "main.h"
#include "foc_core.h"

__INLINE void Serial_Print(const char* s);
void SerialCommander_EventUpdate();
void SerialCommander_Init(BLDCMotor* motor, UART_HandleTypeDef* huart);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


#endif /* INC_SERIAL_COMMANDER_H_ */
