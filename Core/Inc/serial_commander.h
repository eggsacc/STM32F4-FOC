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

extern BLDCMotor* BLDCMotorArray[2];

/*
 * Function prototypes
 */
void Serial_Print(const char* s);
void SerialCommander_PollCommands();
void SerialCommander_Init(UART_HandleTypeDef* huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);


#endif /* INC_SERIAL_COMMANDER_H_ */
