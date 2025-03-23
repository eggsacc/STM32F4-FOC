/*
 * timer_update.h
 *
 *  Created on: Mar 23, 2025
 *      Author: yizha
 */

#ifndef INC_TIMER_UPDATE_H_
#define INC_TIMER_UPDATE_H_

#include "main.h"
#include "stm32f4xx_it.h"

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_TIMER_UPDATE_H_ */
