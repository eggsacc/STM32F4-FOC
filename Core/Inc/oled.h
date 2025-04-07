/*
 * OLED.h
 *
 *  Created on: Apr 6, 2025
 *      Author: yizha
 */

#ifndef INC_OLED_H_
#define INC_OLED_H_

#include <stdio.h>
#include <string.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "foc_hw.h"

#define OLED_LINE_SPACING 15

void OLED_Init();
void OLED_UpdateDMA(BLDCMotor* motors[2]);


#endif /* INC_OLED_H_ */
