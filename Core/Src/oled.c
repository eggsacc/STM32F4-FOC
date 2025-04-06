/*
 * oled.c
 *
 *  Created on: Apr 6, 2025
 *      Author: yizha
 */

#include "oled.h"

static oled_println(const char* str, uint8_t* line)
{
	/* Check if writing outside screen region; if so, do nothing */
	if(*line + OLED_LINE_SPACING > SSD1306_HEIGHT)
	{
		return;
	}

	ssd1306_SetCursor(0, *line);
	ssd1306_WriteString(str, Font_7x10, White);
	*line += OLED_LINE_SPACING;
}

void OLED_UpdateDMA(BLDCMotor* arr)
{
	/* Clear screen */
	ssd1306_Fill(Black);

	uint8_t line = 0;
	char line_buff[18];

	if(arr[0] != NULL && arr[1] == NULL)
	{

	}
	oled_println("       M0  |  M1  ", &line);
	snprintf(line_buff, 18, "Angle: %5.2f %5.2f", )





	if(arr[0] != NULL)
	{

	}
}
