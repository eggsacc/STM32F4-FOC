/*
 * oled.c
 *
 *  Created on: Apr 6, 2025
 *      Author: yizha
 */

#include "oled.h"

#ifdef OLED

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	EVENT_FLAGS |= I2C2_DMA_FLAG;
}

void OLED_Init()
{
	ssd1306_Init();
}

static void oled_println(const char* str, uint8_t* line, SSD1306_Font_t font)
{
	/* Check if writing outside screen region; if so, do nothing */
	if(*line + OLED_LINE_SPACING > SSD1306_HEIGHT)
	{
		return;
	}

	ssd1306_SetCursor(0, *line);
	ssd1306_WriteString(str, font, White);
	*line += OLED_LINE_SPACING;
}

void OLED_Callback(BLDCMotor* motors[2])
{
	/* Clear screen */
	ssd1306_Fill(Black);

	uint8_t line = 0;
	char line_buff[19];
	char m0_angle_buff[6];
	char m1_angle_buff[6];

	if(motors[0] != NULL && motors[0]->sensor != NULL)
	{
		if(motors[0]->sensor->total_angle_rad >= 100)
		{
			snprintf(m0_angle_buff, 6, "%5.1f", motors[0]->sensor->total_angle_rad);
		}
		else {
			snprintf(m0_angle_buff, 6, "%5.2f", motors[0]->sensor->total_angle_rad);
		}
	}
	else {
		snprintf(m0_angle_buff, 6, "  -  ");
	}

	if(motors[1] != NULL && motors[1]->sensor != NULL)
	{
		if(motors[1]->sensor->total_angle_rad >= 100)
		{
			snprintf(m1_angle_buff, 6, "%5.1f", motors[1]->sensor->total_angle_rad);
		}
		else {
			snprintf(m1_angle_buff, 6, "%5.2f", motors[1]->sensor->total_angle_rad);
		}
	}
	else {
		snprintf(m1_angle_buff, 6, "  -  ");
	}

	oled_println("       M0  |  M1  ", &line, Font_7x10);
	snprintf(line_buff, sizeof(line_buff), "Angle:%s %s", m0_angle_buff, m1_angle_buff);
	oled_println((const char*)line_buff, &line, Font_7x10);
	ssd1306_UpdateScreen();
}

#endif
