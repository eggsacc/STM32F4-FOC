/*
 * AS5600.c
 *
 *  Created on: Oct 19, 2024
 *      Author: yizha
 */

#include "AS5600.h"
#include "timer_utils.h"

/*
 * Static low level functions to read/write to registers
 */
__STATIC_INLINE HAL_StatusTypeDef AS5600_ReadRegister(AS5600 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}

__STATIC_INLINE HAL_StatusTypeDef AS5600_ReadRegister_DMA(AS5600 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read_DMA(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1);
}

__STATIC_INLINE HAL_StatusTypeDef AS5600_ReadRegisters_DMA(AS5600 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read_DMA(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, length);
}

//static HAL_StatusTypeDef AS5600_WriteRegister(AS5600 *dev, uint8_t reg, uint8_t *data)
//{
//	return HAL_I2C_Mem_Write(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
//}

/*
 * Utility functions
 */

/*
 * @brief Zeroes the sensor value.
 * @param[in] AS5600* sensor
 */
void AS5600_ZeroAngle(AS5600* dev)
{
	dev->total_angle_rad = 0;
	dev->prev_raw_angle = AS5600_GetRawAngle(dev);
}

/*
 * Initialization
 * Pass the struct to each function as pointer
 */
uint8_t AS5600_Init(AS5600* dev, I2C_HandleTypeDef* i2c_handle)
{
	/* Set struct params */
	dev->i2cHandle = i2c_handle;
	dev->total_angle_rad = 0;
	dev->prev_time_us = micros();
	dev->regdata[0] = 0;
	dev->regdata[1] = 0;

	uint8_t err_num = 0;

	/*
	 * Check magnet strength
	 */
	uint8_t regdata = 0;

	HAL_StatusTypeDef magnet_status = AS5600_ReadRegister(dev, MAGNET_STATUS_REG, &regdata);
	err_num += (magnet_status != HAL_OK);

	/* bit[5] indicates magnet present if set */
	if(!(regdata & (1 << 5)))
	{
		return 255;
	}

	return err_num;
}

void AS5600_UpdateAngle_DMA(AS5600 *dev)
{
	if(dev == NULL)
	{
		return;
	}

	HAL_StatusTypeDef status = AS5600_ReadRegisters_DMA(dev, RAW_ANGLE_MSB_REG, dev->regdata, 2);

	/* Return early if register read fails */
	if(status != HAL_OK)
	{
		return;
	}

	/* Mask & shift 4 MSB left by 8 and concatenate with 8 LSBs */
	uint16_t raw_angle = (((dev->regdata[0] & 0x0F) << 8) | dev->regdata[1]);

	/* Calculate angle delta from previous angle */
	int16_t delta = raw_angle - dev->prev_raw_angle;

	/*
	 * Positive large delta -> negative overflow
	 * Negative large delta -> positive overflow
	 */
	if(delta > HALF_MAX_RESOLUTION)
	{
		dev->total_angle_rad -= (MAX_RESOLUTION - delta) * BIT_TO_RAD;
	}
	else if(delta < -HALF_MAX_RESOLUTION)
	{
		dev->total_angle_rad += (MAX_RESOLUTION + delta) * BIT_TO_RAD;
	}
	else
	{
		dev->total_angle_rad += delta * BIT_TO_RAD;
	}

	dev->prev_raw_angle = raw_angle;
}

float AS5600_GetAngle(AS5600* dev)
{
	return dev->total_angle_rad;
}

uint16_t AS5600_GetRawAngle(AS5600* dev)
{
	return dev->prev_raw_angle;
}

/* @brief Returns the rate of change of sensor angle (velocity)
 * @param[in] AS5600* sensor
 */
float AS5600_GetVelocity(AS5600* dev)
{
	uint32_t now_us = micros();

	/* Save previous angle */
	float prev_angle = dev->total_angle_rad;

	/* Calculate time delta */
	float time_delta_s = (now_us - dev->prev_time_us) * 0.000001f;
	time_delta_s = (time_delta_s > 0.1) ? 0.0001f : time_delta_s;

	/* Calculate angle delta */
	float angle_delta = AS5600_GetAngle(dev) - prev_angle;

	/* Update sensor timestamp */
	dev->prev_time_us = now_us;

	return angle_delta / time_delta_s;
}



