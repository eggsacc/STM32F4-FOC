/*
 * AS5600.c
 *
 *  Created on: Oct 19, 2024
 *      Author: yizha
 */

#include "AS5600.h"

/*
 * Static low level functions to read/write to registers
 */
static HAL_StatusTypeDef AS5600_ReadRegister(AS5600 *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef AS5600_ReadRegisters(AS5600 *dev, uint8_t reg, uint8_t *data, uint8_t length)
{
	return HAL_I2C_Mem_Read(dev->i2cHandle, AS5600_I2C_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
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
	dev->prev_raw_angle = AS5600_ReadRawAngle(dev);
}

/*
 * Initialization
 * Pass the struct to each function as pointer
 */
uint8_t AS5600_Init(AS5600* dev, I2C_HandleTypeDef* i2c_handle, uint8_t zero)
{
	/* Set struct params */
	dev->i2cHandle = i2c_handle;
	dev->total_angle_rad = 0;

	uint8_t err_num = 0;

	/*
	 * Check magnet strength
	 */
	HAL_StatusTypeDef magnet_status;
	uint8_t regdata = 0;

	magnet_status = AS5600_ReadRegister(dev, MAGNET_STATUS_REG, &regdata);
	err_num += (magnet_status != HAL_OK);

	/* bit[5] indicates magnet present if set */
	if(!(regdata & (1 << 5))){

		return 255;
	}

	/* initialize starting angle */
	uint8_t regdata_angle[2] = {0, 0};
	HAL_StatusTypeDef status = AS5600_ReadRegisters(dev, RAW_ANGLE_MSB_REG, regdata_angle, 2);

	/* Mask & shift 4 MSB left by 8 and concatenate with 8 LSBs */
	uint16_t raw_angle = (((regdata_angle[0] & 0x0F) << 8) | regdata_angle[1]);

	err_num += (status != HAL_OK);

	dev->prev_raw_angle = raw_angle;

	if(!zero)
	{
		dev->total_angle_rad = raw_angle * BIT_TO_RAD;
	}

	return err_num;
}

/*
 * Read sensor value
 */
float AS5600_ReadAngle(AS5600 *dev)
{
	int16_t raw_angle = AS5600_ReadRawAngle(dev);
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

	return dev->total_angle_rad;
}

uint16_t AS5600_ReadRawAngle(AS5600 *dev)
{
	uint8_t regdata[2] = {0, 0};
	HAL_StatusTypeDef status = AS5600_ReadRegisters(dev, RAW_ANGLE_MSB_REG, regdata, 2);

	/* Return early if register read fails */
	if(status != HAL_OK)
	{
		return 0;;
	}

	/* Mask & shift 4 MSB left by 8 and concatenate with 8 LSBs */
	uint16_t raw_angle = (((regdata[0] & 0x0F) << 8) | regdata[1]);
	return raw_angle;
}





