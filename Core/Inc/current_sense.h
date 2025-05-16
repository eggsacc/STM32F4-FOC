/*
 * current_sense.h
 *
 *  Created on: Feb 23, 2025
 *      Author: yizha
 */

#ifndef INC_CURRENT_SENSE_H_
#define INC_CURRENT_SENSE_H_

#include "stm32f4xx_hal.h"
#include "foc_hw.h"

#define CS_OFFSET_CALIBRATION_TRIALS 500
#define SHUNT_RESISTANCE 0.01f
#define AMP_GAIN 50
#define ADC_VREF 3.3f
#define ADC_RESOLUTION 4096

/*
 * ADC buffer array
 */
extern uint16_t ADC_buff[4];
extern ADC_HandleTypeDef hadc1;

/*
 * Fucntion prototypes
 */
void BLDC_UpdateMotorADC_DMA();
void CS_Init(BLDCMotor* motor);
void CS_SamplePhaseCurrents(BLDCMotor* motor);
float CS_SetTorque(BLDCMotor* motor);

#endif /* INC_CURRENT_SENSE_H_ */
