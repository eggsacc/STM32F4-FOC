/*
 * foc_driver.h
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_CORE_H_
#define INC_FOC_CORE_H_

#include "foc_hw.h"
#include "current_sense.h"

/*
 * Control functions
 */
void BLDC_RunMode(BLDCMotor* m0, BLDCMotor* m1);
void OLVelocityControl(BLDCMotor* motor, float target_velocity);
void CLPositionControl(BLDCMotor* motor, float target_pos);
void CLVelocityControl(BLDCMotor* motor, float target_velocity);
void Haptic_Virtual_Detents(BLDCMotor* motor, uint8_t divisions);

#endif /* INC_FOC_CORE_H_ */
