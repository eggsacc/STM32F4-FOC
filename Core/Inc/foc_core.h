/*
 * foc_driver.h
 *
 *  Created on: Feb 19, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_CORE_H_
#define INC_FOC_CORE_H_

#include "foc_hw.h"

/*
 * Control functions
 */
void OLVelocityControl(BLDCMotor* motor, float target_velocity);
void CLPositionControl(BLDCMotor* motor, float target_pos);
void CLVelocityControl(BLDCMotor* motor, float target_velocity);
#endif /* INC_FOC_CORE_H_ */
