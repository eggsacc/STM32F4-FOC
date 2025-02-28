	/*
 * foc_utils.h
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

#ifndef INC_FOC_UTILS_H_
#define INC_FOC_UTILS_H_

/*
 * Includes
 */
#include "stm32f4xx_hal.h"
#include "fix16.h"
#include <math.h>

/*
 * Constants definition
 */
#define SINELUT_WIDTH_BITS 8
#define SINELUT_WIDTH 256
#define SINELUT_WIDTH_HALF 128
#define SINELUT_QUAD_1 64
#define SINELUT_QUAD_2 128
#define SINELUT_QUAD_3 192

#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f
#define RAD_TO_DEG 57.2957795131
#define DEG_TO_RAD 0.01745329251

/*
 * Some macros
 */
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define _abs(val)((val < 0) ? (-val) : (val))
/*
 * Trig approximation functions using look-up table
 */
fix16_t _sin(fix16_t angle);

/*
 * @brief Cosine approximation
 * @param[in] angle(radians)
 * @return cos(angle)
 */
__STATIC_INLINE fix16_t _cos(fix16_t angle) {
  fix16_t _angle = angle + FIX16_PI_2;
  _angle = _angle > FIX16_2PI ? _angle - FIX16_2PI : _angle;
  return _sin(_angle);
}

/*
 * @brief Normalize angle to [0, 2pi]
 * @param[in] angle(radians)
 * @return normalized_angle
 */
__STATIC_INLINE fix16_t _normalizeAngle(float angle){
  float a = fmod(angle, _2PI);       // fmod(x,y) returns remainder of x/y
  return a >= 0 ? float_to_fix16(a) : (float_to_fix16(a) + FIX16_2PI);    // add 2pi to negative angles to make positive
}

/*
 * @brief Calculates electrical angle from rotor angle
 * @param[in] shaft_angle(radians)
 * @param[in] pole_pairs
 * @return electrical angle
 */
__STATIC_INLINE fix16_t _electricalAngle(fix16_t shaft_angle, uint8_t pole_pairs){
  return (fix16_mul(shaft_angle, (fix16_t)pole_pairs));
}

/*
 * Sine loop-up table: 16-bit depth, 8-bit resolution
 */
extern uint16_t sineLUT[];

#endif /* INC_FOC_UTILS_H_ */
