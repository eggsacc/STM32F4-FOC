/*
 * fixed_int.h
 *
 *  Created on: Feb 27, 2025
 *      Author: yizha
 */

#ifndef INC_FIX16_H_
#define INC_FIX16_H_

#include "stm32f4xx_hal.h"

/*
 * Fixed 16 typedef
 */
typedef int32_t fix16_t;

/*
 * Some fixed 16 constants
 */
#define FIX16_PI     ((fix16_t)0x0003243F)
#define FIX16_PI_2   ((fix16_t)0x00019220)
#define FIX16_PI_3   ((fix16_t)0x00010C15)
#define FIX16_2PI    ((fix16_t)0x0006487F)
#define FIX16_3PI_2  ((fix16_t)0x0004B65F)

#define FIX16_SQRT3    ((fix16_t)0x0001BB68)
#define FIX16_1_SQRT3  ((fix16_t)0x000093CD)
#define FIX16_SQRT3_2  ((fix16_t)0x0000DDB4)

/*
 * Macros
 */
#define fix16(x) ((fix16_t)(((x) >= 0) ? ((x) * 65536.0f + 0.5f) : ((x) * 65536.0f - 0.5f)))

/*
 * Functions
 */
fix16_t int_to_fix16(int x);

fix16_t float_to_fix16(float x);

float fix16_to_float(fix16_t x);

int32_t fix16_to_int(fix16_t x);

uint32_t fix16_abs(fix16_t x);

int32_t fix16_get_int_bits(fix16_t x);

uint32_t fix16_get_decimal_bits(fix16_t x);

fix16_t fix16_mul(fix16_t a, fix16_t b);
fix16_t fix16_div(fix16_t a, fix16_t b);

#endif /* INC_FIX16_H_ */
