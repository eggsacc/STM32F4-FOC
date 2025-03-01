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

#define FIX16_ONE  ((fix16_t)0x00010000)
#define FIX16_HALF ((fix16_t)0x8000)
#define FIX16_MAX  ((fix16_t)0x7FFFFFFF)
#define FIX16_MIN  ((fix16_t)0x80000000)
#define FIX16_OF   ((fix16_t)0x80000000)

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
 * Fix16 operations
 * Most of these functions follow generic arithmatic operations
 */
__STATIC_INLINE fix16_t int_to_fix16(int x){ return (fix16_t)(x << 16); }

__STATIC_INLINE fix16_t float_to_fix16(float x){ return (fix16_t)(x * FIX16_ONE + (x >= 0 ? 0.5f : -0.5f)); }

__STATIC_INLINE float fix16_to_float(fix16_t x){ return x / (float)FIX16_ONE; }

__STATIC_INLINE int32_t fix16_to_int(fix16_t x){ return (x >= 0) ? ((x + FIX16_HALF) >> 16) : ((x - FIX16_HALF) >> 16); }

__STATIC_INLINE uint32_t fix16_abs(fix16_t x){ return (x >= 0) ? x : -x; }

__STATIC_INLINE fix16_t fix16_mod(fix16_t x, fix16_t y) { return x % y; }

__STATIC_INLINE int32_t fix16_get_int_bits(fix16_t x){ return (int32_t)(x >> 16); }

__STATIC_INLINE uint32_t fix16_get_decimal_bits(fix16_t x){ return (uint32_t)(x & 0xFFFF); }

/*
 * Specialized operations: requires special considerations  & some extra work
 * @warning fix16_mul really likes to overflow. Limit x * y to 16129 (where x & y are normal integers)
 * @note fix16_div leverages hardware division available on most stm32 processors
 */
fix16_t fix16_mul(fix16_t a, fix16_t b);
fix16_t fix16_div(fix16_t a, fix16_t b);

#endif /* INC_FIX16_H_ */
