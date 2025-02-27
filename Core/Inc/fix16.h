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
 * Fixed 16 multiplier constants
 */
static const fix16_t fix16_one = 0x00010000;
static const fix16_t fix16_half = 0x8000;
static const fix16_t fix16_maximum  = 0x7FFFFFFF;
static const fix16_t fix16_minimum  = 0x80000000;
static const fix16_t fix16_overflow = 0x80000000;

/*
 * Macros
 */
#define fix16(x) ((fix16_t)(((x) >= 0) ? ((x) * 65536.0f + 0.5f) : ((x) * 65536.0f - 0.5f)))

/*
 * Functions
 */
__STATIC_INLINE fix16_t int_to_fix16(int x) { return x * fix16_one; }

__STATIC_INLINE fix16_t float_to_fix16(float x)
{
	return (fix16_t)(x * fix16_one + (x >= 0 ? 0.5f : -0.5f));
}

__STATIC_INLINE float fix16_to_float(fix16_t x) { return x / (float)fix16_one; }

__STATIC_INLINE int16_t fix16_to_int(fix16_t x)
{
	if(x >= 0)
	{
		return ((x + fix16_half) >> 16);
	}
	return ((x - fix16_half) >> 16);
}

__STATIC_INLINE uint32_t fix16_abs(fix16_t x) { return (x >= 0) ? x : -x; }

__STATIC_INLINE fix16_t fix16_add(fix16_t a, fix16_t b) { return a + b; }

__STATIC_INLINE fix16_t fix16_sub(fix16_t a, fix16_t b) { return a - b; }

/*
 * @brief 64-bit implementation of multiplication. Fastest on 32-bit MCUs like Cortex M3
 *
 * @note Upper 16 bits -> overflow detection
 * 		 Middle 32 bits -> result
 * 		 Lower 16 bits -> rounding / discarded
 *
 * @warning Fixed-point multiplications really likes to overflow & cause errors.
 * 			Based on calculations, the maximum value to multiply together is only +- 127.
 */
fix16_t fix16_mul(fix16_t a, fix16_t b)
{
	int64_t product = (int64_t)a * (int64_t)b;

	uint32_t upper = (product >> 47);

	/* Check for overflow. The most significant 17 bits should all be 0 or 1 */
	if(product < 0)
	{
		/* For negative numbers, all upper is 1. So invert to check if all 0. */
		if(~upper)
		{
			return fix16_overflow;
		}
		product --;
	}
	else
	{
		/* Positive numbers: all 0 */
		if(upper)
		{
			return fix16_overflow;
		}
	}

	/* Rounding : take bit 15 (the first bit discarded) & add it to result */
	fix16_t result = product >> 16;
	result += (product & 0x8000) >> 15;

	return result;
}

/*
 * @brief Counts leading 0s in an integer
 */
static uint8_t clz(uint32_t x)
{
	uint8_t result = 0;
	if (x == 0) return 32;
	while (!(x & 0xF0000000)) { result += 4; x <<= 4; }
	while (!(x & 0x80000000)) { result += 1; x <<= 1; }
	return result;
}

/*
 * @brief Fixed-point division leveraging hardware division found on compatible MCUs.
 */
fix16_t fix16_div(fix16_t a, fix16_t b)
{
	// This uses a hardware 32/32 bit division multiple times, until we have
	// computed all the bits in (a<<17)/b. Usually this takes 1-3 iterations.

	/* Prevent division by 0: returns min value instead */
	if (b == 0)
	{
		return fix16_minimum;
	}

    uint32_t remainder = fix16_abs(a);
    uint32_t divider = fix16_abs(b);
    uint64_t quotient = 0;
    int bit_pos = 17;

	/* Optimize initial calculations for the case where divisor is large */
	if (divider & 0xFFF00000)
	{
		uint32_t shifted_div = ((divider >> 17) + 1);
        quotient = remainder / shifted_div;
        uint64_t tmp = ((uint64_t)quotient * (uint64_t)divider) >> 17;
        remainder -= (uint32_t)(tmp);
    }

	/* If divider is divisible by 16, take advantage of bit shifting */
	while (!(divider & 0xF) && bit_pos >= 4)
	{
		divider >>= 4;
		bit_pos -= 4;
	}

	/* Main division loop */
	while (remainder && bit_pos >= 0)
	{
		/* Use clz() to check max left shifts without overflowing */
		int shift = clz(remainder);
		if (shift > bit_pos) shift = bit_pos;
		remainder <<= shift;
		bit_pos -= shift;

		uint32_t div = remainder / divider;
        remainder = remainder % divider;
        quotient += (uint64_t)div << bit_pos;

		if (div & ~(0xFFFFFFFF >> bit_pos))
				return fix16_overflow;

		remainder <<= 1;
		bit_pos--;
	}

	/* Rounding: quotient is always positive */
	quotient++;

	fix16_t result = (quotient + (quotient >= 0 ? 1 : -1)) >> 1;

	/* Sign of result */
	if ((a ^ b) & 0x80000000)
	{
		if (result == fix16_minimum)
				return fix16_overflow;

		result = -result;
	}

	return result;
}

#endif /* INC_FIX16_H_ */
