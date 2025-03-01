/*
 * fix16.c
 *
 *  Created on: Feb 28, 2025
 *      Author: yizha
 */

#include "fix16.h"

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
			return FIX16_OF;
		}
		product --;
	}
	else
	{
		/* Positive numbers: all 0 */
		if(upper)
		{
			return FIX16_OF;
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
		return FIX16_MIN;
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
				return FIX16_OF;

		remainder <<= 1;
		bit_pos--;
	}

	/* Rounding: quotient is always positive */
	quotient++;

	fix16_t result = (quotient + (quotient >= 0 ? 1 : -1)) >> 1;

	/* Sign of result */
	if ((a ^ b) & 0x80000000)
	{
		if (result == FIX16_MIN)
				return FIX16_OF;

		result = -result;
	}

	return result;
}
