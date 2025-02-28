/*
 * foc_utils.c
 *
 *  Created on: Jan 12, 2025
 *      Author: yizha
 */

/*
 * Includes
 */
#include "foc_utils.h"
#include "stm32f4xx_hal.h"
#include <math.h>

/*
 * Sine look-up table definition
 * x-range: [0, 2pi] -> 256 entries,
 * y-range: [-1, 1] -> [-32768, 32767]
 */
uint16_t sineLUT[65] = {0, 804, 1607, 2410, 3211, 4011, 4807, 5601, 6392, 7179, 7961, 8739, 9511, 10278, 11039, 11792,
		12539, 13278, 14009, 14732, 15446, 16151, 16845, 17530, 18204, 18867, 19519, 20159, 20787, 21402, 22005, 22594,
		23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
		30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
		32767};

//uint16_t sineLUT[129] = {0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611, 4011, 4409, 4807, 5205, 5601, 5997,
//		6392, 6786, 7179, 7571, 7961, 8351, 8739, 9126, 9511, 9895, 10278, 10659, 11039, 11416, 11792, 12167,
//		12539, 12910, 13278, 13645, 14009, 14372, 14732, 15090, 15446, 15799, 16151, 16499, 16845, 17189, 17530, 17868,
//		18204, 18537, 18867, 19195, 19519, 19841, 20159, 20475, 20787, 21096, 21402, 21705, 22005, 22301, 22594, 22884,
//		23170, 23452, 23731, 24007, 24279, 24547, 24811, 25072, 25329, 25582, 25832, 26077, 26319, 26556, 26790, 27019,
//		27245, 27466, 27683, 27896, 28105, 28310, 28510, 28706, 28898, 29085, 29268, 29447, 29621, 29791, 29956, 30117,
//		30273, 30424, 30571, 30714, 30852, 30985, 31113, 31237, 31356, 31470, 31580, 31685, 31785, 31880, 31971, 32057,
//		32137, 32213, 32285, 32351, 32412, 32469, 32521, 32567, 32609, 32646, 32678, 32705, 32728, 32745, 32757, 32765,
//		32767};
/*
 * @brief Sine approximation using look-up table & linear interpolation
 * @param[in] angle(radians)
 * @return sin(angle)
 */
fix16_t _sin(fix16_t angle){

  int32_t first, second;
  fix16_t index = fix16_div(angle, FIX16_2PI) << SINELUT_WIDTH_BITS;
  uint8_t frac = (uint8_t)((index & 0xFFFF) >> 8);
  index >>= 16;

  if (index < SINELUT_QUAD_1){
    first = (int32_t)sineLUT[index];
    second = (int32_t)sineLUT[index + 1];
  }
  else if (index < SINELUT_QUAD_2){
    first = (int32_t)sineLUT[SINELUT_WIDTH_HALF - index];
    second = (int32_t)sineLUT[SINELUT_WIDTH_HALF - index - 1];
  }
  else if (index < SINELUT_QUAD_3){
    first = -(int32_t)sineLUT[index - SINELUT_WIDTH_HALF];
    second = -(int32_t)sineLUT[index - SINELUT_WIDTH_HALF + 1];
  }
  else {
    first = -(int32_t)sineLUT[SINELUT_WIDTH - index];
    second = -(int32_t)sineLUT[SINELUT_WIDTH - index - 1];
  }

  return (fix16_t)((first + (((second - first) * frac) >> 8)) * 2);
}
