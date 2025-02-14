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
#include "stm32f1xx_hal.h"
#include <math.h>

/*
 * Sine look-up table definition
 */
uint16_t sineLUT[] = {0, 804, 1607, 2410, 3211, 4011, 4808, 5602, 6392, 7179, 7961, 8739, 9512, 10278, 11039, 11793,
		12539, 13278, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594, 23170,
		23732, 24279, 24812, 25330, 25832, 26319, 26790, 27245, 27684, 28106, 28511, 28898, 29269, 29621, 29956, 30273, 30572,
		30852, 31114, 31357, 31581, 31785, 31971, 32138, 32285, 32413, 32521, 32610, 32679, 32728, 32758, 32768};

/*
 * @brief Sine approximation using look-up table
 * @param[in] angle(radians)
 * @return sin(angle)
 */
float _sin(float angle){

  int32_t first, second;
  uint16_t index = (uint16_t)(angle / _2PI * 65536.0f);
  int frac = index & 0xff;
  index = (index >> 8) & 0xff;

  if (index < 64){
    first = (int32_t)sineLUT[index];
    second = (int32_t)sineLUT[index + 1];
  }
  else if (index < 128){
    first = (int32_t)sineLUT[128 - index];
    second = (int32_t)sineLUT[127 - index];
  }
  else if (index < 192){
    first = -(int32_t)sineLUT[index - 128];
    second = -(int32_t)sineLUT[index - 127];
  }
  else {
    first = -(int32_t)sineLUT[256 - index];
    second = -(int32_t)sineLUT[255 - index];
  }

  return (first + (((second - first) * frac) >> 8)) / 32768.0f;
}
