/*
 * timer_update.c
 *
 *  Created on: Mar 23, 2025
 *      Author: yizha
 */

#include "timer_update.h"

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance == TIM4)
  {
	  if(ADC_DMA_READY == 1)
	  {
		  HAL_ADC_Start_DMA(&hadc1, adc_vals, 4);
	  }

  }
  /* USER CODE END Callback 1 */
}
