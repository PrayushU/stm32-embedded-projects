/*
 * it.c
 *
 *  Created on: Apr 15, 2023
 *      Author: prayushu
 */

#include "main.h"
extern TIM_HandleTypeDef htimer6;

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htimer6);

}
