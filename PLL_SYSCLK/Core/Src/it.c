/*
 * it.c
 *
 *  Created on: Apr 15, 2023
 *      Author: prayushu
 */

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
