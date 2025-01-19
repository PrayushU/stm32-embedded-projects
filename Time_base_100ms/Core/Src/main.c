/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "main.h"
#include<string.h>
#include <stdio.h>


void Error_handler(void);
void SystemClock_Config();
void TIMER6_Init();
void GPIO_Init(void);


TIM_HandleTypeDef htimer6;



int main(void)
{
	HAL_Init();
	SystemClock_Config();
	GPIO_Init();
	TIMER6_Init(htimer6);
	__HAL_RCC_BKPSRAM_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();

	//Lets start the timer
	HAL_TIM_Base_Start(&htimer6);


	while(1){
		//have to check timer update in status register
		while(!(TIM6->SR & TIM_SR_UIF));
		/*
		 * Thre required time delay has been elapsed
		 * User code can be executed
		 */
		TIM6->SR = 0;
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);

	}


	while(1);


	return 0;
}




void SystemClock_Config(){



}


void GPIO_Init(void){

	__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &ledgpio);
}



void TIMER6_Init(){

	htimer6.Instance = TIM6;
	htimer6.Init.Prescaler=24;
	htimer6.Init.Period= 1600-1;

	if(HAL_TIM_Base_Init(&htimer6) != HAL_OK){
		Error_handler();
	}



}



void Error_handler(void)
{
	while(1);
}
