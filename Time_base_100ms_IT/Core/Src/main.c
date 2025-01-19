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
	TIMER6_Init();





	//Lets start timer in IT mode
	HAL_TIM_Base_Start_IT(&htimer6);


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
	htimer6.Init.Period= 64000-1;

	if(HAL_TIM_Base_Init(&htimer6) != HAL_OK){
		Error_handler();
	}



}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);

}

void Error_handler(void)
{
	while(1);
}
