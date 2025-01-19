/*
 * main.c
 *
 *  Created on: Apr 18, 2023
 *      Author: prayushu
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include<string.h>
#include <stdio.h>

void Error_handler(void);
void GPIO_Init(void);
void UART2_Init(void);
void TIMER2_Init(void);
void SystemClock_Config_HSE(uint8_t clock_freq );



TIM_HandleTypeDef htimer2;
UART_HandleTypeDef huart2;


//uint32_t pulse1_value = 25000;
uint32_t pulse1_value = 12500000;
uint32_t pulse2_value = 12500;
uint32_t pulse3_value = 6250;
uint32_t pulse4_value = 3125;


volatile uint32_t ccr_content;


int main(void){

	HAL_Init();

	SystemClock_Config_HSE(SYS_CLOCK_FREQ_50_MHZ);

	GPIO_Init();

	UART2_Init();

	TIMER2_Init();


	if( HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_1) != HAL_OK)
	{
		Error_handler();
	}

	if( HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_2) != HAL_OK)
	{
		Error_handler();
	}

	if( HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_3) != HAL_OK)
	{
		Error_handler();
	}

	if( HAL_TIM_OC_Start_IT(&htimer2,TIM_CHANNEL_4) != HAL_OK)
	{
		Error_handler();
	}
	while(1);

	return 0;

}
void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_5;
	ledgpio.Mode = GPIO_MODE_OUTPUT_PP;
	ledgpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA,&ledgpio);
}


void TIMER2_Init(){

	TIM_OC_InitTypeDef time2OC_config;

	//initialise the TIMER2
	 htimer2.Instance = TIM2;
	 htimer2.Init.Period = 10000-1;
	 htimer2.Init.Prescaler = 4999;
	 if ( HAL_TIM_OC_Init(&htimer2) != HAL_OK)
	 {
		 Error_handler();
	 }


	//configuring the output channel
	 time2OC_config.OCMode = TIM_OCMODE_PWM1;
	 time2OC_config.OCPolarity = TIM_OCPOLARITY_HIGH;


	 time2OC_config.Pulse = (htimer2.Init.Period * 0.25);
	if(HAL_TIM_PWM_ConfigChannel(&htimer2, &time2OC_config, TIM_CHANNEL_1) != HAL_OK){
		Error_handler();
	}

	 time2OC_config.Pulse = (htimer2.Init.Period * 0.45);
	if(HAL_TIM_PWM_ConfigChannel(&htimer2, &time2OC_config, TIM_CHANNEL_2) != HAL_OK){
		Error_handler();
	}

	 time2OC_config.Pulse = (htimer2.Init.Period * 0.75);
	if(HAL_TIM_PWM_ConfigChannel(&htimer2, &time2OC_config, TIM_CHANNEL_3) != HAL_OK){
		Error_handler();
	}

	 time2OC_config.Pulse = (htimer2.Init.Period * 0.90);
	if(HAL_TIM_PWM_ConfigChannel(&htimer2, &time2OC_config, TIM_CHANNEL_4) != HAL_OK){
		Error_handler();
	}




}


void SystemClock_Config_HSE(uint8_t clock_freq ){
	RCC_OscInitTypeDef Osc_Init;
	RCC_ClkInitTypeDef Clock_Init;

	Osc_Init.OscillatorType = RCC_OSCILLATORTYPE_HSE | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_HSI;
	Osc_Init.HSEState = RCC_HSE_ON;
	Osc_Init.LSEState = RCC_LSE_ON;
	Osc_Init.HSIState = RCC_HSI_ON;
	Osc_Init.PLL.PLLState = RCC_PLL_ON;
	Osc_Init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clock_freq)
	 {
	  case SYS_CLOCK_FREQ_50_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 50;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Osc_Init.PLL.PLLR = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;

		 break;

	  case SYS_CLOCK_FREQ_84_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 84;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Osc_Init.PLL.PLLR = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV2;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV1;

		 break;

	  case SYS_CLOCK_FREQ_120_MHZ:
		  Osc_Init.PLL.PLLM = 4;
		  Osc_Init.PLL.PLLN = 120;
		  Osc_Init.PLL.PLLP = RCC_PLLP_DIV2;
		  Osc_Init.PLL.PLLQ = 2;
		  Osc_Init.PLL.PLLR = 2;
		  Clock_Init.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
								  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
		  Clock_Init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
		  Clock_Init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		  Clock_Init.APB1CLKDivider = RCC_HCLK_DIV4;
		  Clock_Init.APB2CLKDivider = RCC_HCLK_DIV2;

		 break;

	  default:
	   return ;
	 }

			if (HAL_RCC_OscConfig(&Osc_Init) != HAL_OK)
		{
				Error_handler();
		}



		if (HAL_RCC_ClockConfig(&Clock_Init, FLASH_LATENCY_2) != HAL_OK)
		{
			Error_handler();
		}


		/*Configure the systick timer interrupt frequency (for every 1 ms) */
		uint32_t hclk_freq = HAL_RCC_GetHCLKFreq();
		HAL_SYSTICK_Config(hclk_freq/1000);

		/**Configure the Systick
		*/
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

		/* SysTick_IRQn interrupt configuration */
		HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);



}

void UART2_Init(){

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}


}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
 {
   /* TIM3_CH1 toggling with frequency = 500 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
   {
	   ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,ccr_content+pulse1_value);
   }

   /* TIM3_CH2 toggling with frequency = 1000 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
   {
	   ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,ccr_content+pulse2_value);

   }

   /*
 TIM3_CH3 toggling with frequency = 2000 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
   {
	   ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_3);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,ccr_content+pulse3_value);

   }

   /* TIM3_CH4 toggling with frequency = 4000 Hz */
   if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
   {
	    ccr_content = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_4);
	   __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_4,ccr_content+pulse4_value);

   }
 }







void Error_handler(){

	while(1);
}
