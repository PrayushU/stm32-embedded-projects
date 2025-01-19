/*
 * test.c
 *
 *  Created on: 11 Apr 2023
 *      Author: prayushu
 */





#include "stm32f446xx.h"


int main(void){

	GPIOA_PCLK_EN();
	uint32_t address = 0x40020000;
	uint32_t *ptr = &address;
	*ptr |= 1;

	while(1);

	/*
	 temp = GPIOA->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	 *
	tGpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	tGpio.GPIO_PinConfig.GPIO_PinOPType =GPIO_OP_TYPE_OD;
	tGpio.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;
	tGpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	tGpio.GPIO_PinConfig.GPIO_PinNumber=0;
	GPIO_Init(&tGpio);
	*/


}
