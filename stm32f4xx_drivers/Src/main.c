/*
 * main.c
 *
 *  Created on: 5 Apr 2023
 *      Author: prayushu
 */


#include "stm32f446xx.h"

int main(void){


	return 0;
}


void EXTI0_IRQHandler(void){

	//handle the interrupt
	GPIO_IRQHandling(0);
}
