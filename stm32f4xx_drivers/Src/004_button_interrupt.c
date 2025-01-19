/*
 * 002_led_button.c
 *
 *  Created on: Apr 4, 2023
 *      Author: prayushu
 */


#include "stm32f446xx.h"
#include <string.h>

#define BTN_PRESSED 0


void delay(void){

	for(uint32_t i =0; i < 500000/2; i++);
}

int main(void){
	/*
	 * in STM32 Nucleo board led in:
	 * 		pin=PA5(pin21),
	 * 		Name=D13
	 * 	Pin high = LED ON
	 * 	Pin Low  = LED OFF
	 */




	GPIO_Handle_t GpioLed, GpioBtn;


	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	/*
	 * Push button
	 * pin=PC13
	 * Push button = Ground
	 * Release button = VDD
	 */

	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;




	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioBtn);


	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI12);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);




	while(1);

}

void EXTI15_10_IRQHandler(void){
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}
