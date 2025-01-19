/*
 * 001led_toggle.c
 *
 *  Created on: Apr 4, 2023
 *      Author: prayushu
 */


#include "stm32f446xx.h"

void delay(void){

	for(uint32_t i =0; i < 500000; i++);
}

int main(void){
	/*
	 * in STM32 Nucleo board led in:
	 * 		pin=PA5(pin21),
	 * 		Name=D13
	 * 	Pin high = LED ON
	 * 	Pin Low  = LED OFF
	 */

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	/*
	 * Push button
	 * pin=PC13
	 * Push button = Ground
	 * Release button = VDD
	 */

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay();
	}

	return 0;

}
