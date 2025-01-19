/*
 * 002_led_button.c
 *
 *  Created on: Apr 4, 2023
 *      Author: prayushu
 */


#include "stm32f446xx.h"

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

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioBtn);

	/*
	 * Push button
	 * pin=PC13
	 * Push button = Ground
	 * Release button = VDD
	 */

	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;




	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GpioLed);







	while(1)
	{

		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BTN_PRESSED ){
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_8);
		}


	}
	return 0;

}
