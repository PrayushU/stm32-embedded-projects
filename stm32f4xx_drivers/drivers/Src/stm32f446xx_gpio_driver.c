/*
 * stm32f446xx_gpio_driver.c
 *
 *  Created on: Apr 3, 2023
 *      Author: prayushu
 */


#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"



/*
 * Peripheral clock setup
 *
 *
 * @fn					-	GPIO_PeriClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given GPIO port
 *
 *
 * @param[in]			-	base address of the gpio peripheral
 * @param[in]			-	ENABLE OR DIABLE macros
 * @param[in]			-
 *
 * @return				-
 *
 *
 *
 * @note
 */


void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{

	if(EnOrDi == ENABLE){

		if	(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else{
		if	(pGPIOx == GPIOA){
			GPIOA_PCLK_DIS();
		}if (pGPIOx == GPIOB){
			GPIOB_PCLK_DIS();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DIS();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DIS();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DIS();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DIS();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DIS();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DIS();
		}
	}

}

/*
 * Init and De-Init
 */

/*
 * Peripheral clock setup
 *
 *
 * @fn					-	GPIO_Init
 *
 * @brief				-
 *
 *
 * @param[in]			-	base address of the gpio peripheral
 * @param[in]			-	s
 * @param[in]			-
 *
 * @return				-
 *
 *
 *
 * @note
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp=0;

	//enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//configure mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else{
		//Interrupt mode

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// configure the FTSR

			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear the RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			//configure the RTSR

			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){

			//configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	temp=0;

	//configure the speed  settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~( 0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OSPEEDER |= temp;

	temp = 0;

	//configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType) << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	//configure the alternate functions

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){

		//configure alt function register
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if	(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}

}

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value == GPIO_PIN_SET){
		//write 1 to output data register at the bit field corresponding to the pin nuber
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}

}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){

	pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1 << PinNumber);

}


void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi){

	if(EnOrDi == ENABLE){

		if(IRQNumber <= 31){
			//for IRQSR0
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if((IRQNumber > 31) && (IRQNumber < 64)){
			//for IRQSR1
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}else if((IRQNumber >= 64) && (IRQNumber < 96)){
			// for IRQSR2
			*NVIC_ISER2 |= (1 << (IRQNumber%32));
		}else if((IRQNumber >= 96) && (IRQNumber < 129)){
			// for IRQSR2
			*NVIC_ISER3 |= (1 << (IRQNumber%32));
		}
	}else{
		if(IRQNumber <=31){
			*NVIC_ICER0 |= (1 << IRQNumber);

		}else if((IRQNumber > 31) && (IRQNumber < 64)){
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if((IRQNumber >= 64) && (IRQNumber < 96)){
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}else if((IRQNumber >= 96) && (IRQNumber < 129)){
			*NVIC_ICER3 |= (1 << (IRQNumber % 32));
		}
	}


}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority){

	//1. First lets find out the iqr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	//lower 4 bits are not implemented in stm32 implementation, so we have to shift the set value
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << (8 *  shift_amount));

}

void GPIO_IRQHandling(uint8_t PinNumber){

	if(EXTI->PR & (1 << PinNumber)){
		//clear the exti pr register corresponding to the pin number
		EXTI->PR |= (1 << PinNumber);
	}
}

