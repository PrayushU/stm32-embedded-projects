/*
 * stm32f446xx.h
 *
 *  Created on: Apr 3, 2023
 *      Author: prayushu
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>

#define __vo volatile


/*****************************************Processor specific details**************************************/
/*
 * ARM cortex M4 NVIC ISERx register Address
 * 		- Interrupt set enable register
 */

#define NVIC_ISER0		((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1		((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2		((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3		((__vo uint32_t*)0xE000E10C)
#define NVIC_ISER4		((__vo uint32_t*)0xE000E110)
#define NVIC_ISER5		((__vo uint32_t*)0xE000E114)
#define NVIC_ISER6		((__vo uint32_t*)0xE000E118)
#define NVIC_ISER7		((__vo uint32_t*)0xE000E11C)

/*
 * ARM cortex M4 NVIC ICERx register Address
 * 		- Interrupt clear enable register
 */

#define NVIC_ICER0		((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1		((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2		((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3		((__vo uint32_t*)0xE000E18C)
#define NVIC_ICER4		((__vo uint32_t*)0xE000E190)
#define NVIC_ICER5		((__vo uint32_t*)0xE000E194)
#define NVIC_ICER6		((__vo uint32_t*)0xE000E198)
#define NVIC_ICER7		((__vo uint32_t*)0xE000E19C)

/*
 * ARM cortex m4 NVIC ISPRx registers address
 * 		- Interrupt Priority registers
 */
#define NVIC_PR_BASE_ADDR               ((__vo uint32_t*)0xE000E400)


#define NVIC

#define NO_PR_BITS_IMPLEMENTED	4
/*********************************************MCU peripheral registers definitions***************************************/


#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U //sram1 base address + 16kb (size of sram2)
#define ROM						0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

//Bus domains
#define PERIPH_BASE				0x40000000U
#define APB1PERIPH_BASE			PERIPH_BASE
#define APB2PERIPH_BASE			0x40010000U
#define AHB1PERIPH_BASE			0x40020000U
#define AHB2PERIPH_BASE			0x50000000U
#define AHB3PERIPH_BASE			0x60000000U

//Base addresses of peripherals which are hanging on AHB1 bus
#define GPIOA_BASEADDR			AHB1PERIPH_BASE
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)

#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)


/*Base addresses of peripheral which are hanging on APB1*/
//TIMx
#define TIM2_BASEADDR			APB1PERIPH_BASE
#define TIM3_BASEADDR			(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR			(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR			(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR			(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR			(APB1PERIPH_BASE + 0x1400)

#define TIM12_BASEADDR			(APB1PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR			(APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR			(APB1PERIPH_BASE + 0x2000)


//RTC & BKP register
#define RTC_BKP_BASEADDR		(APB1PERIPH_BASE + 0x2800)

//WWDG
#define WWDG_BASEADDR			(APB1PERIPH_BASE + 0x2C00)
//IWDG
#define IWDG_BASEADDR 			(APB1PERIPH_BASE + 0x3000)


//SPIx
#define SPI2_I2S2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_I2S3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)

//SPDIF-RX
#define SPDIF_RX_BASEADDR		(APB1PERIPH_BASE + 0x4000)


//USARTx
#define USART_2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART_3_BASEADDR		(APB1PERIPH_BASE + 0x4800)


//UARTx
#define UART_4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART_5_BASEADDR			(APB1PERIPH_BASE + 0x5000)


//I2Cx
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)


//CANx
#define CAN1_BASEADDR			(APB1PERIPH_BASE + 0x6400)
#define CAN2_BASEADDR			(APB1PERIPH_BASE + 0x6800)



//HDMI-CEC
#define HDMI_CEC_BASEADDR		(APB1PERIPH_BASE + 0x6C00)


//PWR
#define PWR_BASEADDR			(APB1PERIPH_BASE + 0x7000)


//DAC
#define DAC_BASEADDR			(APB1PERIPH_BASE + 0x7400)


//Base addresses for APB2 peripherals
//TIMx
#define TIM1_BASEADDR			(APB2PERIPH_BASE)
#define TIM8_BASEADDR			(APB2PERIPH_BASE + 0x0400)

//USARTx
#define USART_1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART_6_BASEADDR		(APB2PERIPH_BASE + 0x1400)


//SPIx
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR			(APB2PERIPH_BASE + 0x3400)


//SYSCFG
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)


//EXTI
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)






/*****************************************************Peripheral register definition structures**********************************************/


typedef struct
{
	__vo uint32_t MODER;				//GPIO port mode register
	__vo uint32_t OTYPER;			//GPIO port output type register
	__vo uint32_t OSPEEDER;			//GPIO port output speed register
	__vo uint32_t PUPDR;				//GPIO port pull-up/pull-down register
	__vo uint32_t IDR;				//GPIO port input data register
	__vo uint32_t ODR;				//GPIO port output data register
	__vo uint32_t BSRR;				//GPIO port bit set/reset register
	__vo uint32_t LCKR;				//GPIO port configuration lock register
	__vo uint32_t AFR[2];			//GPIO port alternation function low and high register register
}GPIO_RegDef_t;

#define GPIOx_AFRL	0
#define GPIOx_AFRH	1


//EXTI register defintion
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;



//RCC register structure
typedef struct
{

	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	__vo uint32_t reserved;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t reserved2[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	__vo uint32_t reserved3;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t reserved4[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t reserved7;
	__vo uint32_t APB1LPENR;
	__vo uint32_t PB2LPENR;
	uint32_t reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;


//SYSCFG register definition
typedef struct{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t RESERVED2[2];
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;

//SPI register definition
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;


typedef struct{

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
}I2C_RegDef_t;

/*peripheral definition: type casting the gpio_base address to the struct pointer type*/
#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1					((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2					((SPI_RegDef_t*)SPI2_I2S2_BASEADDR)
#define SPI3					((SPI_RegDef_t*)SPI3_I2S3_BASEADDR)

#define I2C1					((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2					((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3					((I2C_RegDef_t*)I2C3_BASEADDR)
#define I2C4					((I2C_RegDef_t*)I2C4_BASEADDR)
/*
 * Clock Enable Macro for GPIOx peripheral
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7))


/*
 * Clock Enable Macros for I2Cx peripheral
 */
#define I2C1_PCLK_EN()	((RCC->APB1ENR) |= (1 << 21))
#define I2C2_PCLK_EN()	((RCC->APB1ENR) |= (1 << 22))
#define I2C3_PCLK_EN()	((RCC->APB1ENR) |= (1 << 23))


/*
 * Clock Enable Macro for SPIx peripheral
 */
#define SPI1_PCLK_EN()	((RCC->APB2ENR) |= (1 << 12))
#define SPI2_PCLK_EN()	((RCC->APB1ENR) |= (1 << 14))
#define SPI3_PCLK_EN()	((RCC->APB1ENR) |= (1 << 15))
#define SPI4_PCLK_EN()	((RCC->APB2ENR) |= (1 << 13))


/*
 * Clock Enable Macro for USARTx and UARTx  peripheral
 */
#define USART1_PCLK_EN()	((RCC->APB2ENR) |= (1 << 4))
#define USART2_PCLK_EN()	((RCC->APB1ENR) |= (1 << 17))
#define USART3_PCLK_EN()	((RCC->APB1ENR) |= (1 << 18))
#define UART4_PCLK_EN()		((RCC->APB1ENR) |= (1 << 19))
#define UART5_PCLK_EN()		((RCC->APB1ENR) |= (1 << 20))
#define USART6_PCLK_EN()	((RCC->APB2ENR) |= (1 << 6))

/*
 * Clock enable Macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()	((RCC->APB2ENR) |= (1 << 14))




/****************CLK DISABLE MACROS**********/
/*
 * Clock Disable Macro for GPIO
 */
#define GPIOA_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 0))
#define GPIOB_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 1))
#define GPIOC_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 2))
#define GPIOD_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 3))
#define GPIOE_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 4))
#define GPIOF_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 5))
#define GPIOG_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 6))
#define GPIOH_PCLK_DIS()	(RCC->AHB1ENR &= (1 << 7))


/*
 * Clock DISBALE Macros for I2Cx peripheral
 */
#define I2C1_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 21))
#define I2C2_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 22))
#define I2C3_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 23))


/*
 * Clock DISABLE Macro for SPIx peripheral
 */
#define SPI1_PCLK_DIS()	((RCC->APB2ENR) &= (1 << 12))
#define SPI2_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 14))
#define SPI3_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 15))
#define SPI4_PCLK_DIS()	((RCC->APB2ENR) &= (1 << 13))


/*
 * Clock DISABLE Macro for USARTx and UARTx  peripheral
 */
#define USART1_PCLK_DIS()	((RCC->APB2ENR) &= (1 << 4))
#define USART2_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 17))
#define USART3_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 18))
#define UART4_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 19))
#define UART5_PCLK_DIS()	((RCC->APB1ENR) &= (1 << 20))
#define USART6_PCLK_DIS()	((RCC->APB2ENR) &= (1 << 6))



/*
 * Clock DISABLE Macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DIS()	((RCC->APB2ENR) &= (1 << 14))

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= (1 << 0));}while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= (1 << 1));}while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= (1 << 2));}while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= (1 << 3));}while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= (1 << 4));}while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= (1 << 5));}while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= (1 << 6));}while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= (1 << 7));}while(0)


#define GPIO_BASEADDR_TO_CODE(x)	((x == GPIOA)? 0 :\
									(x == GPIOB)? 1 :\
									(x == GPIOC)? 2 :\
									(x == GPIOD)? 3 :\
									(x == GPIOE)? 4 :\
									(x == GPIOF)? 5 :\
									(x == GPIOG)? 6 :\
									(x == GPIOH)? 7 :0)


/***IRQ number defintion of stm32f44xx family*****/
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


/*
 * IRQ Priorities, macros for all possible priority levels
 */
#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI12		12




/*
 * IRQ handler
 */

/**********************************************
 *Bit position definitions of SPI peripheral
 **********************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15


/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/**********************************************
 *Bit position definitions of I2C peripheral
 **********************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE				0
#define I2C_CR1_SM_BUS			1
#define I2C_CR1_SM_TYPE			3
#define I2C_CR1_ENARP			4
#define I2C_CR1_ENPEC			5
#define I2C_CR1_ENGC			6
#define I2C_CR1_NO_STRET_CH		7
#define I2C_CR1_START			8
#define I2C_CR1_STOP			9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SW_RST			15


/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ			0
#define I2C_CR2_ITERR_EN		1
#define I2C_CR2_ITEVT_EN		3
#define I2C_CR2_ITBUF_EN		4
#define I2C_CR2_DMA_EN			5
#define I2C_CR2_LAST			6

/*
 * Bit position definitions I2C_SR1
 */
#define I2C_SR1_SB				0
#define I2C_SR1_ADDR			1
#define I2C_SR1_BTF				2
#define I2C_SR1_ADD10			3
#define I2C_SR1_STOPF			4
#define I2C_SR1_RXNE			6
#define I2C_SR1_TXE				7
#define I2C_SR1_BERR			8
#define I2C_SR1_ARLO			9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL				0
#define I2C_SR2_BUSY			1
#define I2C_SR2_TRA				2
#define I2C_SR2_GENCALL			3
#define I2C_SR2_SMBDEFAULT		4
#define I2C_SR2_SMBHOST			6
#define I2C_SR2_DUALF			7
#define I2C_SR2_PEC				8

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR				0
#define I2C_CCR_DUTY			14
#define I2C_CCR_FS				15

/**************************************************Some generic MACROS*******************************************/
#define ENABLE 					1
#define DISABLE 				0
#define SET 					ENABLE
#define RESET 					DISABLE
#define CLEAR 					DISABLE
#define GPIO_PIN_RESET 			RESET
#define GPIO_PIN_SET			SET
#define FLAG_SET				1
#define	FLAG_RESET				0

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"

#endif /* INC_STM32F446XX_H_ */
