/*
 * stm32f303xCxx.h
 *
 *  Created on: May 19, 2024
 *      Author: vaibh
 */

#ifndef INC_STM32F303XCXX_H_
#define INC_STM32F303XCXX_H_

#include <stdint.h>

/*
 * Global definitions
 */
#define __vo						volatile
#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE

/*
 * Cortex M4 NVIC ISERx specific details
 */
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)
/*
 * Cortex M4 NVIC ISERx specific details
 */
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)
/*
 * Cortex M4 Priority Register Base Address
 */
#define NVIC_IPR_BASEADDR			((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4 //4-bit interrupt priority defined by ST

/*
 * Base addresses of memories
 */
#define FLASH_BASEADDR				0x08000000U
#define SRAM_BASEADDR				0x20000000U
#define SYSTEM_MEMOERY_BASEADDR		0x1FFFD800U
#define ROM							SYSTEM_MEMOERY_BASEADDR
#define SRAM						SRAM_BASEADDR

/*
 * base addresses of AHB and APB bus
 */
#define PERIPHERAL_BASE				0x40000000U
#define	APB1PERIPHERAL_BASEADDR		PERIPHERAL_BASE
#define	APB2PERIPHERAL_BASEADDR		0x40010000U
#define	AHB1PERIPHERAL_BASEADDR		0x40020000U
#define	AHB2PERIPHERAL_BASEADDR		0x48000000U
#define	AHB3PERIPHERAL_BASEADDR		0x50000000U

/*
 * Base addresses of all peripherals (GPIOx, I2Cx, SPIx, UARTx, EXTI, SYSCFG)
 */
//APB1 bus
#define SPI2_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x3800)
#define SPI3_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x3C00)
#define USART2_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x4400)
#define USART3_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x4800)
#define UART4_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x4C00)
#define UART5_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x5000)
#define I2C1_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x5400)
#define	I2C2_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x5800)
//APB2 bus
#define SYSCFG_BASEADDR				APB2PERIPHERAL_BASEADDR
#define EXTI_BASEADDR				(APB2PERIPHERAL_BASEADDR+0x0400)
#define SPI1_BASEADDR				(APB2PERIPHERAL_BASEADDR+0x3000)
#define USART1_BASEADDR				(APB2PERIPHERAL_BASEADDR+0x3800)
//AHB1 bus
#define RCC_BASEADDR				(AHB1PERIPHERAL_BASEADDR+0x1000)
//AHB2 bus
#define GPIOA_BASEADDR				(AHB2PERIPHERAL_BASEADDR)
#define GPIOB_BASEADDR				(AHB2PERIPHERAL_BASEADDR+0x0400)
#define GPIOC_BASEADDR				(AHB2PERIPHERAL_BASEADDR+0x0800)
#define GPIOD_BASEADDR				(AHB2PERIPHERAL_BASEADDR+0x0C00)
#define GPIOE_BASEADDR				(AHB2PERIPHERAL_BASEADDR+0x1000)
#define GPIOF_BASEADDR				(AHB2PERIPHERAL_BASEADDR+0x1400)
//I2Cx base addresses
#define I2C1_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x5400)
#define I2C2_BASEADDR				(APB1PERIPHERAL_BASEADDR+0x5800)
/*
 * RCC register definition structure
 */
typedef struct{
	__vo uint32_t RCC_CR;
	__vo uint32_t RCC_CFGR;
	__vo uint32_t RCC_CIR;
	__vo uint32_t RCC_APB2RSTR;
	__vo uint32_t RCC_APB1RSTR;
	__vo uint32_t RCC_AHBENR;
	__vo uint32_t RCC_APB2ENR;
	__vo uint32_t RCC_APB1ENR;
	__vo uint32_t RCC_BDCR;
	__vo uint32_t RCC_CSR;
	__vo uint32_t RCC_AHBRSTR;
	__vo uint32_t RCC_CFGR2;
	__vo uint32_t RCC_CFGR3;
}RCC_RegDef_t;

/*
 * GPIO structure
 */
typedef struct{
	__vo uint32_t GPIOx_MODER;
	__vo uint32_t GPIOx_OTYPER;
	__vo uint32_t GPIOx_OSPEEDR;
	__vo uint32_t GPIOx_PUPDR;
	__vo uint32_t GPIOx_IDR;
	__vo uint32_t GPIOx_ODR;
	__vo uint32_t GPIOx_BSRR;
	__vo uint32_t GPIOx_LCKR;
	__vo uint32_t GPIOx_AFR[2];
	__vo uint32_t GPIOx_BRR;
}GPIO_RegDef_t;

/*
 * EXTI register definition structure
 */
typedef struct{
	__vo uint32_t EXTI_IMR1;
	__vo uint32_t EXTI_EMR1;
	__vo uint32_t EXTI_RTSR1;
	__vo uint32_t EXTI_FTSR1;
	__vo uint32_t EXTI_SWIER1;
	__vo uint32_t EXTI_PR1;
	__vo uint32_t EXTI_IMR2;
	__vo uint32_t EXTI_EMR2;
	__vo uint32_t EXTI_RTSR2;
	__vo uint32_t EXTI_FTSR2;
	__vo uint32_t EXTI_SWIER2;
	__vo uint32_t EXTI_PR2;
}EXTI_RegDef_t;

/*
 * SYSCFG register definition structure
 */
typedef struct{
	__vo uint32_t SYSCFG_CFGR1;
	__vo uint32_t SYSCFG_RCR;
	__vo uint32_t SYSCFG_EXTICR[4];
//	__vo uint32_t SYSCFG_CFGR4;
}SYSCFG_RegDef_t;

/*
 * I2C peripheral register definition
 */
typedef struct{
	__vo uint32_t I2C_CR1;
	__vo uint32_t I2C_CR2;
	__vo uint32_t I2C_OAR1;
	__vo uint32_t I2C_OAR2;
	__vo uint32_t I2C_TIMINGR;
	__vo uint32_t I2C_TIMEOUTR;
	__vo uint32_t I2C_ISR;
	__vo uint32_t I2C_ICR;
	__vo uint32_t I2C_PECR;
	__vo uint32_t I2C_RXDR;
	__vo uint32_t I2C_TXDR;
}I2C_RegDef_t;

/*
 *  Peripherals as register definition structure member elements
 */
#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)
#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)
#define I2C1		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASEADDR)
/*
 * GPIO peripheral clock enable
 */
#define GPIOA_CLK_EN()		(RCC->RCC_AHBENR |= (1 << 17))
#define GPIOB_CLK_EN()		(RCC->RCC_AHBENR |= (1 << 18))
#define GPIOC_CLK_EN()		(RCC->RCC_AHBENR |= (1 << 19))
#define GPIOD_CLK_EN()		(RCC->RCC_AHBENR |= (1 << 20))
#define GPIOE_CLK_EN()		(RCC->RCC_AHBENR |= (1 << 21))
#define GPIOF_CLK_EN()		(RCC->RCC_AHBENR |= (1 << 22))
/*
 * GPIO peripheral clock disable
 */
#define GPIOA_CLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 17))
#define GPIOB_CLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 18))
#define GPIOC_CLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 19))
#define GPIOD_CLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 20))
#define GPIOE_CLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 21))
#define GPIOF_CLK_DI()		(RCC->RCC_AHBENR &= ~(1 << 22))
/*
 * GPIO peripheral register reset
 */
#define GPIOA_REG_RESET()	do {(RCC->RCC_AHBRSTR |= (1 << 17)); (RCC->RCC_AHBRSTR &= ~(1 << 17));} while(0)
#define GPIOB_REG_RESET()	do {(RCC->RCC_AHBRSTR |= (1 << 18)); (RCC->RCC_AHBRSTR &= ~(1 << 18));} while(0)
#define GPIOC_REG_RESET()	do {(RCC->RCC_AHBRSTR |= (1 << 19)); (RCC->RCC_AHBRSTR &= ~(1 << 19));} while(0)
#define GPIOD_REG_RESET()	do {(RCC->RCC_AHBRSTR |= (1 << 20)); (RCC->RCC_AHBRSTR &= ~(1 << 20));} while(0)
#define GPIOE_REG_RESET()	do {(RCC->RCC_AHBRSTR |= (1 << 21)); (RCC->RCC_AHBRSTR &= ~(1 << 21));} while(0)
#define GPIOF_REG_RESET()	do {(RCC->RCC_AHBRSTR |= (1 << 22)); (RCC->RCC_AHBRSTR &= ~(1 << 22));} while(0)
/*
 * SYSCFG peripheral clock control
 */
#define SYSCFG_PCLCK_EN()	(RCC->RCC_APB2ENR |= (1 << 0))
#define SYSCFG_PCLCK_DI()	(RCC->RCC_APB2ENR &= ~(1 << 0))
/*
 * I2C peripheral clock control
 */
#define I2C1_PCLCK_EN()		(RCC->RCC_APB1ENR |= (1 << 21))
#define I2C2_PCLCK_EN()		(RCC->RCC_APB1ENR |= (1 << 22))
#define I2C1_PCLCK_DI()		(RCC->RCC_APB1ENR &= ~(1 << 21))
#define I2C2_PCLCK_DI()		(RCC->RCC_APB1ENR &= ~(1 << 22))
/*
 * Get port code between 0 and 5 for a given GPIO base address
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 :\
										 (x == GPIOB) ? 1 :\
										 (x == GPIOC) ? 2 :\
										 (x == GPIOD) ? 3 :\
										 (x == GPIOE) ? 4 :\
										 (x == GPIOF) ? 5 :0)
/*
 * IRQ number for EXTI lines
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/*
 * Interrupt programmable priority level
 */
#define PRI_0			0
#define PRI_1			1
#define PRI_2			2
#define PRI_3			3
#define PRI_4			4
#define PRI_5			5
#define PRI_6			6
#define PRI_7			7
#define PRI_8			8
#define PRI_9			9
#define PRI_10			10
#define PRI_11			11
#define PRI_12			12
#define PRI_13			13
#define PRI_14			14
#define PRI_15			15

/*
 * I2Cx Bit position definitions
 */



#include "stm32f303xCxx_gpio_driver.h"
#include "stm32f303xCxx_i2c_driver.h"

#endif /* INC_STM32F303XCXX_H_ */
