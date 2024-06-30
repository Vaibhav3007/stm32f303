/*
 * stm32f303xCxx_gpio_driver.c
 *
 *  Created on: May 20, 2024
 *      Author: vaibhav
 */


#include "stm32f303xCxx_gpio_driver.h"

/*********************************************************************
 * @function 		- GPIO_PClkCtrl
 *
 * @brief 			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_PClkCtrl(GPIO_RegDef_t *pGPIOx, uint8_t E_D)
{
	if(E_D == ENABLE)
	{
		if(pGPIOx == GPIOA)			{	GPIOA_CLK_EN();	}
		else if(pGPIOx == GPIOB) 	{	GPIOB_CLK_EN();	}
		else if(pGPIOx == GPIOC) 	{	GPIOC_CLK_EN();	}
		else if(pGPIOx == GPIOD) 	{	GPIOD_CLK_EN();	}
		else if(pGPIOx == GPIOE) 	{	GPIOE_CLK_EN();	}
		else if(pGPIOx == GPIOF) 	{	GPIOF_CLK_EN();	}
	}
	else
	{
		if(pGPIOx == GPIOA)			{	GPIOA_CLK_DI();	}
		else if(pGPIOx == GPIOB) 	{	GPIOB_CLK_DI();	}
		else if(pGPIOx == GPIOC) 	{	GPIOC_CLK_DI();	}
		else if(pGPIOx == GPIOD) 	{	GPIOD_CLK_DI();	}
		else if(pGPIOx == GPIOE) 	{	GPIOE_CLK_DI();	}
		else if(pGPIOx == GPIOF) 	{	GPIOF_CLK_DI();	}
	}
}

/*********************************************************************
 * @function 		- GPIO_Init
 *
 * @brief 			- This function in initializes a GPIO with its mode, output type, speed,
 * 					- pull up-down configuration,alternate function
 *
 * @param[in] 		- base address of the GPIO peripheral
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	//1. Configure mode of port
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->GPIOx_MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//clear
		pGPIOHandle->pGPIOx->GPIOx_MODER |= temp;//To set always use bitwise OR rather than equating a variable
		temp = 0;
	}
	else
	{
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTRUPT_FALL)
		{
			// Configure FTSR (Falling Trigger Selection Register)
			EXTI->EXTI_FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTRUPT_RISE)
		{
			// Configure RTSR (Rising Trigger Selection Register)
			EXTI->EXTI_RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_INTRUPT_RISE_FALL)
		{
			//configure both FTSR and RTSR
			EXTI->EXTI_RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		// Configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLCK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] = (portcode << (temp2 * 4));

		// Enable EXTI interrupt delivery using IMR
		EXTI->EXTI_IMR1 |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}
	//2. Configure speed of port
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIOx_OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_OSPEEDR |= temp;
	temp = 0;

	//3. Configure pull up down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPd |= temp;
	temp = 0;

	//4. Configure output type of GPIO
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->GPIOx_OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->GPIOx_OTYPER |= temp;
	temp = 0;

	//5. Configure alternate function only if Pin Mode is alternate
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FN)
	{
		uint8_t temp1,temp2;
		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);//To get array index
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);//To get shift position
		pGPIOHandle->pGPIOx->GPIOx_AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->GPIOx_AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFn << (4 * temp2));
	}
}

/*********************************************************************
 * @function 		- GPIO_DeInit
 *
 * @brief 			- This function de-initializes a GPIO
 *
 * @param[in] 		- base address of the GPIO peripheral
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)	{	GPIOA_REG_RESET();	}
	if (pGPIOx == GPIOB)	{	GPIOB_REG_RESET();	}
	if (pGPIOx == GPIOC)	{	GPIOC_REG_RESET();	}
	if (pGPIOx == GPIOD)	{	GPIOD_REG_RESET();	}
	if (pGPIOx == GPIOE)	{	GPIOE_REG_RESET();	}
	if (pGPIOx == GPIOF)	{	GPIOF_REG_RESET();	}
}

/*********************************************************************
 * @function 		- GPIO_ReadFromPin
 *
 * @brief 			- This function reads a specific pin of  GPIO
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- pin number to be read
 *
 * @return 			- value of read pin
 *
 * @Note			- none
 */
uint8_t GPIO_ReadFrmIpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->GPIOx_IDR >> PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @function 		- GPIO_ReadFromPort
 *
 * @brief 			- This function reads from a port
 *
 * @param[in] 		- base address of the GPIO peripheral
 *
 * @return 			- read value of specified port
 *
 * @Note			- none
 */
uint16_t GPIO_ReadFrmIpPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->GPIOx_IDR);
	return value;
}

/*********************************************************************
 * @function 		- GPIO_WriteToPin
 *
 * @brief 			- This function writes user value to specified pin
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- pin number of specified port
 * @param[in]		- value to be written
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_WriteToOpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == SET)
	{
		pGPIOx->GPIOx_ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->GPIOx_ODR &= (1 << PinNumber);
	}
}

/*********************************************************************
 * @function 		- GPIO_WriteToPort
 *
 * @brief 			- This function writes user value to specified port
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- value to be written
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_WriteToOpPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->GPIOx_ODR = Value;
}

/*********************************************************************
 * @function 		- GPIO_TogglePin
 *
 * @brief 			- This function toggles value of specified pin
 *
 * @param[in] 		- base address of the GPIO peripheral
 * @param[in]		- pin number
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_ToggleOpPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->GPIOx_ODR ^= (1 << PinNumber);
}

/*********************************************************************
 * @function 		- GPIO_IRQConfig
 *
 * @brief 			- This function sets/resets NVIC in the processor
 *
 * @param[in] 		- IRQ number
 * @param[in]		- Enable/Disable
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t E_D)
{
	if (E_D == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 64)
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else if (IRQNumber > 64 && IRQNumber <= 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
			else if (IRQNumber > 31 && IRQNumber <= 64)
		{
			//program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber%32));
		}
			else if (IRQNumber > 64 && IRQNumber <= 96)
		{
			//program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber%64));
		}
	}
}

/*********************************************************************
 * @function 		- GPIO_IRQPriorityConfig
 *
 * @brief 			- This function sets priority of interrupt
 *
 * @param[in] 		- IRQ number
 * @param[in]		- IRQ priority (0 to 15)
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + NO_PR_BITS_IMPLEMENTED;
	*(NVIC_IPR_BASEADDR+(iprx)) |= (IRQPriority << shift_amount);
}

/*********************************************************************
 * @function 		- GPIO_IRQHandling
 *
 * @brief 			- This function handles IRQ by acknowledging interrupt and resetting pending register
 *
 * @param[in] 		- Pin number of EXTI
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//Clear EXTI PR register
	if(EXTI->EXTI_PR1 & (1 << PinNumber))
	{
		EXTI->EXTI_PR1 |= (1 << PinNumber);
	}
}
