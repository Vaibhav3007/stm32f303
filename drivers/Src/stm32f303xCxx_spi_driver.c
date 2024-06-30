/*
 * stm32f303xCxx_spi_driver.c
 *
 *  Created on: Jun 30, 2024
 *      Author: vaibhav
 */

#include "stm32f303xCxx_spi_driver.h"

/*********************************************************************
 * @function 		- SPI_PClkCtrl
 *
 * @brief 			- This function enables/disables clock for SPI,
 *
 * @param[in] 		- base address of the SPIx peripheral
 * @param[in] 		- ENABLE or DISABLE macros
 *
 * @return 			- none
 *
 * @Note			- none
 */
void SPI_PClkCtrl(SPI_RegDef_t *pSPIx, uint8_t E_D)
{
	if(E_D == ENABLE)
	{
		if(pSPIx == SPI1)			{	SPI1_CLK_EN();	}
		else if(pSPIx == SPI2)		{	SPI2_CLK_EN();	}
		else if(pSPIx == SPI3)		{	SPI3_CLK_EN();	}
	}
	else
	{
		if(pSPIx == SPI1)			{	SPI1_CLK_EN();	}
		else if(pSPIx == SPI2)		{	SPI2_CLK_EN();	}
		else if(pSPIx == SPI3)		{	SPI3_CLK_EN();	}
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
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp = 0;

	temp |= (pSPIHandle->SPI_Config.SPI_DeviceMode << 2);
	if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		temp &= ~(1 << 15);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		temp |= (1 << 15);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		temp &= ~(1 << 15);
		temp |= (1 << 10);
	}
	temp |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << 3);
	temp |= (pSPIHandle->SPI_Config.SPI_CPOL << 1);
	temp |= (pSPIHandle->SPI_Config.SPI_CPHA << 0);

	pSPIHandle->pSPIx->SPIx_CR1 |= temp;
	pSPIHandle->pSPIx->SPIx_CR2 |= (pSPIHandle->SPI_Config.SPI_DS << 8);
}

/*********************************************************************
 * @function 		- SPI_DeInit
 *
 * @brief 			- This function de-initializes a SPI peripheral
 *
 * @param[in] 		- base address of the SPI peripheral
 *
 * @return 			- none
 *
 * @Note			- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1) 			{ SPI1_REG_RESET(); }
	else if (pSPIx == SPI2) 	{ SPI2_REG_RESET(); }
	else if (pSPIx == SPI3)		{ SPI3_REG_RESET(); }
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
void SPI_Send_Data(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);

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
void SPI_Receive_Data(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

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
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t E_D);

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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

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
void SPI_IRQHandling(SPI_Handle_t *pHandle);

