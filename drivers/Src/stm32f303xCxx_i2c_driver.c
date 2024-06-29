/*
 * stm32f303xCxx_i2c_driver.c
 *
 *  Created on: Jun 5, 2024
 *      Author: vaibh
 */


#include "stm32f303xCxx.h"
#include "stm32f303xCxx_i2c_driver.h"

void I2C_PCtrl(I2C_RegDef_t *pI2Cx, uint8_t E_D)
{
	if (E_D == ENABLE)		{	pI2Cx->I2C_CR1 |= (1 << 0);		}
	else					{	pI2Cx->I2C_CR1 &= ~(1 << 0);	}
}

void I2C_PClckCtrl(I2C_RegDef_t *pI2Cx, uint8_t E_D)
{
	if(E_D == ENABLE)
	{
		if(pI2Cx == I2C1)	{	I2C1_PCLCK_EN();	}
		else				{	I2C2_PCLCK_EN();	}
	}
	else
	{
		if(pI2Cx == I2C1)	{	I2C1_PCLCK_DI();	}
		else				{	I2C2_PCLCK_DI();	}
	}
}

void I2C_Init(I2C_Handle_t *pI2CHandle);

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		RCC->RCC_APB1RSTR |= (1 << 21);
		RCC->RCC_APB1RSTR &= ~(1 << 21);
	}
	else
	{
		RCC->RCC_APB1RSTR |= (1 << 22);
		RCC->RCC_APB1RSTR &= ~(1 << 22);
	}
}
