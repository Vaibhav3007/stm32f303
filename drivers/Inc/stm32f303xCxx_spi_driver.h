/*
 * stm32f303xCxx_spi_driver.h
 *
 *  Created on: Jun 30, 2024
 *      Author: vaibhav
 */

#ifndef INC_STM32F303XCXX_SPI_DRIVER_H_
#define INC_STM32F303XCXX_SPI_DRIVER_H_

#include "stm32f303xCxx.h"

/*
 * SPI configuration structure
 */
typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DS;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * SPI handle structure
 */
typedef struct{
	SPI_RegDef_t	*pSPIx;
	SPI_Config_t 	SPI_Config;
}SPI_Handle_t;

/*
 *@SPI_DeviceMode
 */
#define SPI_MODE_SLAVE 	0
#define SPI_MODE_MASTER 1

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY	3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_DIV2			0
#define SPI_SCLK_DIV4			1
#define SPI_SCLK_DIV8			2
#define SPI_SCLK_DIV16			3
#define SPI_SCLK_DIV32			4
#define SPI_SCLK_DIV64			5
#define SPI_SCLK_DIV128			6
#define SPI_SCLK_DIV256			7

/*
 * @SPI_DS
 */
#define SPI_DS_4BIT		4
#define SPI_DS_5BIT		5
#define SPI_DS_6BIT		6
#define SPI_DS_7BIT		7
#define SPI_DS_8BIT		8
#define SPI_DS_9BIT		9
#define SPI_DS_10BIT	10
#define SPI_DS_11BIT	11
#define SPI_DS_12BIT	12
#define SPI_DS_13BIT	13
#define SPI_DS_14BIT	14
#define SPI_DS_15BIT	15
#define SPI_DS_16BIT	16

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LO		0
#define SPI_CPOL_HI		1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LO		0
#define SPI_CPHA_HI		1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/******************************************* Supported APIs by this driver*******************************************/
void SPI_PClkCtrl(SPI_RegDef_t *pSPIx, uint8_t E_D); //Enable or Disable a peripheral
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);
void SPI_Send_Data(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_Receive_Data(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQConfig(uint8_t IRQNumber, uint8_t E_D);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

#endif /* INC_STM32F303XCXX_SPI_DRIVER_H_ */
