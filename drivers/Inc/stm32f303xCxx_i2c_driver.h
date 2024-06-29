/*
 * stm32f303xCxx_i2c_driver.h
 *
 *  Created on: Jun 5, 2024
 *      Author: vaibh
 */

#ifndef INC_STM32F303XCXX_I2C_DRIVER_H_
#define INC_STM32F303XCXX_I2C_DRIVER_H_

#include "stm32f303xCxx.h"

/*
 * I2C Configuration structure
 */
typedef struct{
	uint32_t 	I2C_SCLSpeed;
	uint8_t 	I2C_DeviceAddress;
	uint8_t 	I2C_AckControl;
	uint8_t 	I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * I2C Handle structure
 */
typedef struct{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
}I2C_Handle_t;

/*
 * SCL Speed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM2K	200000
#define I2C_SCL_SPEED_FM4K	400000
/*
 * ACK control
 */
#define I2C_ACK_SLAVE		0
#define I2C_NACK_SLAVE		1
/*
 * @I2C FM Duty Cycle
 */

/*
 * Supported APIs
 */
void I2C_PClckCtrl(I2C_RegDef_t *pI2Cx, uint8_t E_D);

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t E_D);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_PCtrl(I2C_RegDef_t *pI2Cx, uint8_t E_D);

#endif /* INC_STM32F303XCXX_I2C_DRIVER_H_ */
