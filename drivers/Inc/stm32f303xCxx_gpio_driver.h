/*
 * stm32f303xCxx_gpio_driver.h
 *
 *  Created on: May 20, 2024
 *      Author: vaibh
 */

#ifndef INC_STM32F303XCXX_GPIO_DRIVER_H_
#define INC_STM32F303XCXX_GPIO_DRIVER_H_

#include "stm32f303xCxx.h"

/*
 * Structure to configure GPIO pins
 */
typedef struct{
	uint8_t 	GPIO_PinNumber;			/*!<possible values from @GPIO_PIN_NUMS>*/
	uint8_t 	GPIO_PinMode;			/*!<possible values from @GPIO_PIN_MODES>*/
	uint8_t 	GPIO_PinOutType;
	uint8_t 	GPIO_PinOutSpeed;
	uint8_t 	GPIO_PinPuPd;
	uint8_t 	GPIO_PinAltFn;
}GPIO_PinConfig_t;

/*
 * Handle structure for GPIO Pins
 */
typedef struct{
	GPIO_RegDef_t 		*pGPIOx; //Base address of port to which pin belongs
	GPIO_PinConfig_t	GPIO_PinConfig; //GPIO pin configuration settings
}GPIO_Handle_t;

/*
 *@GPIO_PIN_NUMS
 * GPIOs pin numbers
 */
#define GPIO_PIN_NO_0					0
#define GPIO_PIN_NO_1					1
#define GPIO_PIN_NO_2					2
#define GPIO_PIN_NO_3					3
#define GPIO_PIN_NO_4					4
#define GPIO_PIN_NO_5					5
#define GPIO_PIN_NO_6					6
#define GPIO_PIN_NO_7					7
#define GPIO_PIN_NO_8					8
#define GPIO_PIN_NO_9					9
#define GPIO_PIN_NO_10					10
#define GPIO_PIN_NO_11					11
#define GPIO_PIN_NO_12					12
#define GPIO_PIN_NO_13					13
#define GPIO_PIN_NO_14					14
#define GPIO_PIN_NO_15					15

/*
 *@GPIO_PIN_MODES
 * GPIOs pin modes
 */
#define GPIO_MODE_IN					0
#define GPIO_MODE_OUT					1
#define GPIO_MODE_ALT_FN				2
#define GPIO_MODE_ANALOG				3
#define GPIO_MODE_INTRUPT_FALL			4
#define GPIO_MODE_INTRUPT_RISE			5
#define GPIO_MODE_INTRUPT_RISE_FALL		6

/*
 * GPIO output types
 */
#define GPIO_OP_TYPE_PP					0
#define GPIO_OP_TYPE_OD					1

/*
 * GPIO output speeds
 */
#define GPIO_OP_SPEED_LOW				0
#define GPIO_OP_SPEED_MED				1
#define GPIO_OP_SPEED_HIGH				2

/*
 * GPIO pull up/down configuration
 */
#define GPIO_NO_PUPD					0
#define GPIO_PU							1
#define GPIO_PD							3

/*
 * GPIO alternate function configuration
 */


/* APIs supported by this driver*/
void 		GPIO_PClkCtrl			(GPIO_RegDef_t *pGPIOx, uint8_t E_D); //Enable or Disable a peripheral

void 		GPIO_Init				(GPIO_Handle_t *pGPIOHandle);
void 		GPIO_DeInit				(GPIO_RegDef_t *pGPIOx);

uint8_t 	GPIO_ReadFrmIpPin		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t 	GPIO_ReadFrmIpPort		(GPIO_RegDef_t *pGPIOx);
void 		GPIO_WriteToOpPin		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void 		GPIO_WriteToOpPort		(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void 		GPIO_ToggleOpPort		(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void 		GPIO_IRQConfig			(uint8_t IRQNumber, uint8_t E_D);
void 		GPIO_IRQPriorityConfig	(uint8_t IRQNumber, uint32_t IRQPriority);
void 		GPIO_IRQHandling		(uint8_t PinNumber);


#endif /* INC_STM32F303XCXX_GPIO_DRIVER_H_ */
