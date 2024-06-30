#include <string.h>
#include <stdint.h>
#include "stm32f303xCxx.h"
#include "stm32f303xCxx_gpio_driver.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay()
{
	for(uint32_t i = 0; i <= 300000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, UsrBtn;
	memset(&GpioLed,0,sizeof(GpioLed)); //This ensure all member elements are initialized to zero
	memset(&UsrBtn,0,sizeof(UsrBtn));
	//Configure LED pin
	GpioLed.pGPIOx = GPIOE;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;
	//Enable LED pin
	GPIO_PClkCtrl(GPIOE, ENABLE);
	GPIO_Init(&GpioLed);
	//Configure Button Pin
	UsrBtn.pGPIOx = GPIOA;
	UsrBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	UsrBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INTRUPT_FALL;
	UsrBtn.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;
	//Enable Button Pin
	GPIO_PClkCtrl(GPIOA, ENABLE);
	GPIO_Init(&UsrBtn);
	//Interrupt handling
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0, PRIORITY_1);
	GPIO_IRQConfig(IRQ_NO_EXTI0, ENABLE);
    /* Loop forever */
	while(1);
}

void EXTI0_IRQHandler(void)
{
//	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOpPin(GPIOE, GPIO_PIN_NO_14);
}
