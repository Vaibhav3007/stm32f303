#include <stdint.h>
#include "stm32f303xCxx.h"
#include "stm32f303xCxx_gpio_driver.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

void delay()
{
	for(uint32_t i = 0; i <= 300000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOE;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOutType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPd = GPIO_NO_PUPD;

	GPIO_PClkCtrl(GPIOE, ENABLE);
	GPIO_Init(&GpioLed);

    /* Loop forever */
	while (1)
	{
		GPIO_ToggleOpPort(GPIOE, GPIO_PIN_NO_14);
		delay();
	}
	return 0;
}
