/*
 * 003LedwithButton.c
 *
 *  Created on: 26-Aug-2020
 *      Author: KAMAL CHOPRA
 *
 *      IN THIS WE HAVE USED ON-BOARD LED AT PD12 AND ON BOARD USER BUTTON
 */

#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"

void delay()
{
	for ( uint32_t i = 0 ; i < 500000/2 ; i++ );
}

int main()
{
	GPIO_Handle_t GpioDLed;

	GpioDLed.pGPIOx = GPIOD;
	GpioDLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioDLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioDLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioDLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Handle_t GpioAbtn;
	GpioAbtn.pGPIOx = GPIOA;
	GpioAbtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioAbtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	// no need of pull up or pull down register as it is internally
	// connected it with button



	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&GpioDLed);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioAbtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0))
		{
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
			delay(); // for button debouncing
		}
	}

}
