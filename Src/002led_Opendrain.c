/*
 * 001led_toggle.c
 *
 *  Created on: 25-Aug-2020
 *      Author: kamal chopra
 */

// toggling on board led_PD12 using open drain output type

#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"

void delay()
{
	for ( uint32_t i = 0 ; i < 500000 ; i++ );
}

int main()
{
	GPIO_Handle_t GpioDLed;

	GpioDLed.pGPIOx = GPIOD;
	GpioDLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioDLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioDLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioDLed.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_NO_PUPD;
	GpioDLed.GPIO_PinConfig.GPIO_PinSpeed =  GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioDLed);

	while(1)
	{

	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
	delay();

	}

	return(0);

}
