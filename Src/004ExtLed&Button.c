/*
 * 004ExtLed&Button.c
 *
 *  Created on: 28-Aug-2020
 *      Author: MUKUL
 */
#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"

void delay()
{
	for(uint32_t i ; i <= (500000) ; i++);
}

int main()
{
	GPIO_Handle_t GpioALed;
	GpioALed.pGPIOx = GPIOA;
	GpioALed.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_OUT;
	GpioALed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GpioALed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;


	GPIO_Handle_t Gpiobbut;
	Gpiobbut.pGPIOx = GPIOB;
	Gpiobbut.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_IN;
	Gpiobbut.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	//Gpiobbut.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&Gpiobbut);
	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioALed);

	Gpiobbut.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	Gpiobbut.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;


	while(1)
	{
		/*if( GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_12) == 1)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_8 );*/

			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET	);
			delay();
			GPIO_WriteToOutputPin(GPIOB, GPIO_PIN_14,GPIO_PIN_RESET);
			delay();



		//}
	}


}

