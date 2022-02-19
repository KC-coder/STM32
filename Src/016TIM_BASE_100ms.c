/*
 * 016TIM_BASE_100ms.c
 *
 *  Created on: 06-Sep-2020
 *      Author: MUKUL
 */

#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"
#include"stm32F407xx_TIM_driver.h"

void GPIO_PIN_ready();

int main()
{
	TIM_Handle_t TIM6_delay;

	TIM6_delay.pTIMx = TIM6;
	TIM6_delay.TIMConfig.Prescaler = 24;
	TIM6_delay.TIMConfig.Period = 64000 - 1;

	TIM_PeriClockControl(TIM6, ENABLE);
	TIM_BASIC_Init(&TIM6_delay);
	//TIM_IRQInterruptConfig(IRQ_N0_TIM6_DAC, ENABLE);
	//TIM_IRQPriorityConfig(IRQ_N0_TIM6_DAC, NVIC_IRQ_PRI15);

	TIM_BASIC_ENABLE(TIM6);
	GPIO_PIN_ready();

	while(1)
	{
		while(!(TIM6->SR & (1<<0) ));
		TIM6->SR = 0;
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);


	}

}

void GPIO_PIN_ready()
{
	GPIO_Handle_t GpioDLed;

	GpioDLed.pGPIOx = GPIOD;
	GpioDLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioDLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioDLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioDLed.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_NO_PUPD;
	GpioDLed.GPIO_PinConfig.GPIO_PinSpeed =  GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioDLed);
}
