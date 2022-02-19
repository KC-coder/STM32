/*
 * 005Button_interrupt.c
 *
 *  Created on: 31-Aug-2020
 *      Author: MUKUL
 */
#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"
#include<string.h>
void delay()
{
	for(uint32_t i ; i <= (500000) ; i++);
}

int main()
{
	GPIO_Handle_t gpioEled , gpioEbutton ;
	memset(&gpioEled,0,sizeof(gpioEled));
	memset(&gpioEbutton,0,sizeof(gpioEbutton));

	 gpioEled.pGPIOx = GPIOE;
	 gpioEled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	 gpioEled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	 gpioEled.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	 gpioEled.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	 gpioEbutton.pGPIOx = GPIOE;
	 gpioEbutton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	 gpioEbutton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	 gpioEbutton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	 GPIO_PeriClockControl(GPIOE, ENABLE);
	 GPIO_Init(&gpioEled);
	 GPIO_Init(&gpioEbutton);

	 //IRQ Configuration of this pin
	 GPIO_IRQPriorityConfig(IRQ_N0_EXTI15_10, NVIC_IRQ_PRI15);
	 GPIO_IRQInterruptConfig(IRQ_N0_EXTI15_10, ENABLE);


	 while(1);

}

void EXTI15_10_IRQHandler (void) //void as IRQ does't take or return any value
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_12);
	GPIO_ToggleOutputPin(GPIOE, GPIO_PIN_10);
}
