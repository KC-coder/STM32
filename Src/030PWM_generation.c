/*
 * 030PWM_generation.c
 *
 *  Created on: 24-Sep-2020
 *      Author: kamal chopra
 *
 */
#include"stm32f407xx.h"
#include "stm32F407xx_gpio_driver.h"
#include"stm32F407xx_ADC_driver.h"
#include"stm32F407xx_TIM_driver.h"
#include<stdio.h>
void TO_GET_PWM();

int main()
{
	TO_GET_PWM();
	PWM_Start(TIM2);
	while(1);

	return 0;

}

void TO_GET_PWM()
{
	TIM_Handle_t PWM1;
	TIM_OC_InitTypeDef PWM1_OC_CONFIG;

	PWM1.pTIMx = TIM2;
	PWM1.TIMConfig.CounterMode = (TIM_COUNTERMODE_UP);
	PWM1.TIMConfig.Prescaler = 64;
	PWM1.TIMConfig.Period = 24615-1;

	PWM1.TIM_ChannelNumber = TIM_OC_Channel_1;
	PWM1_OC_CONFIG.OC_Mode = TIM_OC_MODE_PWM_Mode1;
	PWM1_OC_CONFIG.OC_Polarity = TIM_OC_Polarity_ActiveHigh;
	PWM1_OC_CONFIG.Pulse = 8000;
	TIM_PeriClockControl(TIM2, ENABLE);

	TIM_BASIC_Init(&PWM1);
	TIM_PWM_Init(&PWM1, &PWM1_OC_CONFIG);

	//configuring GPIO pin PA0 to act as tim2 op  channel 1
	GPIO_Handle_t pGPIOA;
	pGPIOA.pGPIOx = GPIOA;
	pGPIOA.GPIO_PinConfig.GPIO_PinMode =GPIO_MODE_ALTFUN;
	pGPIOA.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	pGPIOA.GPIO_PinConfig.GPIO_PinOPType = 0;
	pGPIOA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	pGPIOA.GPIO_PinConfig.GPIO_PinAlFunMode = 1;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&pGPIOA);
}
