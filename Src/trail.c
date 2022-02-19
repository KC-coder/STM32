/*
 * 022ADC_BASIC.c
 *
 *  Created on: 08-Sep-2020
 *      Author: KAMAL
 */
#include<stdio.h>
#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"
#include"stm32F407xx_ADC_driver.h"

void dealy();

void ADC_GPIO_INT();

void line1Sensor_int ();
void line2Sensor_int();
void line3Sensor_int();

int main()
{
	printf("hello");

	uint16_t data1 ,data2,data3 ;

/*	ADC_Handle_t line_Sensor ;

	line_Sensor.ADCx = ADC1 ;
	line_Sensor.ADC_Config.Resolution = RESOLUTION_8bit;
	line_Sensor.ADC_Config.Ext_trig_Type = ADC_EXT_TYP_DIS	;
	line_Sensor.ADC_Config.Ext_trig_Source = ADC_EXT_TIM1_CC1;
	line_Sensor.ADC_Config.Discontinuous_Mode_EnOrDi = DISABLE;
	line_Sensor.ADC_Config.overrun_IT = OVERRUN_IT_DI;
	line_Sensor.ADC_Config.Align_Data = ALIGN_DATA_RIGHT;

	ADC_PeriClock(ADC1, ENABLE); // ADC Peripheral  clock , to configure ADC registers

	ADC_ComReg->CCR |= (2<<16); //ADC_CommonRegDef_t *adc ;

	ADC_Int(&line_Sensor); //function to configure ADC registers


	ADC_GPIO_INT();

	//ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_2, 2, CyclePerSample_84);

	ADC1->SMPR2 = (CyclePerSample_84 << ADC_SMPR2_SMP2);
	ADC1->SQR3 = (ADC_CHANNEL_2 <<  ADC_SQR3_SQ1);
	ADC1->CR2 |=(1<<ADC_CR2_CONT); //TO enable ADC continous mode
	*/

	line1Sensor_int ();

	line2Sensor_int();
	line3Sensor_int();

	while(1)
	{



		data1 = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_3);
		data2 = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_2);
		data3 = GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_1);



		printf("ADC1_data %d\t",data1);


		printf("ADC2_data2 %d\t",data2);

		printf("ADC2_data3 %d\n",data3);

	}


}

void line1Sensor_int ()
{

	GPIO_Handle_t line1Sensor_int;

	line1Sensor_int.pGPIOx = GPIOA;
	line1Sensor_int.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	line1Sensor_int.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	line1Sensor_int.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	line1Sensor_int.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	line1Sensor_int.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&line1Sensor_int);


}



void line2Sensor_int()
{

	GPIO_Handle_t line2Sensor_int;

	line2Sensor_int.pGPIOx = GPIOA;
	line2Sensor_int.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	line2Sensor_int.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	line2Sensor_int.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	line2Sensor_int.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	line2Sensor_int.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&line2Sensor_int);

}
void line3Sensor_int()
{

	GPIO_Handle_t line3Sensor_int;

	line3Sensor_int.pGPIOx = GPIOA;
	line3Sensor_int.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	line3Sensor_int.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	line3Sensor_int.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	line3Sensor_int.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	line3Sensor_int.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GPIO_Init(&line3Sensor_int);

}





void dealy()
{
	for(uint32_t i =0 ; i< 40000 ; i++);

}
/*
 * trail.c
 *
 *  Created on: 25-Sep-2020
 *      Author: MUKUL
 */


