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
uint32_t ADC_average2();
uint32_t ADC_average1();
uint32_t adc1_value;
uint32_t adc2_value;
int main()
{
	printf("hello");

	//uint16_t data ;

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
	ADC_GPIO_INT();
	line1Sensor_int ();

	line2Sensor_int();

	ADC1_ON();
	dealy();
	ADC1_ON();
	dealy();

	ADC2_ON();
	dealy();
	ADC2_ON();
	dealy();
	adc1_value =0;
	adc2_value =0 ;


	uint32_t data1 = ADC_average1();
	uint32_t data2 = ADC_average2();

	while(1)
	{



		printf("ADC1_average %ld\t",adc1_value);
		printf("ADC2_average %ld\n",adc2_value);


		ADC_Reg_Coversion_EnORDi(ADC1, ENABLE);

		ADC_Reg_Coversion_EnORDi(ADC2, ENABLE);



		//while( !(ADC1->SR & (1<<ADC_SR_EOC)) );


		//data = ADC_ReadRegularData(ADC1); //ERROR




		printf("ADC1_data %d\t",ADC_ReadRegularData(ADC1));
		ADC1->SR &= ~(1 << ADC_SR_STRT);
		ADC1->SR &= ~(1 << ADC_SR_EOC); //ERROR
		ADC_Reg_Coversion_EnORDi(ADC1, ENABLE);


		printf("ADC2_data2 %d\n",ADC_ReadRegularData(ADC2));
		ADC2->SR &= ~(1 << ADC_SR_STRT);
		ADC2->SR &= ~(1 << ADC_SR_EOC); //ERROR
		ADC_Reg_Coversion_EnORDi(ADC2, ENABLE);
	}


}
uint32_t ADC_average2()
{
	long adc2[50];
	uint32_t data2;
	uint32_t average_adc2;
	for(uint32_t i =0 ;i<40 ; i++)
	{
			ADC_Reg_Coversion_EnORDi(ADC2, ENABLE);
			data2 = ADC_ReadRegularData(ADC2);


			ADC2->SR &= ~(1 << ADC_SR_STRT);
			ADC2->SR &= ~(1 << ADC_SR_EOC);
			ADC_Reg_Coversion_EnORDi(ADC2, ENABLE);

			adc2[i] = data2;


	}
	average_adc2 =0 ;
	for (uint32_t i = 40 ; i<0 ; i--)
	{
		average_adc2 = average_adc2 + adc2[i];

	}
	average_adc2 = (average_adc2 / 40);

	adc2_value = average_adc2;
	printf("ADC2_average %ld\n",adc2_value);
	return adc2_value;


}
uint32_t ADC_average1()
{
	long adc1[50];
	uint32_t data1;
	uint32_t average_adc1;
	for(uint32_t i =0 ;i<40 ; i++)
	{
			data1 = ADC_ReadRegularData(ADC1);

			ADC1->SR &= ~(1 << ADC_SR_STRT);
			ADC1->SR &= ~(1 << ADC_SR_EOC);
			ADC_Reg_Coversion_EnORDi(ADC1, ENABLE);

			adc1[i] = data1;
	}
	average_adc1 =0 ;

	for (uint32_t i = 40 ; i<0 ; i--)
	{
		average_adc1 = average_adc1	+ adc1[i];


	}
	average_adc1 = (average_adc1 / 40);


	adc1_value =  average_adc1;


	printf("ADC1_average %ld\t",adc1_value);

return adc1_value ;
	}







void ADC_GPIO_INT()
{
	 GPIO_Handle_t GpioARead;

	 GpioARead.pGPIOx = GPIOA;

	 GpioARead.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ANALOG;
	 GpioARead.GPIO_PinConfig.GPIO_PinPuPdControl =GPIO_NO_PUPD;
	 GpioARead.GPIO_PinConfig.GPIO_PinSpeed =  GPIO_SPEED_MEDIUM;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GpioARead.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&GpioARead);

	GpioARead.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&GpioARead);


}

void line1Sensor_int ()
{
	ADC_Handle_t line_Sensor1 ;

	line_Sensor1.ADCx = ADC1 ;
	line_Sensor1.ADC_Config.Resolution = RESOLUTION_10bit;
	line_Sensor1.ADC_Config.Ext_trig_Type = ADC_EXT_TYP_DIS	;
	line_Sensor1.ADC_Config.Ext_trig_Source = ADC_EXT_TIM1_CC1;
	line_Sensor1.ADC_Config.Discontinuous_Mode_EnOrDi = DISABLE;
	line_Sensor1.ADC_Config.overrun_IT = OVERRUN_IT_DI;
	line_Sensor1.ADC_Config.Align_Data = ALIGN_DATA_RIGHT;
	line_Sensor1.ADC_Config.Continous_Mode = ENABLE;

	ADC_PeriClock(ADC1, ENABLE); // ADC Peripheral  clock , to configure ADC registers

	ADC_ComReg->CCR |= (2<<16); //ADC_CommonRegDef_t *adc ;

	ADC_Int(&line_Sensor1); //function to configure ADC registers



	ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_2, 1, CyclePerSample_84);

	//ADC1->SMPR2 = (CyclePerSample_84 << ADC_SMPR2_SMP2);
	//ADC1->SQR3 = (ADC_CHANNEL_2 <<  ADC_SQR3_SQ1);

}



void line2Sensor_int()
{
	ADC_Handle_t line_Sensor2 ;

	line_Sensor2.ADCx = ADC2 ;
	line_Sensor2.ADC_Config.Resolution = RESOLUTION_10bit;
	line_Sensor2.ADC_Config.Ext_trig_Type = ADC_EXT_TYP_DIS	;
	line_Sensor2.ADC_Config.Ext_trig_Source = ADC_EXT_TIM1_CC1;
	line_Sensor2.ADC_Config.Discontinuous_Mode_EnOrDi = DISABLE;
	line_Sensor2.ADC_Config.overrun_IT = OVERRUN_IT_DI;
	line_Sensor2.ADC_Config.Align_Data = ALIGN_DATA_RIGHT;
	line_Sensor2.ADC_Config.Continous_Mode = ENABLE;

	ADC_PeriClock(ADC2, ENABLE); // ADC Peripheral  clock , to configure ADC registers

	ADC_ComReg->CCR |= (2<<16); //ADC_CommonRegDef_t *adc ;

	ADC_Int(&line_Sensor2); //function to configure ADC registers



	ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_3, 1, CyclePerSample_84);

	//ADC1->SMPR2 = (CyclePerSample_84 << ADC_SMPR2_SMP2);
	//ADC1->SQR3 = (ADC_CHANNEL_2 <<  ADC_SQR3_SQ1);

}



void dealy()
{
	for(uint32_t i =0 ; i< 40000 ; i++);

}
