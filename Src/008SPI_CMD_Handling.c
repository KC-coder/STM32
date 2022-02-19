 /*
 * 008SPI_CMD_Handling.c
 *
 *  Created on: 15-Sep-2020
 *      Author: KAMAL CHOPRA
 */


/*
 * 006SPI_Tx_Tesing.c
 *
 *  Created on: 02-Sep-2020
 *      Author: kamal chopra
 */
#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"
#include"stm32F407xx_SPI_driver.h"
#include<string.h>
#include<stdio.h>
/* FOR SPI2_
 *PB14 : MISO
 *PB15 : MOSI
 *PB13 : SCLK
 *PB12 : NSS
 * ALT Function mode : 5 ( information from data sheet of MCU )
 */

/*
 * in this we send data to external world i.e only transmit the data
 *
 */

//Some macros related to this macros
#define COMMAND_LED_CTRL			0x50
#define COMMAND_SENSOR_READ			0x51
#define COMMAND_LED_READ			0x52
#define COMMAND_PRINT				0x53
#define COMMAND_ID_READ				0x54

#define LED_ON 						1
#define LED_OFF						0

#define ANALOG_PIN0					0
#define ANALOG_PIN1					1
#define ANALOG_PIN2					2
#define ANALOG_PIN3					3
#define ANALOG_PIN4					4
#define ANALOG_PIN5					5

#define LED_PIN 					9


uint8_t SPI_VerifyResponse(uint8_t ackbyte);

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t pSPIpins;

	pSPIpins.pGPIOx = GPIOB;
	pSPIpins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	pSPIpins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP; // open drain output type is required in I2C
	pSPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	pSPIpins.GPIO_PinConfig.GPIO_PinAlFunMode = 5;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	//MISO
	pSPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&pSPIpins);

	//MOSI
	pSPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&pSPIpins);

	//SCLK
	pSPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&pSPIpins);

	//NSS
	pSPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&pSPIpins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;//generate clk of 8MHz, as we using 19MHz RC osscilator
	SPI2Handle.SPIConfig.SPI_CPAH = SPI_CPAH_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // FOR NSS PIN

	SPI_PeriClockControl(SPI2, ENABLE);
	SPI_Init(&SPI2Handle);

}

void GPIO_ButtonInit()
{
	GPIO_Handle_t GpioAbtn;
	GpioAbtn.pGPIOx = GPIOA;
	GpioAbtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GpioAbtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioAbtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioAbtn);


}

void delay()
{
	for ( uint32_t i = 0 ; i < 500000 ; i++ );
}


int main()
{

	uint8_t dummy_write	= 0xFF;
	uint8_t dummy_read;

	//this function to intinalize gpio pins to be used for spi
	SPI2_GPIOInits();

	GPIO_ButtonInit();

	//this is to configure the spi peripherals
	SPI2_Inits();

	SPI_SSOEConfig(SPI2, ENABLE);



	//this makes the NSS high internally and avoid MODEF error
	//SPI_SSIConfig(SPI2, ENABLE); it is needed when software slave management is enabled
	while(1)
	{

		while( !(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) ) ) ;

		delay();//to avoid button denouncing


		//enable the SPI2 peripheral after configuring
		SPI_PeripheralControl(SPI2, ENABLE);

		//Send Command 1. CMD_LED_CTRL	<pin no(1)>		<value(1)>
		uint8_t commandcode	= COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		SPI_SendData(SPI2, &commandcode, 1);

		//do dummy read clear from RXNE (as when data is send it also revice the data in return
		SPI_ReceiveData(SPI2,&dummy_read , 1);

		//send some dummy bits (1byte) to fetch  response from the slave
		SPI_SendData(SPI2, &dummy_write,1);


		//read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte, 1);


		if( SPI_VerifyResponse(ackbyte) )
		{
			//send other commands
			args[0] = LED_PIN;
			args[1] = LED_ON;

			SPI_SendData(SPI2, args ,2);

		}
		//2. CMD_SENOSR_READ   <analog pin number(1) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2,&analog_read,1);
			printf("COMMAND_SENSOR_READ %d\n",analog_read);
		}

		//3.  CMD_LED_READ 	 <pin no(1) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_LED_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;

			//send arguments
			SPI_SendData(SPI2,args,1); //sending one byte of

			//do dummy read to clear off the RXNE
			SPI_ReceiveData(SPI2,&dummy_read,1);

			//insert some delay so that slave can ready with the data
			delay();

			//Send some dummy bits (1 byte) fetch the response from the slave
			SPI_SendData(SPI2,&dummy_write,1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2,&led_status,1);
			printf("COMMAND_READ_LED %d\n",led_status);

		}

		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_PRINT;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t message[] = "Hello ! How are you ??";
		if( SPI_VerifyResponse(ackbyte))
		{
			args[0] = strlen((char*)message);

			//send arguments
			SPI_SendData(SPI2,args,1); //sending length

			//send message
			SPI_SendData(SPI2,message,args[0]);

			printf("COMMAND_PRINT Executed \n");

		}

		//5. CMD_ID_READ
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = COMMAND_ID_READ;

		//send command
		SPI_SendData(SPI2,&commandcode,1);

		//do dummy read to clear off the RXNE
		SPI_ReceiveData(SPI2,&dummy_read,1);

		//Send some dummy byte to fetch the response from the slave
		SPI_SendData(SPI2,&dummy_write,1);

		//read the ack byte received
		SPI_ReceiveData(SPI2,&ackbyte,1);

		uint8_t id[11];
		uint32_t i=0;
		if( SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[11] = '\0';

			printf("COMMAND_ID : %s \n",id);

		}




		//lets confirm SPI is not busy
		while( SPI_GetFlagStatus(SPI2,SPI_BSY_FLAG) );

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("SPI Communication Closed\n");
	}


	return 0;
}


uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == (uint8_t)0xF5)
	{
		//if it is ack
		return 1;
	}

	return 0;
}
