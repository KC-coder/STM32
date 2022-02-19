/*
 * 007SPI_Txonly_ardunio.c
 *
 *  Created on: 14-Sep-2020
 *      Author: kamal chopra
 */

/*
 * 006SPI_Tx_Tesing.c
 *
 *  Created on: 02-Sep-2020
 *      Author: kamal chopra
 *      use ardunio file : 006spi_tx_testing.c
 */
#include"stm32f407xx.h"
#include"stm32F407xx_gpio_driver.h"
#include"stm32F407xx_SPI_driver.h"
#include<string.h>

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
	//pSPIpins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&pSPIpins);

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
	for ( uint32_t i = 0 ; i < 500000/2 ; i++ );
}


int main()
{
	char user_Data[] = "kamal chopra ";

	//this function to intinalize gpio pins to be used for spi
	SPI2_GPIOInits();

	GPIO_ButtonInit();

	//this is to configure the spi peripherals
	SPI2_Inits();

	/*
	 * making SSOE = 1 enable the NSS output
	 * the NSS pin is managed by the hardware
	 * i.e when SPE = 1 the NSS is pulled to low
	 * when SPE = 0 the NSS is pulled to high
	 */
	SPI_SSOEConfig(SPI2, ENABLE);



	//this makes the NSS high internally and avoid MODEF error
	//SPI_SSIConfig(SPI2, ENABLE); it is needed when software slave management is enabled
	while(1)
	{
		//when button is not pressed pin reads 0 , NOT of 0 is 1 so loops hangs here
		//when we press button pin reads 1 , NOT of 1 is 0, So loops exit the code
		while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)  ) ;

		//for button de bouncing
		delay();//to avoid button denouncing


		//enable the SPI2 peripheral after configuring
		SPI_PeripheralControl(SPI2, ENABLE);

		//first send the length information
		uint8_t dataLen = strlen(user_Data);
		//here we have't wrtten it as (uint8_t*)dataLen as ,
		//dataLen was already uint8_t from so no need to typecast

		SPI_SendData(SPI2, &dataLen ,1);

		SPI_SendData( SPI2, (uint8_t*)user_Data, strlen(user_Data));

		//to make program wait until last byte is transfered, i.e wait till SPI is not busy
		while ( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		//SPI_SSIConfig(SPI2, DISABLE);
		SPI_PeripheralControl(SPI2,DISABLE);

	}


	return 0;
}

