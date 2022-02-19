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
	pSPIpins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;//generate clk of 8MHz, as we using 16MHz RC osscilator
	SPI2Handle.SPIConfig.SPI_CPAH = SPI_CPAH_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_HIGH;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // FOR NSS PIN

	SPI_PeriClockControl(SPI2, ENABLE);
	SPI_Init(&SPI2Handle);

}


int main()
{
	char user_Data[] = "kamal chopra ";

	//this function to intinalize gpio pins to be used for spi
	SPI2_GPIOInits();

	//this is to configure the spi peripherals
	SPI2_Inits();

	//this makes the NSS high internally and avoid MODEF error
	SPI_SSIConfig(SPI2, ENABLE);

	//enable the SPI2 peripheral after configuring

	SPI_PeripheralControl(SPI2, ENABLE);

	//here, (uint8_t*)user_Data we have type casted it as user_Data was of char and we had to send
	//data of type -  uint8_t pointer , by type casting we have changed it from char to uint8_t pointer type
	SPI_SendData(SPI2, (uint8_t*)user_Data, strlen(user_Data));

	//to make program wait until last byte is transfered, i.e wait till SPI is not busy
	while ( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

	SPI_PeripheralControl(SPI2,DISABLE);
	SPI_SSIConfig(SPI2, DISABLE);

	while(1);


	return 0;
}
