/*
 * 010I2C_master_tx_testing.c
 *
 *  Created on: 07-Nov-2020
 *      Author: kamal chopra
 */

/*
 * IN THIS DATA IS SEND TO THE ARDUNIO
 * STM32 act as master
 * ardunio sketch - 001I2CSlaveRxString.ino
 * ardunio pins A4 - PB7
 * 				A5 - PB6
 */

#include<stdio.h>
#include<string.h>
#include "stm32f407xx.h"
#include "stm32F407xx_gpio_driver.h"
#include "stm32F407xx_I2C_driver.h"



#define MY_ADDR 0x61;

#define SLAVE_ADDR  0x70

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

I2C_Handle_t I2C1Handle;

//receve buffer
uint8_t rcv_buf[32] ;
/*
 * PB6-> SCL
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAlFunMode = 4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.slaveADDRmode = I2C_SLAVE_ADDR_MODE_7BITS;

	I2C_PeriClockControl(I2C1, ENABLE);
	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);

}


int main(void)
{
	uint8_t  commandcode;
	uint8_t Len; //in this length value from ardunio is stored
 	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();
	//enable the i2c peripheral
//	I2C_PeripheralControl(I2C1,ENABLE);



	printf("Application is running\n");


	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made one here
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);



	while(1)
	{
		//wait till button is pressed
		while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandcode = 0x51;

		//send command
		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR , I2C_DISABLE_SR);

		//to get the length of data master wants to send
		I2C_MasterReceiveData(&I2C1Handle, &Len, 1, SLAVE_ADDR, I2C_DISABLE_SR);
		printf("length :%d\n", Len);

		//send command to salve , to send data
		commandcode = 0x52;

		//send command
		I2C_MasterSendData(&I2C1Handle,&commandcode,1,SLAVE_ADDR , I2C_DISABLE_SR);

		//master recives data from slave
		I2C_MasterReceiveData(&I2C1Handle, rcv_buf, Len, SLAVE_ADDR, I2C_DISABLE_SR);

		rcv_buf[Len+1] = '\0';

		printf("Data : %s", rcv_buf);




	}

}
