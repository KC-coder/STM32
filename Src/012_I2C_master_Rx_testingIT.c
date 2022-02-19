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

//flag to handle flow of program
uint8_t rxComplt = RESET;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


//prefer to declare it has global as these are used in many functions
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

	//I2C IRQ Configuration
	I2C_IRQInterruptConfig(IRQ_N0_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_N0_I2C1_ER, ENABLE);

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

		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR , I2C_ENABLE_SR) != I2C_READY );

		//to get the length of data master wants to send
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &Len, 1, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY );
		printf("length :%d\n", Len);

		//send command to salve , to send data
		commandcode = 0x52;

		//send command
		while(I2C_MasterSendDataIT(&I2C1Handle,&commandcode,1,SLAVE_ADDR , I2C_DISABLE_SR) != I2C_READY );

		//master recives data from slave
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, Len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY );

		rxComplt = RESET;//as it could be set in previous run
		while(rxComplt != SET);
		rcv_buf[Len+1] = '\0';

		printf("Data : %s", rcv_buf);

		rxComplt = RESET;

		while(1);


	}

}

void I2C1_EV_IRQHandler()
{
	/* I2C1 event interrupt */

	I2C_EV_IRQHandling(&I2C1Handle); // & is used as function accepts pointer type of data
									//so we have to pass the address of the variable

}


void I2C1_ER_IRQHandler()
{
	  /* I2C1 error interrupt  */
	I2C_ER_IRQHandling(&I2C1Handle);


}



void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{

	//THIS FUNCTION IS APPLICATION SPECIFIC
     if(AppEv == I2C_EV_TX_CMPLT)
     {
    	 printf("Tx is completed\n");
     }else if (AppEv == I2C_EV_RX_CMPLT)
     {
    	 printf("Rx is completed\n");
    	 rxComplt = SET;
     }else if (AppEv == I2C_ERROR_AF)
     {
    	 printf("Error : Ack failure\n");
    	 //in master ack failure happens when slave fails to send ack for the byte
    	 //sent from the master.
    	 I2C_CloseSendData(pI2CHandle);

    	 //generate the stop condition to release the bus
    	 I2C_GenerateStopCondition(I2C1);

    	 //Hang in infinite loop
    	 while(1);
     }
}
















