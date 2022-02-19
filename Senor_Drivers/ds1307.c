/*
 * ds1307.c
 * refer datasheet of RTC ds1307
 *
 *  Created on: 11-Aug-2021
 *      Author: kamal chopra
 */
/*
 * DATA write on the DS1307 chip
 *
 * start->addressPhase{RTC chips address}(R/W` - 0 )->wait for ACK->
 * send address of register we want to write  ->wait for ack ->
 * send the data we want to write in that register->wait for ACK->......-. produce stop
 *
 * DATA read from  the DS1307 chip :
 * start->addressPhase{RTC chips address}(R/W` - 1 )->wait for ACK->
 * (slave will send data from the address where its address pointer is pointing to)
 * so before data read we have to intinalize the address pointer of the slave by wrting the
 * chip with thee register address we want to read
 * So,
 * start->addressPhase{RTC chips address}(R/W` - 0 )->wait for ACK->send address of register
 * we want to read->wait for ack ->generate repeated start ->addressPhase{RTC chips address}(R/W` - 1)
 * -> wait for ACK->start reading data->after every read the address pointer will increment
 * to point to next address
 */


#include "ds1307.h"

I2C_Handle_t ds1307_i2cHandle;

static void DS1307_i2c_pin_config(void);
static void DS1307_i2c_config();
static void DS1307_write(uint8_t value , uint8_t reg_addr);
static uint8_t DS1307_read(uint8_t reg_addr);


//return 1 : CH  = 1 : int failed
//return 0 : CH  = 0   int sucess
uint8_t DS1307_init()
{
	//1. initialize i2c Pins
	DS1307_i2c_pin_config();

	//2. initialize the i2C peripheral
	DS1307_i2c_config();

	//enable the I2C peripheral
	I2C_PeripheralControl(DS1307_I2C, ENABLE);
	//4. TO ENABLE THE INTERNAL oscillator of the chip
	//by making clk halt bit = 0

	DS1307_write(0x00,DS1307_ADDR_SEC);

	//5.read back clock halt bit
	uint8_t clock_state = DS1307_read(DS1307_ADDR_SEC);

	return (clock_state >> 7 ) & 0x1; //as clock state is bit 7 and our read function
						//returns the byte of data so to get only 1 bit we & it by 0x1

}

void DS1307_set_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds , hrs;
	seconds = binary_to_bcd(rtc_time->seconds);
	seconds &=~(1 << 7); //CH bit of the chip, it has to be 0 to enable clk of DS1307
	DS1307_write(seconds, DS1307_ADDR_SEC);

	DS1307_write(binary_to_bcd(rtc_time->minutes), DS1307_ADDR_MIN);

	hrs = binary_to_bcd(rtc_time->hours);

	if(rtc_time->time_format ==  DS1307_FORMAT_24HRS)
	{
		hrs &= ~(1 << 6);
	}else
	{
		hrs |= (1 << 6);

		hrs = (rtc_time->time_format == DS1307_FORMAT_12HRS_PM) ? hrs | (1 << 5) : hrs & ~ (1 << 5)
	}

	DS1307_write(hrs, DS1307_ADDR_HRS);

}


void DS1307_get_current_time(RTC_time_t *rtc_time)
{
	uint8_t seconds , hrs;

	seconds = DS1307_read(DS1307_ADDR_SEC);

	seconds &= ~ (1 << 7); // we have to clear the 7th bit as it does not hold the value of seconds
					// it holds the value of CH bit

	rtc_time->seconds = bcd_to_binary(seconds);
	rtc_time->minutes = bcd_to_binary(DS1307_read(DS1307_ADDR_MIN));

	hrs = DS1307_read(DS1307_ADDR_HRS);
	//checking if formate is 12 or 24hr
	if(hrs & (1 << 6))
	{
		//12hr format
		rtc_time->time_format = !( (hrs & (1 << 5) ) == 0) ;
		hrs &= ~(0x3 << 5);

	}else
	{
		//24hr format
		rtc_time->time_format = DS1307_FORMAT_24HRS	;


	}
	rtc_time->hours = bcd_to_binary(hrs);

}



void DS1307_set_current_date(RTC_date_t *rtc_date)
{
	DS1307_write(rtc_date->day,   DS1307_ADDR_DAY);

	DS1307_write(rtc_date->date,  DS1307_ADDR_DATE);

	DS1307_write(rtc_date->month, DS1307_ADDR_MONTH);

	DS1307_write(rtc_date->year,  DS1307_ADDR_YEAR);
}

void DS1307_get_current_date(RTC_time_t *rtc_date)
{

}

static void DS1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda , i2c_scl;

	memset(&i2c_scl , 0 , sizeof(i2c_scl));
	memset(&i2c_sda , 0 , sizeof(i2c_sda));

	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	i2c_sda.GPIO_PinConfig.GPIO_PinAlFunMode = 4;
	i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClockControl(DS1307_I2C_GPIO_PORT, ENABLE);
	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCAL_PIN	;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFUN;
	i2c_scl.GPIO_PinConfig.GPIO_PinAlFunMode = 4;
	i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD	;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	GPIO_Init(&i2c_scl);





}


static void DS1307_i2c_config(void)
{
	memset(&ds1307_i2cHandle , 0 , sizeof(ds1307_i2cHandle));
	ds1307_i2cHandle.pI2Cx = DS1307_I2C;
	ds1307_i2cHandle.I2C_Config.I2C_AckControl = ENABLE;
	ds1307_i2cHandle.I2C_Config.I2C_FMDutyCycle = DS1307_I2C_SPEED;
	I2C_Init(&ds1307_i2cHandle);
}

static void DS1307_write(uint8_t value , uint8_t reg_addr)
{
	uint8_t tx[2];
	tx[0] = reg_addr;
	tx[1] = value;

	I2C_MasterSendData(&ds1307_i2cHandle, tx, 2, DS1307_I2C_ADDRESS, 0);
}

static uint8_t DS1307_read(uint8_t reg_addr)
{
	uint8_t data;
	I2C_MasterSendData(&ds1307_i2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, 0);
	I2C_MasterReceiveData(&ds1307_i2cHandle, &data, 1,DS1307_I2C_ADDRESS, 0);


	return data;


}

bcd_to_binary
