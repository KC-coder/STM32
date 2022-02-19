/*
 * ds1307.h
 *
 *  Created on: 11-Aug-2021
 *      Author: kamal chopra
 */
/*
 * it contains device ds1307(RTC) related information
 *
 * 	- I2C address
 * 	- Register addresses
 *	- Data structure to handle the information
 *	- function prototypes which are exposed to application layer
 * 	- application configurable items
 */


#ifndef DS1307_H_
#define DS1307_H_

#include <stdint.h>
#include <string.h>
#include "stm32F407xx_I2C_driver.h"
#include "stm32F407xx_gpio_driver.h"
/* Register address */
#define DS1307_ADDR_SEC				0x00
#define DS1307_ADDR_MIN				0x01
#define DS1307_ADDR_HRS				0x02
#define DS1307_ADDR_DAY				0x03
#define DS1307_ADDR_DATE			0x04
#define DS1307_ADDR_MONTH			0x05
#define DS1307_ADDR_YEAR			0x06

#define DS1307_FORMAT_12HRS_AM		0
#define DS1307_FORMAT_12HRS_PM		1
#define DS1307_FORMAT_24HRS			2

#define DS1307_I2C_ADDRESS			0x68

#define SUNDAY						1
#define MONDAY						2
#define TUESDAY						3
#define WEDNESDAY					4
#define THURSDAY					5
#define FRIDAY						6
#define SATURDAY					7

/* application configurable items */

//if we want to use some other pins then application
//has to make a change here
#define DS1307_I2C					I2C1
#define DS1307_I2C_SDA_PIN			7
#define DS1307_I2C_SCAL_PIN			6
#define DS1307_I2C_GPIO_PORT		GPIOB
#define DS1307_I2C_SPEED			100000
#define DS1307_I2C_PUPD				GPIO_PU //no internal pull ups


typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
}RTC_date_t;

typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;

}RTC_time_t;


//functions

uint8_t DS1307_init(); //to intinalise the chip
void DS1307_set_current_time(RTC_time_t *);//to set the current time value in the register
void DS1307_get_current_time(RTC_time_t *);


void DS1307_set_current_date(RTC_date_t *);//to set the current date value in the register
void DS1307_get_current_date(RTC_time_t *);





#endif /* DS1307_H_ */
