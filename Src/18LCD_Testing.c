/*
 * 18LCD_Testing.c
 *
 *  Created on: 12-Aug-2021
 *      Author: kamal chopra
 */

#include<stdio.h>


#include "lcd.h"



/* Enable this macro if you want to test RTC on LCD */
//#define PRINT_LCD




int main(void)
{
	uint8_t data = 9;

	lcd_init();
	lcd_set_cursor(0, 0);
	lcd_print_string("RTC Test...");
	for(uint16_t i = 0 ; i < 6500 ; i ++);
	while(1);

//	mdelay(2000);

	lcd_display_clear();
	lcd_display_return_home();





}



