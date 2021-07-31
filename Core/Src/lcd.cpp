/*
 * lcd.cpp
 *
 *  Created on: 31 juil. 2021
 *      Author: Zed
 */
#include "stm32f0xx_hal.h"
#include "lcd.hpp"
#include "tm1620.hpp"

LCD::LCD():led_driver(5,6,7)
{
    for (int i = 0; i < 5; i++)
    {
        this->led_array[i] = 0;
    }
}

void LCD::start_display(void)
{
	this->led_driver.start_display();
}


void LCD::convertToInternal(LCD::lcd_t in)
{

    int tenth = ((int)in.value / 10) % 10;
    this->led_driver.convertToSeven(tenth, &(this->led_array[4]));
    int unit = ((int)in.value) % 10;
    this->led_driver.convertToSeven(unit, &(this->led_array[3]));
    int dec = ((int)(in.value * 10)) % 10;
    this->led_driver.convertToSeven(dec, &(this->led_array[2]));
    LCD::info_led_t info_led;
    info_led.bottom.bits.B_maint_green = in.B_maint_green;
    info_led.bottom.bits.B_maint_red = in.B_maint_red;
    info_led.bottom.bits.B_reserved = in.B_reserved;
    info_led.bottom.bits.B_bat_0 = in.battery >= 1;
    info_led.bottom.bits.B_bat_1 = in.battery >= 2;
    info_led.bottom.bits.B_bat_2 = in.battery >= 3;
    info_led.bottom.bits.B_bat_3 = in.battery >= 4;

    this->led_array[0] = info_led.bottom.word;

    info_led.up.bits.B_kmh = in.B_kmh;
    info_led.up.bits.B_mph = in.B_mph;

    info_led.up.bits.B_dot = true;

    info_led.up.bits.B_right = in.B_right;
    info_led.up.bits.B_light = in.B_light;
    info_led.up.bits.B_left = in.B_left;
    info_led.up.bits.B_bluetooth = in.B_bluetooth;

    this->led_array[1] = info_led.up.word;
}

void LCD::displayLcd(LCD::lcd_t lcd_ext)
{
    convertToInternal(lcd_ext);
    for (int i = 0; i < 5; i += 1)
    {
        this->led_driver.sendData(i * 2, this->led_array[i]);
    }
}
