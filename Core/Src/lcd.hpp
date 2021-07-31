/*
 * lcd.hpp
 *
 *  Created on: 31 juil. 2021
 *      Author: Zed
 */

#ifndef SRC_LCD_HPP_
#define SRC_LCD_HPP_

#include "tm1620.hpp"

class LCD
{
public:
    typedef struct
    {
        bool B_kmh;
        bool B_mph;
        bool B_bluetooth;
        bool B_left;
        bool B_right;
        bool B_light;
        bool B_reserved;
        bool B_maint_red;
        bool B_maint_green;
        char battery;
        float value;
    } lcd_t;
    LCD();
    ~LCD();
    void start_display(void);
    void displayLcd(LCD::lcd_t lcd_ext);

private:
    typedef struct
    {

        union
        {
            struct
            {
                char B_maint_green : 1;
                char B_maint_red : 1;
                char B_reserved : 1;
                char B_bat_3 : 1;
                char B_bat_2 : 1;
                char B_bat_1 : 1;
                char B_bat_0 : 1;
                char B_padding : 1;
            } bits;
            char word;

        } bottom;

        union
        {
            struct
            {
                char B_kmh : 1;
                char B_mph : 1;
                char B_dot : 1;
                char B_right : 1;
                char B_light : 1;
                char B_bluetooth : 1;
                char B_left : 1;
                char B_padding : 1;
            } bits;
            char word;

        } up;

    } info_led_t;

    /**
     * Converts number into seven segment, and store it into 7 LSB of out.
     *
     * \param[in] number number to be converted [0,9]
     * \param[out] out converted number
     */
    void convertToSeven(int number, char *out);

    /**
     * Proceeds to conversion of external structure (visible to user),
     * into a format that can be easely displayed, into this->led_array.
     *
     * \param[in] in incoming lcd_t data
     */
    void convertToInternal(LCD::lcd_t in);



private:
    char led_array[5];

    // 5 : DIN
    // 6 : CLK
    // 7 : STB
    TM1620 led_driver;

};


#endif /* SRC_LCD_HPP_ */
