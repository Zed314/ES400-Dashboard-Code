/*
 * lcd.cpp
 *
 *  Created on: 31 juil. 2021
 *      Author: Zed
 */
#include "tm1620.hpp"

#include "stm32f0xx_hal.h"

#define TM1620_CMD_DATA_AUTO 0x40
#define TM1620_CMD_DATA_READ 0x42 // command to read data used on two wire interfaces of TM1637
#define TM1620_CMD_DATA_FIXED 0x44
#define TM1620_CMD_DISPLAY 0x80
#define TM1620_CMD_ADDRESS 0xC0

const char TM1620::conv_seven[] = {0x7E, 48, 109, 121, 51, 91, 95, 112, 127, 123};

TM1620::TM1620(char din, char clk, char stb)
{
    this->din = din;
    this->clk = clk;
    this->stb = stb;
}

TM1620::~TM1620()
{
}

void TM1620::start_display(void)
{
    HAL_GPIO_WritePin(GPIOB, this->clk | this->stb, GPIO_PIN_SET);
    this->sendCommand(0x01);
    this->setupDisplay();
    this->clearDisplay();
    // Because 5 "digits"
    this->sendData(2, 0x01);
}

void TM1620::convertToSeven(int number, char *out)
{
    *out = *out & 0x80;
    *out = *out | TM1620::conv_seven[number];
}

void TM1620::send_cmd(char data)
{
    for (int i = 0; i < 8; i++)
    {
        HAL_GPIO_WritePin(GPIOB, this->clk, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, this->din, data & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data >>= 1;
        HAL_GPIO_WritePin(GPIOB, this->clk, GPIO_PIN_SET);
    }
}
void TM1620::start_cmd(void)
{
    HAL_GPIO_WritePin(GPIOB, this->stb, GPIO_PIN_RESET);
}

void TM1620::stop_cmd(void)
{
    HAL_GPIO_WritePin(GPIOB, this->stb, GPIO_PIN_SET);
}

void TM1620::sendCommand(char cmd)
{
    this->start_cmd();
    this->send_cmd(cmd);
    this->stop_cmd();
}

void TM1620::sendData(char address, char data)
{
    sendCommand(TM1620_CMD_DATA_FIXED); // use fixed addressing for data
    start_cmd();
    send_cmd(TM1620_CMD_ADDRESS | address); // address command + address
    send_cmd(data);
    stop_cmd();
}

void TM1620::clearDisplay()
{
    for (char nPos = 0; nPos < 10; nPos++)
    {
        // all OFF
        sendData(nPos << 1, 0);
        sendData((nPos << 1) | 1, 0);
        // all ON
        //sendData(nPos << 1, 0b11111111);
        //sendData((nPos << 1) | 1, 0b00000011);
    }
}

void TM1620::setupDisplay()
{
    sendCommand(TM1620_CMD_DISPLAY | 8 | 7);
}
