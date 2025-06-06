/**--------------------------------------------------------------------------
@file	board_blueio_tag_evim.h

@brief	I/O definitions for BLUEIO-TAG_EVIM board

This board uses BLYST Nano module with multiple sensors

@author	Hoang Nguyen Hoan
@date	Mar. 24, 2023

@license

MIT License

Copyright (c) 2023 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_BLUEIO_TAG_EVIM_H__
#define __BOARD_BLUEIO_TAG_EVIM_H__

#include "coredev/iopincfg.h"
#include "miscdev/led.h"

// ***** BLUEIO-TAG-EVIM

#define BLUEIO_TAG_EVIM_BUT1_PORT				0
#define BLUEIO_TAG_EVIM_BUT1_PIN				2
#define BLUEIO_TAG_EVIM_BUT1_PINOP				0

#define BLUEIO_TAG_EVIM_BUT1_SENSE				IOPINSENSE_LOW_TRANSITION

#define BLUEIO_TAG_EVIM_BUT2_PORT				0
#define BLUEIO_TAG_EVIM_BUT2_PIN				13
#define BLUEIO_TAG_EVIM_BUT2_PINOP				0

#define BLUEIO_TAG_EVIM_BUT2_SENSE				IOPINSENSE_LOW_TRANSITION

#define BLUEIO_TAG_EVIM_BUT_PINS_CFG			{ \
	{BLUEIO_TAG_EVIM_BUT1_PORT, BLUEIO_TAG_EVIM_BUT1_PIN, BLUEIO_TAG_EVIM_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BLUEIO_TAG_EVIM_BUT2_PORT, BLUEIO_TAG_EVIM_BUT2_PIN, BLUEIO_TAG_EVIM_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define BLUEIO_TAG_EVIM_LED1_PORT				0
#define BLUEIO_TAG_EVIM_LED1_PIN				30
#define BLUEIO_TAG_EVIM_LED1_PINOP				0
#define BLUEIO_TAG_EVIM_LED1_LOGIC				LED_LOGIC_LOW

#define BLUEIO_TAG_EVIM_LED2R_PORT				0
#define BLUEIO_TAG_EVIM_LED2R_PIN				18
#define BLUEIO_TAG_EVIM_LED2R_PINOP				0
#define BLUEIO_TAG_EVIM_LED2R_LOGIC				LED_LOGIC_HIGH

#define BLUEIO_TAG_EVIM_LED2G_PORT				0
#define BLUEIO_TAG_EVIM_LED2G_PIN				20
#define BLUEIO_TAG_EVIM_LED2G_PINOP				0
#define BLUEIO_TAG_EVIM_LED2G_LOGIC				LED_LOGIC_HIGH

#define BLUEIO_TAG_EVIM_LED2B_PORT				0
#define BLUEIO_TAG_EVIM_LED2B_PIN				19
#define BLUEIO_TAG_EVIM_LED2B_PINOP				0
#define BLUEIO_TAG_EVIM_LED2B_LOGIC				LED_LOGIC_HIGH

#define BLUEIO_TAG_EVIM_LED_PINS_CFG			{ \
	{BLUEIO_TAG_EVIM_LED1_PORT, BLUEIO_TAG_EVIM_LED1_PIN, BLUEIO_TAG_EVIM_LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO_TAG_EVIM_LED2R_PORT, BLUEIO_TAG_EVIM_LED2R_PIN, BLUEIO_TAG_EVIM_LED2R_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO_TAG_EVIM_LED2G_PORT, BLUEIO_TAG_EVIM_LED2G_PIN, BLUEIO_TAG_EVIM_LED2G_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO_TAG_EVIM_LED2B_PORT, BLUEIO_TAG_EVIM_LED2B_PIN, BLUEIO_TAG_EVIM_LED2B_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define BLUEIO_TAG_EVIM_I2C0_SDA_PORT			0
#define BLUEIO_TAG_EVIM_I2C0_SDA_PIN			28
#define BLUEIO_TAG_EVIM_I2C0_SDA_PINOP			1
#define BLUEIO_TAG_EVIM_I2C0_SCL_PORT			0
#define BLUEIO_TAG_EVIM_I2C0_SCL_PIN			29
#define BLUEIO_TAG_EVIM_I2C0_SCL_PINOP			1

#define BLUEIO_TAG_EVIM_I2C0_PINS_CFG	{ \
	{BLUEIO_TAG_EVIM_I2C0_SDA_PORT, BLUEIO_TAG_EVIM_I2C0_SDA_PIN, BLUEIO_TAG_EVIM_I2C0_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO_TAG_EVIM_I2C0_SCL_PORT, BLUEIO_TAG_EVIM_I2C0_SCL_PIN, BLUEIO_TAG_EVIM_I2C0_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, }

#define BLUEIO_TAG_EVIM_SPI2_SCK_PORT			0
#define BLUEIO_TAG_EVIM_SPI2_SCK_PIN			17
#define BLUEIO_TAG_EVIM_SPI2_SCK_PINOP			1
#define BLUEIO_TAG_EVIM_SPI2_MISO_PORT			0
#define BLUEIO_TAG_EVIM_SPI2_MISO_PIN			15
#define BLUEIO_TAG_EVIM_SPI2_MISO_PINOP			1
#define BLUEIO_TAG_EVIM_SPI2_MOSI_PORT			0
#define BLUEIO_TAG_EVIM_SPI2_MOSI_PIN			16
#define BLUEIO_TAG_EVIM_SPI2_MOSI_PINOP			1

#define BLUEIO_TAG_EVIM_IMU_CS_PORT				0
#define BLUEIO_TAG_EVIM_IMU_CS_PIN				5
#define BLUEIO_TAG_EVIM_IMU_CS_PINOP			0

#define BLUEIO_TAG_EVIM_IMU_INT_PORT			0
#define BLUEIO_TAG_EVIM_IMU_INT_PIN				6
#define BLUEIO_TAG_EVIM_IMU_INT_PINOP			0

#define BLUEIO_TAG_EVIM_EEP_WP_PORT				0
#define BLUEIO_TAG_EVIM_EEP_WP_PIN				27
#define BLUEIO_TAG_EVIM_EEP_WP_PINOP			0

#define BLUEIO_TAG_EVIM_BUZZ_PORT				0
#define BLUEIO_TAG_EVIM_BUZZ_PIN				14
#define BLUEIO_TAG_EVIM_BUZZ_PINOP				0

#define BLUEIO_TAG_EVIM_FLASH_CS_PORT			0
#define BLUEIO_TAG_EVIM_FLASH_CS_PIN			26
#define BLUEIO_TAG_EVIM_FLASH_CS_PINOP			0

#define BLUEIO_TAG_EVIM_SPI2_PINS_CFG	{\
	{BLUEIO_TAG_EVIM_SPI2_SCK_PORT, BLUEIO_TAG_EVIM_SPI2_SCK_PIN, BLUEIO_TAG_EVIM_SPI2_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO_TAG_EVIM_SPI2_MISO_PORT, BLUEIO_TAG_EVIM_SPI2_MISO_PIN, BLUEIO_TAG_EVIM_SPI2_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {BLUEIO_TAG_EVIM_SPI2_MOSI_PORT, BLUEIO_TAG_EVIM_SPI2_MOSI_PIN, BLUEIO_TAG_EVIM_SPI2_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {BLUEIO_TAG_EVIM_IMU_CS_PORT, BLUEIO_TAG_EVIM_IMU_CS_PIN, BLUEIO_TAG_EVIM_IMU_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO_TAG_EVIM_FLASH_CS_PORT, BLUEIO_TAG_EVIM_FLASH_CS_PIN, BLUEIO_TAG_EVIM_FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},}	// CS

// UART pins
#define BLUEIO_TAG_EVIM_UART_RX_PORT			0
#define BLUEIO_TAG_EVIM_UART_RX_PIN				8
#define BLUEIO_TAG_EVIM_UART_RX_PINOP			1
#define BLUEIO_TAG_EVIM_UART_TX_PORT			0
#define BLUEIO_TAG_EVIM_UART_TX_PIN				7
#define BLUEIO_TAG_EVIM_UART_TX_PINOP			1

#define BLUEIO_TAG_EVIM_UART_PINS_CFG	{ \
	{BLUEIO_TAG_EVIM_UART_RX_PORT, BLUEIO_TAG_EVIM_UART_RX_PIN, BLUEIO_TAG_EVIM_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLUEIO_TAG_EVIM_UART_TX_PORT, BLUEIO_TAG_EVIM_UART_TX_PIN, BLUEIO_TAG_EVIM_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}}


// BLUEIO-TAG-EVIM *****


#endif // __BOARD_BLUEIO_TAG_EVIM_H__
