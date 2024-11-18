/**--------------------------------------------------------------------------
@file	blystnano_boards.h

@brief	Contains definitions for BLYST Nano (IMM-NRF52832-NANO) based boards

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

#ifndef __BLYSTNANO_BOARDS_H__
#define __BLYSTNANO_BOARDS_H__

#include "coredev/iopincfg.h"
#include "miscdev/led.h"

#define IBK_BLUEIO							1
#define IBK_BLYST_NANO						2
#define BLUEIO_TAG_EVIM						3
#define BLUEIO_ADXL_WCHRG					4
#define BLYST_NANO_DK						5
#define BLUEIO832_MINI						6
#define BLUEPYRO_M3225						7
#define BLUEPYRO_CR24S_M3225				8
#define BLYST_MOTION						9

#define BLYSTNANO_PULSE_TRAIN_PINS	{ \
	{0, 2, 0}, {0, 3, 0}, {0, 4, 0}, {0, 5, 0}, {0, 6, 0}, {0, 7, 0}, {0, 8, 0}, \
	{0, 9, 0}, {0, 10, 0}, {0, 11, 0}, {0, 12, 0}, {0, 13, 0}, {0, 14, 0}, {0, 15, 0}, \
	{0, 16, 0}, {0, 17, 0}, {0, 18, 0}, {0, 19, 0}, {0, 20, 0}, {0, 21, 0}, {0, 22, 0}, \
	{0, 23, 0}, {0, 24, 0}, {0, 25, 0}, {0, 26, 0}, {0, 27, 0}, {0, 28, 0}, {0, 29, 0}, \
	{0, 30, 0}, {0, 31, 0} \
}

// ***** IBK-BLUEIO

// Button
#define IBK_BLUEIO_BUT1_PORT					0
#define IBK_BLUEIO_BUT1_PIN						2
#define IBK_BLUEIO_BUT1_PINOP					0

#define IBK_BLUEIO_BUT1_SENSE					IOPINSENSE_LOW_TRANSITION

#define IBK_BLUEIO_BUT2_PORT					0
#define IBK_BLUEIO_BUT2_PIN						13
#define IBK_BLUEIO_BUT2_PINOP					0

#define IBK_BLUEIO_BUT2_SENSE					IOPINSENSE_LOW_TRANSITION

#define IBK_BLUEIO_BUT_PINS_CFG			{ \
	{IBK_BLUEIO_BUT1_PORT, IBK_BLUEIO_BUT1_PIN, IBK_BLUEIO_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{IBK_BLUEIO_BUT2_PORT, IBK_BLUEIO_BUT2_PIN, IBK_BLUEIO_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// LED
#define IBK_BLUEIO_LED1_PORT					0
#define IBK_BLUEIO_LED1_PIN						30
#define IBK_BLUEIO_LED1_PINOP					0
#define IBK_BLUEIO_LED1_ACTIVE					LED_LOGIC_LOW

#define IBK_BLUEIO_LED2_PORT					0
#define IBK_BLUEIO_LED2_PIN						29
#define IBK_BLUEIO_LED2_PINOP					0
#define IBK_BLUEIO_LED2_ACTIVE					LED_LOGIC_LOW

#define IBK_BLUEIO_LED3_PORT					0
#define IBK_BLUEIO_LED3_PIN						28
#define IBK_BLUEIO_LED3_PINOP					0
#define IBK_BLUEIO_LED3_ACTIVE					LED_LOGIC_LOW

#define IBK_BLUEIO_LED_PINS_CFG		{ \
	{IBK_BLUEIO_LED1_PORT, IBK_BLUEIO_LED1_PIN, IBK_BLUEIO_LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IBK_BLUEIO_LED2_PORT, IBK_BLUEIO_LED2_PIN, IBK_BLUEIO_LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IBK_BLUEIO_LED3_PORT, IBK_BLUEIO_LED3_PIN, IBK_BLUEIO_LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// UART pins
#define IBK_BLUEIO_UART_RX_PORT					0
#define IBK_BLUEIO_UART_RX_PIN					8
#define IBK_BLUEIO_UART_RX_PINOP				1
#define IBK_BLUEIO_UART_TX_PORT					0
#define IBK_BLUEIO_UART_TX_PIN					7
#define IBK_BLUEIO_UART_TX_PINOP				1
#define IBK_BLUEIO_UART_CTS_PORT				0
#define IBK_BLUEIO_UART_CTS_PIN					12
#define IBK_BLUEIO_UART_CTS_PINOP				1
#define IBK_BLUEIO_UART_RTS_PORT				0
#define IBK_BLUEIO_UART_RTS_PIN					11
#define IBK_BLUEIO_UART_RTS_PINOP				1

#define IBK_BLUEIO_UART_PINS_CFG	{ \
	{IBK_BLUEIO_UART_RX_PORT, IBK_BLUEIO_UART_RX_PIN, IBK_BLUEIO_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{IBK_BLUEIO_UART_TX_PORT, IBK_BLUEIO_UART_TX_PIN, IBK_BLUEIO_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{IBK_BLUEIO_UART_CTS_PORT, IBK_BLUEIO_UART_CTS_PIN, IBK_BLUEIO_UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{IBK_BLUEIO_UART_RTS_PORT, IBK_BLUEIO_UART_RTS_PIN, IBK_BLUEIO_UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

// IBK-BLUEIO *****

// ***** IBK-BLYST-NANO

// Button
#define IBK_BLYST_NANO_BUT1_PORT				0
#define IBK_BLYST_NANO_BUT1_PIN					2
#define IBK_BLYST_NANO_BUT1_PINOP				0

#define IBK_BLYST_NANO_BUT1_SENSE				IOPINSENSE_LOW_TRANSITION

#define IBK_BLYST_NANO_BUT2_PORT				0
#define IBK_BLYST_NANO_BUT2_PIN					13
#define IBK_BLYST_NANO_BUT2_PINOP				0

#define IBK_BLYST_NANO_BUT2_SENSE				IOPINSENSE_LOW_TRANSITION

#define IBK_BLYST_NANO_BUT_PINS_CFG			{ \
	{IBK_BLYST_NANO_BUT1_PORT, IBK_BLYST_NANO_BUT1_PIN, IBK_BLYST_NANO_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{IBK_BLYST_NANO_BUT2_PORT, IBK_BLYST_NANO_BUT2_PIN, IBK_BLYST_NANO_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// LED
#define IBK_BLYST_NANO_LED1_PORT				0
#define IBK_BLYST_NANO_LED1_PIN					30
#define IBK_BLYST_NANO_LED1_PINOP				0
#define IBK_BLYST_NANO_LED1_ACTIVE				LED_LOGIC_LOW


#define IBK_BLYST_NANO_LED2_PORT				0
#define IBK_BLYST_NANO_LED2_PIN					29
#define IBK_BLYST_NANO_LED2_PINOP				0
#define IBK_BLYST_NANO_LED2_ACTIVE				LED_LOGIC_LOW

#define IBK_BLYST_NANO_LED3_PORT				0
#define IBK_BLYST_NANO_LED3_PIN					28
#define IBK_BLYST_NANO_LED3_PINOP				0
#define IBK_BLYST_NANO_LED3_ACTIVE				LED_LOGIC_LOW

#define IBK_BLYST_NANO_LED_PINS_CFG		{ \
	{IBK_BLYST_NANO_LED1_PORT, IBK_BLYST_NANO_LED1_PIN, IBK_BLYST_NANO_LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IBK_BLYST_NANO_LED2_PORT, IBK_BLYST_NANO_LED2_PIN, IBK_BLYST_NANO_LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IBK_BLYST_NANO_LED3_PORT, IBK_BLYST_NANO_LED3_PIN, IBK_BLYST_NANO_LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// UART pins
#define IBK_BLYST_NANO_UART_RX_PORT				0
#define IBK_BLYST_NANO_UART_RX_PIN				8
#define IBK_BLYST_NANO_UART_RX_PINOP			0
#define IBK_BLYST_NANO_UART_TX_PORT				0
#define IBK_BLYST_NANO_UART_TX_PIN				7
#define IBK_BLYST_NANO_UART_TX_PINOP			0
#define IBK_BLYST_NANO_UART_CTS_PORT			0
#define IBK_BLYST_NANO_UART_CTS_PIN				12
#define IBK_BLYST_NANO_UART_CTS_PINOP			0
#define IBK_BLYST_NANO_UART_RTS_PORT			0
#define IBK_BLYST_NANO_UART_RTS_PIN				11
#define IBK_BLYST_NANO_UART_RTS_PINOP			0

#define IBK_BLYST_NANO_UART_PINS_CFG	{ \
	{IBK_BLYST_NANO_UART_RX_PORT, IBK_BLYST_NANO_UART_RX_PIN, IBK_BLYST_NANO_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{IBK_BLYST_NANO_UART_TX_PORT, IBK_BLYST_NANO_UART_TX_PIN, IBK_BLYST_NANO_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{IBK_BLYST_NANO_UART_CTS_PORT, IBK_BLYST_NANO_UART_CTS_PIN, IBK_BLYST_NANO_UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{IBK_BLYST_NANO_UART_RTS_PORT, IBK_BLYST_NANO_UART_RTS_PIN, IBK_BLYST_NANO_UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

// IBK-BLYST-NANO *****

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
#define BLUEIO_TAG_EVIM_LED1_ACTIVE				LED_LOGIC_LOW

#define BLUEIO_TAG_EVIM_LED2R_PORT				0
#define BLUEIO_TAG_EVIM_LED2R_PIN				18
#define BLUEIO_TAG_EVIM_LED2R_PINOP				0
#define BLUEIO_TAG_EVIM_LED2R_ACTIVE			LED_LOGIC_HIGH

#define BLUEIO_TAG_EVIM_LED2G_PORT				0
#define BLUEIO_TAG_EVIM_LED2G_PIN				20
#define BLUEIO_TAG_EVIM_LED2G_PINOP				0
#define BLUEIO_TAG_EVIM_LED2G_ACTIVE			LED_LOGIC_HIGH

#define BLUEIO_TAG_EVIM_LED2B_PORT				0
#define BLUEIO_TAG_EVIM_LED2B_PIN				19
#define BLUEIO_TAG_EVIM_LED2B_PINOP				0
#define BLUEIO_TAG_EVIM_LED2B_ACTIVE			LED_LOGIC_HIGH

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

// BLUEIO-TAG-EVIM *****

// ***** BLUEIO-ADXL-WCHRG

#define BLUEIO_ADXL_WCHRG_ADXL362_SCK_PORT		0
#define BLUEIO_ADXL_WCHRG_ADXL362_SCK_PIN		17
#define BLUEIO_ADXL_WCHRG_ADXL362_SCK_PINOP		1
#define BLUEIO_ADXL_WCHRG_ADXL362_MOSI_PORT		0
#define BLUEIO_ADXL_WCHRG_ADXL362_MOSI_PIN		16
#define BLUEIO_ADXL_WCHRG_ADXL362_MOSI_PINOP	1
#define BLUEIO_ADXL_WCHRG_ADXL362_MISO_PORT		0
#define BLUEIO_ADXL_WCHRG_ADXL362_MISO_PIN		15
#define BLUEIO_ADXL_WCHRG_ADXL362_MISO_PINOP	1

#define BLUEIO_ADXL_WCHRG_ADXL362_CS_PORT		0
#define BLUEIO_ADXL_WCHRG_ADXL362_CS_PIN		5
#define BLUEIO_ADXL_WCHRG_ADXL362_CS_PPINOP		0

#define BLUEIO_ADXL_WCHRG_ADXL362_INT1_PORT		0
#define BLUEIO_ADXL_WCHRG_ADXL362_INT1_PIN		6
#define BLUEIO_ADXL_WCHRG_ADXL362_INT1_PPINOP	0

#define BLUEIO_ADXL_WCHRG_ADXL362_INT1_PORT		0
#define BLUEIO_ADXL_WCHRG_ADXL362_INT1_PIN		6
#define BLUEIO_ADXL_WCHRG_ADXL362_INT1_PPINOP	0

#define BLUEIO_ADXL_WCHRG_EEPROM_SCL_PORT		0
#define BLUEIO_ADXL_WCHRG_EEPROM_SCL_PIN		11
#define BLUEIO_ADXL_WCHRG_EEPROM_SCL_PINOP		1
#define BLUEIO_ADXL_WCHRG_EEPROM_SDA_PORT		0
#define BLUEIO_ADXL_WCHRG_EEPROM_SDA_PIN		12
#define BLUEIO_ADXL_WCHRG_EEPROM_SDA_PINOP		1

#define BLUEIO_ADXL_WCHRG_EEPROM_WRPROT_PORT	0
#define BLUEIO_ADXL_WCHRG_EEPROM_WRPROT_PIN		20
#define BLUEIO_ADXL_WCHRG_EEPROM_WRPROT_PORT	0

#define BLUEIO_ADXL_WCHRG_FLASH_SCK_PORT		0
#define BLUEIO_ADXL_WCHRG_FLASH_SCK_PIN			23
#define BLUEIO_ADXL_WCHRG_FLASH_SCK_PINOP		1
#define BLUEIO_ADXL_WCHRG_FLASH_MOSI_PORT		0
#define BLUEIO_ADXL_WCHRG_FLASH_MOSI_PIN		24
#define BLUEIO_ADXL_WCHRG_FLASH_MOSI_PINOP		1
#define BLUEIO_ADXL_WCHRG_FLASH_MISO_PORT		0
#define BLUEIO_ADXL_WCHRG_FLASH_MISO_PIN		25
#define BLUEIO_ADXL_WCHRG_FLASH_MISO_PINOP		1

#define BLUEIO_ADXL_WCHRG_FLASH_CS_PORT			0
#define BLUEIO_ADXL_WCHRG_FLASH_CS_PIN			26
#define BLUEIO_ADXL_WCHRG_FLASH_CS_PINOP		0

// BLUEIO-ADXL-WCHRG *****

// ***** BLYST-NANO-DK

// LED
#define BLYST_NANO_DK_LED1_PORT					0
#define BLYST_NANO_DK_LED1_PIN					30
#define BLYST_NANO_DK_LED1_PINOP				0
#define BLYST_NANO_DK_LED1_ACTIVE				LED_LOGIC_LOW

#define BLYST_NANO_DK_LEDR_PORT					0
#define BLYST_NANO_DK_LEDR_PIN					28
#define BLYST_NANO_DK_LEDR_PINOP				0
#define BLYST_NANO_DK_LEDR_ACTIVE				LED_LOGIC_LOW
#define BLYST_NANO_DK_LEDG_PORT					0
#define BLYST_NANO_DK_LEDG_PIN					29
#define BLYST_NANO_DK_LEDG_PINOP				0
#define BLYST_NANO_DK_LEDG_ACTIVE				LED_LOGIC_LOW
#define BLYST_NANO_DK_LEDB_PORT					0
#define BLYST_NANO_DK_LEDB_PIN					27
#define BLYST_NANO_DK_LEDB_PINOP				0
#define BLYST_NANO_DK_LEDB_ACTIVE				LED_LOGIC_LOW

#define BLYST_NANO_DK_LED_PINS_CFG			{ \
	{BLYST_NANO_DK_LED1_PORT, BLYST_NANO_DK_LED1_PIN, BLYST_NANO_DK_LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLYST_NANO_DK_LEDR_PORT, BLYST_NANO_DK_LEDR_PIN, BLYST_NANO_DK_LEDR_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLYST_NANO_DK_LEDG_PORT, BLYST_NANO_DK_LEDG_PIN, BLYST_NANO_DK_LEDG_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLYST_NANO_DK_LEDB_PORT, BLYST_NANO_DK_LEDB_PIN, BLYST_NANO_DK_LEDB_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Button
#define BLYST_NANO_DK_BUT1_PORT					0
#define BLYST_NANO_DK_BUT1_PIN					2
#define BLYST_NANO_DK_BUT1_PINOP				0

#define BLYST_NANO_DK_BUT1_SENSE				IOPINSENSE_LOW_TRANSITION

#define BLYST_NANO_DK_BUT2_PORT					0
#define BLYST_NANO_DK_BUT2_PIN					13
#define BLYST_NANO_DK_BUT2_PINOP				0

#define BLYST_NANO_DK_BUT2_SENSE				IOPINSENSE_LOW_TRANSITION

#define BLYST_NANO_DK_BUT_PINS_CFG			{ \
	{BLYST_NANO_DK_BUT1_PORT, BLYST_NANO_DK_BUT1_PIN, BLYST_NANO_DK_BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BLYST_NANO_DK_BUT2_PORT, BLYST_NANO_DK_BUT2_PIN, BLYST_NANO_DK_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// UART pins
#define BLYST_NANO_DK_UART_RX_PORT				0
#define BLYST_NANO_DK_UART_RX_PIN				8
#define BLYST_NANO_DK_UART_RX_PINOP				1
#define BLYST_NANO_DK_UART_TX_PORT				0
#define BLYST_NANO_DK_UART_TX_PIN				7
#define BLYST_NANO_DK_UART_TX_PINOP				1
#define BLYST_NANO_DK_UART_CTS_PORT				0
#define BLYST_NANO_DK_UART_CTS_PIN				12
#define BLYST_NANO_DK_UART_CTS_PINOP			1
#define BLYST_NANO_DK_UART_RTS_PORT				0
#define BLYST_NANO_DK_UART_RTS_PIN				11
#define BLYST_NANO_DK_UART_RTS_PINOP			1

#define BLYST_NANO_DK_UART_PINS_CFG	{ \
	{BLYST_NANO_DK_UART_RX_PORT, BLYST_NANO_DK_UART_RX_PIN, BLYST_NANO_DK_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLYST_NANO_DK_UART_TX_PORT, BLYST_NANO_DK_UART_TX_PIN, BLYST_NANO_DK_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLYST_NANO_DK_UART_CTS_PORT, BLYST_NANO_DK_UART_CTS_PIN, BLYST_NANO_DK_UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLYST_NANO_DK_UART_RTS_PORT, BLYST_NANO_DK_UART_RTS_PIN, BLYST_NANO_DK_UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

// BLYST-NANO-DK *****

// ***** BLUEIO832-MINI

// BLUEIO832-MINI LED pins
#define BLUEIO832_MINI_LEDR_PORT				0
#define BLUEIO832_MINI_LEDR_PIN					18
#define BLUEIO832_MINI_LEDR_PINOP				0
#define BLUEIO832_MINI_LEDR_ACTIVE				LED_LOGIC_HIGH

#define BLUEIO832_MINI_LEDG_PORT				0
#define BLUEIO832_MINI_LEDG_PIN					16
#define BLUEIO832_MINI_LEDG_PINOP				0
#define BLUEIO832_MINI_LEDG_ACTIVE				LED_LOGIC_HIGH

#define BLUEIO832_MINI_LEDB_PORT				0
#define BLUEIO832_MINI_LEDB_PIN					17
#define BLUEIO832_MINI_LEDB_PINOP				0
#define BLUEIO832_MINI_LEDB_ACTIVE				LED_LOGIC_HIGH

#define BLUEIO832_MINI_LED_PINS_CFG		{ \
	{BLUEIO832_MINI_LEDR_PORT, BLUEIO832_MINI_LEDR_PIN, BLUEIO832_MINI_LEDR_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO832_MINI_LEDG_PORT, BLUEIO832_MINI_LEDG_PIN, BLUEIO832_MINI_LEDG_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEIO832_MINI_LEDB_PORT, BLUEIO832_MINI_LEDB_PIN, BLUEIO832_MINI_LEDB_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Analog input
#define BLUEIO832_MINI_A0						0	// AIN0
#define BLUEIO832_MINI_A1						1	// AIN1
#define BLUEIO832_MINI_A2						2	// AIN2
#define BLUEIO832_MINI_VIN_AIN3					3	// AIN3

// Digital I/O
#define BLUEIO832_MINI_D0_PORT					0
#define BLUEIO832_MINI_D0_PIN					12
#define BLUEIO832_MINI_D0_PINOP					0
#define BLUEIO832_MINI_D1_PORT					0
#define BLUEIO832_MINI_D1_PIN					11
#define BLUEIO832_MINI_D1_PINOP					0
#define BLUEIO832_MINI_D2_PORT					0
#define BLUEIO832_MINI_D2_PIN					8
#define BLUEIO832_MINI_D2_PINOP					0
#define BLUEIO832_MINI_D3_PORT					0
#define BLUEIO832_MINI_D3_PIN					7
#define BLUEIO832_MINI_D3_PINOP					0
#define BLUEIO832_MINI_D4_PORT					0
#define BLUEIO832_MINI_D4_PIN					6
#define BLUEIO832_MINI_D4_PINOP					0
#define BLUEIO832_MINI_D5_PORT					0
#define BLUEIO832_MINI_D5_PIN					13
#define BLUEIO832_MINI_D5_PINOP					0

// BLUEIO832-MINI *****

// ***** BLUEPYRO-M3225

#define BLUEPYRO_M3225_DL_PORT					0
#define BLUEPYRO_M3225_DL_PIN					6
#define BLUEPYRO_M3225_DL_PINOP					0

#define BLUEPYRO_M3225_INT_NO					0

#define BLUEPYRO_M3225_SI_PORT					0
#define BLUEPYRO_M3225_SI_PIN					7
#define BLUEPYRO_M3225_SI_PINOP					0

#define BLUEPYRO_M3225_P1_PORT					0
#define BLUEPYRO_M3225_P1_PIN					22
#define BLUEPYRO_M3225_P2_PORT					0
#define BLUEPYRO_M3225_P2_PIN					20
#define BLUEPYRO_M3225_P3_PORT					0
#define BLUEPYRO_M3225_P3_PIN					19
#define BLUEPYRO_M3225_P4_PORT					0
#define BLUEPYRO_M3225_P4_PIN					18
#define BLUEPYRO_M3225_P5_PORT					0
#define BLUEPYRO_M3225_P5_PIN					17
#define BLUEPYRO_M3225_P6_PORT					0
#define BLUEPYRO_M3225_P6_PIN					16
#define BLUEPYRO_M3225_P7_PORT					0
#define BLUEPYRO_M3225_P7_PIN					15
#define BLUEPYRO_M3225_P8_PORT					0
#define BLUEPYRO_M3225_P8_PIN					14
#define BLUEPYRO_M3225_P9_PORT					0
#define BLUEPYRO_M3225_P9_PIN					13
#define BLUEPYRO_M3225_P10_PORT					0
#define BLUEPYRO_M3225_P10_PIN					12
#define BLUEPYRO_M3225_P11_PORT					0
#define BLUEPYRO_M3225_P11_PIN					11
#define BLUEPYRO_M3225_P12_PORT					0
#define BLUEPYRO_M3225_P12_PIN					10		// NFC2
#define BLUEPYRO_M3225_P13_PORT					0
#define BLUEPYRO_M3225_P13_PIN					9		// NFC1
#define BLUEPYRO_M3225_P14_PORT					0
#define BLUEPYRO_M3225_P14_PIN					8
//#define BLUEPYRO_M3225_P15_PORT	// Unused
//#define BLUEPYRO_M3225_P15_PIN
#define BLUEPYRO_M3225_P16_PORT					0
#define BLUEPYRO_M3225_P16_PIN					5
#define BLUEPYRO_M3225_P17_PORT					0
#define BLUEPYRO_M3225_P17_PIN					4

//#define BLUEPYRO_M3225_P18_PORT					0
//#define BLUEPYRO_M3225_P18_PIN					5
//#define BLUEPYRO_M3225_P19_PORT					0
//#define BLUEPYRO_M3225_P19_PIN					5
//#define BLUEPYRO_M3225_P20_PORT					0
//#define BLUEPYRO_M3225_P20_PIN					5

#define BLUEPYRO_M3225_P23_PORT					0
#define BLUEPYRO_M3225_P23_PIN					3
#define BLUEPYRO_M3225_P24_PORT					0
#define BLUEPYRO_M3225_P24_PIN					2
#define BLUEPYRO_M3225_P25_PORT					0
#define BLUEPYRO_M3225_P25_PIN					31
#define BLUEPYRO_M3225_P26_PORT					0
#define BLUEPYRO_M3225_P26_PIN					30
#define BLUEPYRO_M3225_P27_PORT					0
#define BLUEPYRO_M3225_P27_PIN					29
#define BLUEPYRO_M3225_P28_PORT					0
#define BLUEPYRO_M3225_P28_PIN					28
#define BLUEPYRO_M3225_P29_PORT					0
#define BLUEPYRO_M3225_P29_PIN					27
#define BLUEPYRO_M3225_P30_PORT					0
#define BLUEPYRO_M3225_P30_PIN					26
#define BLUEPYRO_M3225_P31_PORT					0
#define BLUEPYRO_M3225_P31_PIN					25
#define BLUEPYRO_M3225_P32_PORT					0
#define BLUEPYRO_M3225_P32_PIN					24
#define BLUEPYRO_M3225_P33_PORT					0
#define BLUEPYRO_M3225_P33_PIN					23

// BLUEPYRO-M3225 *****

// ***** BLUEPYRO-CR24S-M3225

// Button
#define BLUEPYRO_CR24S_M3225_BUT2_PORT			BLUEPYRO_M3225_P10_PORT
#define BLUEPYRO_CR24S_M3225_BUT2_PIN			BLUEPYRO_M3225_P10_PIN
#define BLUEPYRO_CR24S_M3225_BUT2_PINOP			0

#define BLUEPYRO_CR24S_M3225_BUT2_SENSE			IOPINSENSE_LOW_TRANSITION

#define BLUEPYRO_CR24S_M3225_PINS_CFG			{ \
	{BLUEPYRO_CR24S_M3225_BUT2_PORT, BLUEPYRO_CR24S_M3225_BUT2_PIN, BLUEPYRO_CR24S_M3225_BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// LED
#define BLUEPYRO_CR24S_M3225_LED1_PORT			BLUEPYRO_M3225_P26_PORT
#define BLUEPYRO_CR24S_M3225_LED1_PIN			BLUEPYRO_M3225_P26_PIN
#define BLUEPYRO_CR24S_M3225_LED1_PINOP			0
#define BLUEPYRO_CR24S_M3225_LED1_ACTIVE		LED_LOGIC_LOW

#define BLUEPYRO_CR24S_M3225_LED2R_PORT			BLUEPYRO_M3225_P1_PORT
#define BLUEPYRO_CR24S_M3225_LED2R_PIN			BLUEPYRO_M3225_P1_PIN
#define BLUEPYRO_CR24S_M3225_LED2R_PINOP		0
#define BLUEPYRO_CR24S_M3225_LED2R_ACTIVE		LED_LOGIC_LOW

#define BLUEPYRO_CR24S_M3225_LED2G_PORT			BLUEPYRO_M3225_P3_PORT
#define BLUEPYRO_CR24S_M3225_LED2G_PIN			BLUEPYRO_M3225_P3_PIN
#define BLUEPYRO_CR24S_M3225_LED2G_PINOP		0
#define BLUEPYRO_CR24S_M3225_LED2G_ACTIVE		LED_LOGIC_LOW

#define BLUEPYRO_CR24S_M3225_LED2B_PORT			BLUEPYRO_M3225_P2_PORT
#define BLUEPYRO_CR24S_M3225_LED2B_PIN			BLUEPYRO_M3225_P2_PIN
#define BLUEPYRO_CR24S_M3225_LED2B_PINOP		0
#define BLUEPYRO_CR24S_M3225_LED2B_ACTIVE		LED_LOGIC_LOW

#define BLUEPYRO_CR24S_M3225_LED_PINS_CFG		{ \
	{BLUEPYRO_CR24S_M3225_LED1_PORT, BLUEPYRO_CR24S_M3225_LED1_PIN, BLUEPYRO_CR24S_M3225_LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEPYRO_CR24S_M3225_LED2R_PORT, BLUEPYRO_CR24S_M3225_LED2R_PIN, BLUEPYRO_CR24S_M3225_LED2B_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEPYRO_CR24S_M3225_LED2G_PORT, BLUEPYRO_CR24S_M3225_LED2G_PIN, BLUEPYRO_CR24S_M3225_LED2B_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLUEPYRO_CR24S_M3225_LED2B_PORT, BLUEPYRO_CR24S_M3225_LED2B_PIN, BLUEPYRO_CR24S_M3225_LED2B_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// UART pins
#define BLUEPYRO_CR24S_M3225_UART_RX_PORT		BLUEPYRO_M3225_P16_PORT
#define BLUEPYRO_CR24S_M3225_UART_RX_PIN		BLUEPYRO_M3225_P16_PIN
#define BLUEPYRO_CR24S_M3225_UART_RX_PINOP		1
#define BLUEPYRO_CR24S_M3225_UART_TX_PORT		BLUEPYRO_M3225_P17_PORT
#define BLUEPYRO_CR24S_M3225_UART_TX_PIN		BLUEPYRO_M3225_P17_PIN
#define BLUEPYRO_CR24S_M3225_UART_TX_PINOP		1

#define BLUEPYRO_CR24S_M3225_UART_PINS_CFG	{ \
	{BLUEPYRO_CR24S_M3225_UART_RX_PORT, BLUEPYRO_CR24S_M3225_UART_RX_PIN, BLUEPYRO_CR24S_M3225_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLUEPYRO_CR24S_M3225_UART_TX_PORT, BLUEPYRO_CR24S_M3225_UART_TX_PIN, BLUEPYRO_CR24S_M3225_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
}

#define BLUEPYRO_CR24S_M3225_IN_PORT			BLUEPYRO_CR24S_M3225_BUT2_PORT
#define BLUEPYRO_CR24S_M3225_IN_PIN				BLUEPYRO_CR24S_M3225_BUT2_PIN
#define BLUEPYRO_CR24S_M3225_IN_PINOP			BLUEPYRO_CR24S_M3225_BUT2_PINOP

#define BLUEPYRO_CR24S_M3225_OUT_PORT			BLUEPYRO_M3225_P14_PORT
#define BLUEPYRO_CR24S_M3225_OUT_PIN			BLUEPYRO_M3225_P14_PIN
#define BLUEPYRO_CR24S_M3225_OUT_PINOP			0

// BLUEPYRO-CR24S-M3225 *****

// ***** BLYST-MOTION

#define BLYST_MOTION_LED_RED_PORT				0
#define BLYST_MOTION_LED_RED_PIN				25
#define BLYST_MOTION_LED_RED_PINOP				0

#define BLYST_MOTION_LED_GREEN_PORT				0
#define BLYST_MOTION_LED_GREEN_PIN				22
#define BLYST_MOTION_LED_GREEN_PINOP			0

#define BLYST_MOTION_LED_BLUE_PORT				0
#define BLYST_MOTION_LED_BLUE_PIN				20
#define BLYST_MOTION_LED_BLUE_PINOP				0

#define BLYST_MOTION_I2C_SDA_PORT				0
#define BLYST_MOTION_I2C_SDA_PIN				18
#define BLYST_MOTION_I2C_SDA_PINOP				1
#define BLYST_MOTION_I2C_SCL_PORT				0
#define BLYST_MOTION_I2C_SCL_PIN				17
#define BLYST_MOTION_I2C_SCL_PINOP				1

#define BLYST_MOTION_I2C_PINS_CFG	{ \
	{BLYST_MOTION_I2C_SDA_PORT, BLYST_MOTION_I2C_SDA_PIN, BLYST_MOTION_I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLYST_MOTION_I2C_SCL_PORT, BLYST_MOTION_I2C_SCL_PIN, BLYST_MOTION_I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, }


#define BLYST_MOTION_BMM350_INT_PORT			0
#define BLYST_MOTION_BMM350_INT_PIN				19
#define BLYST_MOTION_BMM350_INT_PINOP			0

#define BLYST_MOTION_SPI_SCK_PORT				0
#define BLYST_MOTION_SPI_SCK_PIN				11
#define BLYST_MOTION_SPI_SCK_PINOP				1
#define BLYST_MOTION_SPI_MISO_PORT				0
#define BLYST_MOTION_SPI_MISO_PIN				12
#define BLYST_MOTION_SPI_MISO_PINOP				1
#define BLYST_MOTION_SPI_MOSI_PORT				0
#define BLYST_MOTION_SPI_MOSI_PIN				13
#define BLYST_MOTION_SPI_MOSI_PINOP				1

#define BLYST_MOTION_IMU_CS_PORT				0
#define BLYST_MOTION_IMU_CS_PIN					6
#define BLYST_MOTION_IMU_CS_PINOP				0

#define BLYST_MOTION_IMU_INT_PORT				0
#define BLYST_MOTION_IMU_INT_PIN				16
#define BLYST_MOTION_IMU_INT_PINOP				0

#define BLYST_MOTION_FLASH_CS_PORT				0
#define BLYST_MOTION_FLASH_CS_PIN				5
#define BLYST_MOTION_FLASH_CS_PINOP				0

#define BLYST_MOTION_SPI_PINS_CFG	{\
	{BLYST_MOTION_SPI_SCK_PORT, BLYST_MOTION_SPI_SCK_PIN, BLYST_MOTION_SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BLYST_MOTION_SPI_MISO_PORT, BLYST_MOTION_SPI_MISO_PIN, BLYST_MOTION_SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {BLYST_MOTION_SPI_MOSI_PORT, BLYST_MOTION_SPI_MOSI_PIN, BLYST_MOTION_SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {BLYST_MOTION_IMU_CS_PORT, BLYST_MOTION_IMU_CS_PIN, BLYST_MOTION_IMU_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BLYST_MOTION_FLASH_CS_PORT, BLYST_MOTION_FLASH_CS_PIN, BLYST_MOTION_FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},}	// CS

#define BLYST_MOTION_UART_RX_PORT				0
#define BLYST_MOTION_UART_RX_PIN				8
#define BLYST_MOTION_UART_RX_PINOP				1
#define BLYST_MOTION_UART_TX_PORT				0
#define BLYST_MOTION_UART_TX_PIN				7
#define BLYST_MOTION_UART_TX_PINOP				1

#define BLYST_MOTION_UART_PINS_CFG	{ \
	{BLYST_MOTION_UART_RX_PORT, BLYST_MOTION_UART_RX_PIN, BLYST_MOTION_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{BLYST_MOTION_UART_TX_PORT, BLYST_MOTION_UART_TX_PIN, BLYST_MOTION_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}


// BLYST-MOTION *****

#endif // __BLYSTNANO_BOARDS_H__
