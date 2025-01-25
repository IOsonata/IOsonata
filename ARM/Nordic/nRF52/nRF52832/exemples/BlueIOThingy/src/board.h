/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

#include "blystnano_boards.h"
//#include "blueio_board.h"

#define BOARD		BLUEIO_TAG_EVIM
//#define BOARD		BLYST_NANO_DK
//#define BOARD		BLUEIO_THINGY

#define BUT_INT_NO					0
#define IMU_INT_NO					1
#define IMU_INT_PRIO				6

#if BOARD == BLUEIO_THINGY

#define BUTTON_PINS_MAP		BLUEIO_TAG_EVIM_BUT_PINS_CFG
#define BUT1_SENSE			BLUEIO_TAG_EVIM_BUT1_SENSE
#define BUT2_SENSE			BLUEIO_TAG_EVIM_BUT2_SENSE

#define LED_PINS_MAP		BLUEIO_TAG_EVIM_LED_PINS_CFG

#define BLUEIOTHINGY_CONNECT_LED_PORT		BLUEIO_TAG_EVIM_LED1_PORT
#define BLUEIOTHINGY_CONNECT_LED_PIN		BLUEIO_TAG_EVIM_LED1_PIN
#define BLUEIOTHINGY_CONNECT_LED_LOGIC		BLUEIO_TAG_EVIM_LED1_LOGIC

#define BLUEIOTHINGY_LEDR_PORT				BLUEIO_TAG_EVIM_LED2R_PORT
#define BLUEIOTHINGY_LEDR_PIN				BLUEIO_TAG_EVIM_LED2R_PIN
#define BLUEIOTHINGY_LEDG_PORT				BLUEIO_TAG_EVIM_LED2G_PORT
#define BLUEIOTHINGY_LEDG_PIN				BLUEIO_TAG_EVIM_LED2G_PIN
#define BLUEIOTHINGY_LEDB_PORT				BLUEIO_TAG_EVIM_LED2B_PORT
#define BLUEIOTHINGY_LEDB_PIN				BLUEIO_TAG_EVIM_LED2B_PIN

#define BLUEIOTHINGY_IMU_INT_PORT			BLUEIO_TAG_EVIM_IMU_INT_PORT
#define BLUEIOTHINGY_IMU_INT_PIN			BLUEIO_TAG_EVIM_IMU_INT_PIN
#define BLUEIOTHINGY_IMU_INT_PINOP			BLUEIO_TAG_EVIM_IMU_INT_PINOP

#define BLUEIOTHINGY_I2C0_DEVNO				0
#define BLUEIOTHINGY_I2C0_PINS_MAP			BLUEIO_TAG_EVIM_I2C0_PINS_CFG

#define BLUEIOTHINGY_SPI2_DEVNO				2
#define BLUEIOTHINGY_SPI2_PINS_MAP			BLUEIO_TAG_EVIM_SPI2_PINS_CFG

#define BLUEIO_UART_RX_PORT					0
#define BLUEIO_UART_RX_PIN					8
#define BLUEIO_UART_RX_PINOP				0
#define BLUEIO_UART_TX_PORT					0
#define BLUEIO_UART_TX_PIN					7
#define BLUEIO_UART_TX_PINOP				0
#define BLUEIO_UART_CTS_PORT				0
#define BLUEIO_UART_CTS_PIN					12
#define BLUEIO_UART_CTS_PINOP				0
#define BLUEIO_UART_RTS_PORT				0
#define BLUEIO_UART_RTS_PIN					11
#define BLUEIO_UART_RTS_PINOP				0

#define UART_RX_PORT		BLUEIO_UART_RX_PORT
#define UART_RX_PIN			BLUEIO_UART_RX_PIN
#define UART_RX_PINOP		BLUEIO_UART_RX_PINOP
#define UART_TX_PORT		BLUEIO_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_UART_TX_PINOP
#define UART_CTS_PORT		BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN		BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP		BLUEIO_UART_CTS_PINOP
#define UART_RTS_PORT		BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN		BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP		BLUEIO_UART_RTS_PINOP

#elif BOARD == BLUEIO_TAG_EVIM
#define BUT_PINS			BLUEIO_TAG_EVIM_BUT_PINS_CFG
#define BUT1_SENSE			BLUEIO_TAG_EVIM_BUT1_SENSE
#define BUT2_SENSE			BLUEIO_TAG_EVIM_BUT1_SENSE
#define BUT_PIN_SENSE		BUT1_SENSE

// LEDs
#define CONNECT_LED_PORT	BLUEIO_TAG_EVIM_LED1_PORT
#define CONNECT_LED_PIN		BLUEIO_TAG_EVIM_LED1_PIN
#define CONNECT_LED_LOGIC	BLUEIO_TAG_EVIM_LED1_LOGIC

#define LED1_PORT			BLUEIO_TAG_EVIM_LED1_PORT
#define LED1_PIN			BLUEIO_TAG_EVIM_LED1_PIN
#define LED1_PINOP			BLUEIO_TAG_EVIM_LED1_PINOP
#define LED1_LOGIC			BLUEIO_TAG_EVIM_LED1_LOGIC

#define LED_RED_PORT		BLUEIO_TAG_EVIM_LED2R_PORT
#define LED_RED_PIN			BLUEIO_TAG_EVIM_LED2R_PIN
#define LED_RED_PINOP		BLUEIO_TAG_EVIM_LED2R_PINOP
#define LED_RED_LOGIC		BLUEIO_TAG_EVIM_LED2R_LOGIC

#define LED_GREEN_PORT		BLUEIO_TAG_EVIM_LED2G_PORT
#define LED_GREEN_PIN		BLUEIO_TAG_EVIM_LED2G_PIN
#define LED_GREEN_PINOP		BLUEIO_TAG_EVIM_LED2G_PINOP
#define LED_GREEN_LOGIC		BLUEIO_TAG_EVIM_LED2G_LOGIC

#define LED_BLUE_PORT		BLUEIO_TAG_EVIM_LED2B_PORT
#define LED_BLUE_PIN		BLUEIO_TAG_EVIM_LED2B_PIN
#define LED_BLUE_PINOP		BLUEIO_TAG_EVIM_LED2B_PINOP
#define LED_BLUE_LOGIC		BLUEIO_TAG_EVIM_LED2B_LOGIC

#define LED_PINS {\
	{LED_RED_PORT, LED_RED_PIN, LED_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{LED_BLUE_PORT, LED_BLUE_PIN, LED_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// UARTs for debug
#define UART_DEVNO			0
#define UART_RX_PORT		BLUEIO_TAG_EVIM_UART_RX_PORT
#define UART_RX_PIN			BLUEIO_TAG_EVIM_UART_RX_PIN
#define UART_RX_PINOP		BLUEIO_TAG_EVIM_UART_RX_PINOP

#define UART_TX_PORT		BLUEIO_TAG_EVIM_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_TAG_EVIM_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_TAG_EVIM_UART_TX_PINOP

#define UART_PINS  		{\
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}

#define I2C_DEVNO			0

#define I2C_SDA_PORT		BLUEIO_TAG_EVIM_I2C0_SDA_PORT
#define I2C_SDA_PIN			BLUEIO_TAG_EVIM_I2C0_SDA_PIN
#define I2C_SDA_PINOP		BLUEIO_TAG_EVIM_I2C0_SDA_PINOP

#define I2C_SCL_PORT		BLUEIO_TAG_EVIM_I2C0_SCL_PORT
#define I2C_SCL_PIN			BLUEIO_TAG_EVIM_I2C0_SCL_PIN
#define I2C_SCL_PINOP		BLUEIO_TAG_EVIM_I2C0_SCL_PINOP

#define I2C_PINS {\
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},\
};

// SPI Bus
#define SPI_DEVNO			2

#define SPI_SCK_PORT		BLUEIO_TAG_EVIM_SPI2_SCK_PORT
#define SPI_SCK_PIN			BLUEIO_TAG_EVIM_SPI2_SCK_PIN
#define SPI_SCK_PINOP		BLUEIO_TAG_EVIM_SPI2_SCK_PINOP
// DIN
#define SPI_MISO_PORT		BLUEIO_TAG_EVIM_SPI2_MISO_PORT
#define SPI_MISO_PIN		BLUEIO_TAG_EVIM_SPI2_MISO_PIN
#define SPI_MISO_PINOP		BLUEIO_TAG_EVIM_SPI2_MISO_PINOP
// DOUT
#define SPI_MOSI_PORT		BLUEIO_TAG_EVIM_SPI2_MOSI_PORT
#define SPI_MOSI_PIN		BLUEIO_TAG_EVIM_SPI2_MOSI_PIN
#define SPI_MOSI_PINOP		BLUEIO_TAG_EVIM_SPI2_MOSI_PINOP

// IMU
#define IMU_CS_PORT			BLUEIO_TAG_EVIM_IMU_CS_PORT
#define IMU_CS_PIN			BLUEIO_TAG_EVIM_IMU_CS_PIN
#define IMU_CS_PINOP		BLUEIO_TAG_EVIM_IMU_CS_PINOP

#define IMU_INT_PORT		BLUEIO_TAG_EVIM_IMU_INT_PORT
#define IMU_INT_PIN			BLUEIO_TAG_EVIM_IMU_INT_PIN
#define IMU_INT_PINOP		BLUEIO_TAG_EVIM_IMU_INT_PINOP

#define SPI_PINS 			BLUEIO_TAG_EVIM_SPI2_PINS_CFG

#elif BOARD == BLYST_NANO_DK
#define BUT_PINS			BLYST_NANO_DK_BUT_PINS_CFG


// LEDs
#define CONNECT_LED_PORT	BLYST_NANO_DK_LEDB_PORT
#define CONNECT_LED_PIN		BLYST_NANO_DK_LEDB_PIN
#define CONNECT_LED_LOGIC	BLYST_NANO_DK_LEDB_LOGIC

#define LED_RED_PORT		BLYST_NANO_DK_LEDR_PORT
#define LED_RED_PIN			BLYST_NANO_DK_LEDR_PIN
#define LED_RED_PINOP		BLYST_NANO_DK_LEDR_PINOP
#define LED_RED_LOGIC		BLYST_NANO_DK_LEDR_LOGIC

#define LED_GREEN_PORT		BLYST_NANO_DK_LEDG_PORT
#define LED_GREEN_PIN		BLYST_NANO_DK_LEDG_PIN
#define LED_GREEN_PINOP		BLYST_NANO_DK_LEDG_PINOP
#define LED_GREEN_LOGIC		BLYST_NANO_DK_LEDG_LOGIC

#define LED_BLUE_PORT		BLYST_NANO_DK_LEDB_PORT
#define LED_BLUE_PIN		BLYST_NANO_DK_LEDB_PIN
#define LED_BLUE_PINOP		BLYST_NANO_DK_LEDB_PINOP
#define LED_BLUE_LOGIC		BLYST_NANO_DK_LEDB_LOGIC

#define LED_PINS {\
	{LED_RED_PORT, LED_RED_PIN, LED_RED_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{LED_BLUE_PORT, LED_BLUE_PIN, LED_BLUE_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// UARTs for debug
#define UART_DEVNO			0
#define UART_RX_PORT		BLYST_NANO_DK_UART_RX_PORT
#define UART_RX_PIN			BLYST_NANO_DK_UART_RX_PIN
#define UART_RX_PINOP		BLYST_NANO_DK_UART_RX_PINOP

#define UART_TX_PORT		BLYST_NANO_DK_UART_TX_PORT
#define UART_TX_PIN			BLYST_NANO_DK_UART_TX_PIN
#define UART_TX_PINOP		BLYST_NANO_DK_UART_TX_PINOP

#define UART_PINS  		{\
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}

#define I2C_DEVNO			0

#define I2C_SDA_PORT		0
#define I2C_SDA_PIN			4
#define I2C_SDA_PINOP		1

#define I2C_SCL_PORT		0
#define I2C_SCL_PIN			3
#define I2C_SCL_PINOP		1

#define I2C_PINS {\
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},\
};

// SPI Bus
#define SPI_DEVNO			2

#define SPI_SCK_PORT		BLUEIO_TAG_EVIM_SPI2_SCK_PORT
#define SPI_SCK_PIN			BLUEIO_TAG_EVIM_SPI2_SCK_PIN
#define SPI_SCK_PINOP		BLUEIO_TAG_EVIM_SPI2_SCK_PINOP
// DIN
#define SPI_MISO_PORT		BLUEIO_TAG_EVIM_SPI2_MISO_PORT
#define SPI_MISO_PIN		BLUEIO_TAG_EVIM_SPI2_MISO_PIN
#define SPI_MISO_PINOP		BLUEIO_TAG_EVIM_SPI2_MISO_PINOP
// DOUT
#define SPI_MOSI_PORT		BLUEIO_TAG_EVIM_SPI2_MOSI_PORT
#define SPI_MOSI_PIN		BLUEIO_TAG_EVIM_SPI2_MOSI_PIN
#define SPI_MOSI_PINOP		BLUEIO_TAG_EVIM_SPI2_MOSI_PINOP

// IMU
#define IMU_CS_PORT			BLUEIO_TAG_EVIM_IMU_CS_PORT
#define IMU_CS_PIN			BLUEIO_TAG_EVIM_IMU_CS_PIN
#define IMU_CS_PINOP		BLUEIO_TAG_EVIM_IMU_CS_PINOP

#define IMU_INT_PORT		BLUEIO_TAG_EVIM_IMU_INT_PORT
#define IMU_INT_PIN			BLUEIO_TAG_EVIM_IMU_INT_PIN
#define IMU_INT_PINOP		BLUEIO_TAG_EVIM_IMU_INT_PINOP

#define SPI_PINS {\
    {SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
    {IMU_CS_PORT, IMU_CS_PIN, IMU_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
};

#else

//#define NORDIC_DK			// For Nordic DevKit

#ifdef NORDIC_DK

#define UART_TX_PIN					9//7
#define UART_RX_PIN					11//8
#define UART_RTS_PIN					8//11
#define UART_CTS_PIN					10//12

#define SPI_DEVNO            		2
#define SPI_MISO_PORT        		0
#define SPI_MISO_PIN         		13
#define SPI_MISO_PINOP       		1
#define SPI_MOSI_PORT        		0
#define SPI_MOSI_PIN         		12
#define SPI_MOSI_PINOP       		1
#define SPI_SCK_PORT         		0
#define SPI_SCK_PIN          		11
#define SPI_SCK_PINOP        		1

#define SPI_BME280_CS_PORT         	0
#define SPI_BME280_CS_PIN          	26
#define SPI_BME280_CS_PINOP        	1

#else

// I-SYST BlueIO boards

#define UART_RX_PORT					BLUEIO_UART_RX_PORT
#define UART_RX_PIN						BLUEIO_UART_RX_PIN
#define UART_RX_PINOP					BLUEIO_UART_RX_PINOP
#define UART_TX_PORT					BLUEIO_UART_TX_PORT
#define UART_TX_PIN						BLUEIO_UART_TX_PIN
#define UART_TX_PINOP					BLUEIO_UART_TX_PINOP
#define UART_CTS_PORT					BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN					BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP					BLUEIO_UART_CTS_PINOP
#define UART_RTS_PORT					BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN					BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP					BLUEIO_UART_RTS_PINOP

//#define BLUEIO_TAG_BME680_PROTO
#ifdef BLUEIO_TAG_BME680_PROTO
#define I2C0_SDA_PORT					BLUEIO_TAG_BME280_I2C_SDA_PORT
#define I2C0_SDA_PIN					BLUEIO_TAG_BME280_I2C_SDA_PIN
#define I2C0_SDA_PINOP					BLUEIO_TAG_BME280_I2C_SDA_PINOP
#define I2C0_SCL_PORT					BLUEIO_TAG_BME280_I2C_SCL_PORT
#define I2C0_SCL_PIN					BLUEIO_TAG_BME280_I2C_SCL_PIN
#define I2C0_SCL_PINOP					BLUEIO_TAG_BME280_I2C_SCL_PINOP
#else
#define I2C0_SDA_PORT					BLUEIO_TAG_BME680_I2C_SDA_PORT
#define I2C0_SDA_PIN					BLUEIO_TAG_BME680_I2C_SDA_PIN
#define I2C0_SDA_PINOP					BLUEIO_TAG_BME680_I2C_SDA_PINOP
#define I2C0_SCL_PORT					BLUEIO_TAG_BME680_I2C_SCL_PORT
#define I2C0_SCL_PIN					BLUEIO_TAG_BME680_I2C_SCL_PIN
#define I2C0_SCL_PINOP					BLUEIO_TAG_BME680_I2C_SCL_PINOP
#endif

#define SPI_DEVNO      					1
#define SPI_MISO_PORT       			0
#define SPI_MISO_PIN        			13
#define SPI_MISO_PINOP      			1
#define SPI_MOSI_PORT       			0
#define SPI_MOSI_PIN        			12
#define SPI_MOSI_PINOP      			1
#define SPI_SCK_PORT        			0
#define SPI_SCK_PIN         			11
#define SPI_SCK_PINOP       			1

#define SPI2_DEVNO      				2
#define SPI2_MISO_PORT       			0
#define SPI2_MISO_PIN        			15
#define SPI2_MISO_PINOP      			1
#define SPI2_MOSI_PORT       			0
#define SPI2_MOSI_PIN        			16
#define SPI2_MOSI_PINOP      			1
#define SPI2_SCK_PORT        			0
#define SPI2_SCK_PIN         			17
#define SPI2_SCK_PINOP       			1

#define BME280_CS_IDX          		0
#define BME280_CS_PORT         		0
#define BME280_CS_PIN          		26
#define BME280_CS_PINOP        		1

#define BLUEIO_TAG_EVIM_IMU_INT_NO		0

#endif
#endif

#endif // __BOARD_H__

