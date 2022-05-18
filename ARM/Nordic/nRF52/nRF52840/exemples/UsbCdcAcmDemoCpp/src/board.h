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

#include "blueio_board.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "cfifo.h"
#include "ble_app.h"
#include "ble_service.h"
#include "ble_intrf.h"
#include "bluetooth/blueio_blesrvc.h"

/// BlueIO breakout board

#define DEVICE_NAME     "BleCentral"				/**< Name of device. Will be included in the advertising data. */
#define MODEL_NAME      "Blyst840"               	/**< Model number. Will be passed to Device Information Service. */

//#define UDG

// LEDs
#ifdef UDG
#define LED1_PORT		0
#define LED1_PIN		6
#define LED1_PINOP		0

#define LED2_PORT		1
#define LED2_PIN		9
#define LED2_PINOP		0

#define LED3_PORT		0
#define LED3_PIN		8
#define LED3_PINOP		0

#define LED4_PORT		0
#define LED4_PIN		12
#define LED4_PINOP		0

#else
#define LED1_PORT		0
#define LED1_PIN		30
#define LED1_PINOP		0

#define LED2_PORT		1
#define LED2_PIN		8
#define LED2_PINOP		0

#define LED3_PORT		1
#define LED3_PIN		9
#define LED3_PINOP		0

#define LED4_PORT		BLUEIO_LED4_PORT
#define LED4_PIN		BLUEIO_LED4_PIN
#define LED4_PINOP		BLUEIO_LED4_PINOP
#endif

#define LED_BLUE_PORT 	LED1_PORT
#define LED_BLUE_PIN	LED1_PIN
#define LED_BLUE_PINOP	LED1_PINOP

#define LED_GREEN_PORT	LED2_PORT
#define LED_GREEN_PIN	LED2_PIN
#define LED_GREEN_PINOP	LED2_PINOP

#define LED_RED_PORT	LED3_PORT
#define LED_RED_PIN		LED3_PIN
#define LED_RED_PINOP	LED3_PINOP


#define LED_PIN_MAP 	{\
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
}

// Button pins
#define BUTTON1_PORT		BLUEIO_BUT1_PORT
#define BUTTON1_PIN			BLUEIO_BUT1_PIN
#define BUTTON2_PORT		BLUEIO_BUT2_PORT
#define BUTTON2_PIN			BLUEIO_BUT2_PIN

#define BUTTON_PIN_MAP  	{\
	{BUTTON1_PORT, BUTTON1_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{BUTTON2_PORT, BUTTON2_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
}



// UART pins
#ifdef UDG
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1

#define UART_TX_PORT		BLUEIO_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_UART_TX_PINOP

#define UART_CTS_PORT		0
#define UART_CTS_PIN		13
#define UART_CTS_PINOP		1

#define UART_RTS_PORT		BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN		BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP		BLUEIO_UART_RTS_PINOP
#else
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
#endif

/// UART pins definitions
#define UART_PIN_MAP  		{\
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
}

#define MANUFACTURER_NAME               "I-SYST inc."                       /**< Manufacturer. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID                  /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID                  /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                MSEC_TO_UNITS(64, UNIT_0_625_MS)	/**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_TIMEOUT					MSEC_TO_UNITS(0, UNIT_10_MS)		/**< The advertising timeout (in units of 10ms seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)     /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(40, UNIT_1_25_MS)     /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

/// BlueIO device UUIDs
#define BLE_UART_UUID_BASE			BLUEIO_UUID_BASE				// Base UUID of the device
#define BLE_UART_UUID_SERVICE			BLUEIO_UUID_UART_SERVICE		// BlueIO UART service

#define BLE_UART_UUID_TX_CHAR			BLUEIO_UUID_UART_TX_CHAR		// UART Tx characteristic
#define BLE_UART_UUID_TX_CHAR_PROP		(BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_WRAUTH | BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_UART_UUID_RX_CHAR			BLUEIO_UUID_UART_RX_CHAR		// UART Rx characteristic
#define BLE_UART_UUID_RX_CHAR_PROP		(BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_UART_UUID_CONFIG_CHAR		BLUEIO_UUID_UART_CONFIG_CHAR 							// UART configuration characteristic
#define BLE_UART_UUID_CONFIG_CHAR_PROP	(BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_WRAUTH | BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN | \
											BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY ) // Property of UART confg. char.

/// Re-use the UART Rx characteristic for the Configuration ACK

// BLE I2C Service
#define BLE_I2C_UUID_BASE			BLUEIO_UUID_BASE				// Base UUID of the device
#define BLE_I2C_UUID_SERVICE			BLUEIO_UUID_I2C_SERVICE			// BlueIO I2C service

#define BLE_I2C_UUID_TX_CHAR			BLUEIO_UUID_I2C_TX_CHAR			// I2C Tx characteristic
#define BLE_I2C_UUID_TX_CHAR_PROP		(BLESVC_CHAR_PROP_WRITE | \
											BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_I2C_UUID_RX_CHAR			BLUEIO_UUID_I2C_RX_CHAR			// I2C Rx characteristic
#define BLE_I2C_UUID_RX_CHAR_PROP		(BLESVC_CHAR_PROP_READ | \
											BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_I2C_UUID_CONFIG_CHAR		BLUEIO_UUID_I2C_CONFIG_CHAR 	// I2C configuration characteristic
#define BLE_I2C_UUID_CONFIG_CHAR_PROP	(BLESVC_CHAR_PROP_WRITE | \
											BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN) // Property of I2C config. char.

// BLE SPI Service
#define BLE_SPI_UUID_BASE				BLUEIO_UUID_BASE				// Base UUID of the device
#define BLE_SPI_UUID_SERVICE			BLUEIO_UUID_SPI_SERVICE			// BlueIO SPI service

#define BLE_SPI_UUID_TX_CHAR			BLUEIO_UUID_SPI_TX_CHAR			// SPI Tx characteristic
#define BLE_SPI_UUID_TX_CHAR_PROP		(BLESVC_CHAR_PROP_WRITE | \
											BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_SPI_UUID_RX_CHAR			BLUEIO_UUID_SPI_RX_CHAR			// SPI Rx characteristic
#define BLE_SPI_UUID_RX_CHAR_PROP		(BLESVC_CHAR_PROP_READ | \
											BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN) // Property of Tx characteristic

#define BLE_SPI_UUID_CONFIG_CHAR		BLUEIO_UUID_SPI_CONFIG_CHAR 	// SPI configuration characteristic
#define BLE_SPI_UUID_CONFIG_CHAR_PROP	(BLESVC_CHAR_PROP_WRITE | \
											BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN) // Property of SPI config. char.

//#define BLE_SPI_UUID_CONFIG_CHAR			BLUEIO_UUID_SPI_CONFIG_CHAR		// SPI Configuration characteristic
//#define BLE_SPI_UUID_CONFIG_CHAR_PROP	(BLESVC_CHAR_PROP_WRITE | \
//											BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN) // Property of SPI config. char.

// I/O Control Service (GPIO ??)
#define BLE_CTRL_UUID_BASE 				BLUEIO_UUID_BASE
#define BLE_CTRL_UUID_SERVICE			BLUEIO_UUID_CTRL_SERVICE
#define BLE_CTRL_UUID_DATA_CHAR			BLUEIO_UUID_CTRL_DATACHAR
#define BLE_CTRL_UUID_DATA_CHAR_PROP 	(BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN)
#define BLE_CTRL_UUID_CMD_CHAR			BLUEIO_UUID_CTRL_CMDCHAR
#define BLE_CTRL_UUID_CMD_CHAR_PROP		(BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN)

#define MAX_COUNT 	5

#define BLE_MTU_SIZE			512//byte
#define PACKET_SIZE				(BLE_MTU_SIZE)

// UART
#define UART_MAX_DATA_LEN  		(PACKET_SIZE)
#define UARTFIFOSIZE			CFIFO_MEMSIZE(UART_MAX_DATA_LEN)

// I2C
#define I2C_MAX_DATA_LEN		(PACKET_SIZE)
#define I2CFIFOSIZE				CFIFO_MEMSIZE(I2C_MAX_DATA_LEN * 4)
#define I2C_SLAVE_ADDR			0x22

// SPI
#define SPI_MAX_DATA_LEN		(PACKET_SIZE)
#define SPIFIFOSIZE 			CFIFO_MEMSIZE(SPI_MAX_DATA_LEN * 4)

// BLE Interface buffer
#define BLEINTRF_PKTSIZE		(BLE_MTU_SIZE)
#define BLEINTRF_FIFOSIZE		BLEINTRF_CFIFO_TOTAL_MEMSIZE(15, BLEINTRF_PKTSIZE)//(NbPkt, PktSize)

#define BLESRV_READ_CHAR_IDX		0
#define BLESRV_WRITE_CHAR_IDX		1
#define BLESRV_CONFIG_CHAR_IDX		2


//// BLE Interface event handler
//int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);
//int BleCfgIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);
//// Button event handler
//void ButEvent(int IntNo, void *pCtx);


#endif // __BOARD_H__
