/**-------------------------------------------------------------------------
@file	board.h
@brief	Onboard SPI RGB LED connections for the ESP32 devkit.
@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "esp32xx_spi.h"
#include "esp32xx_uart.h"

// The onboard addressable RGB LED data input is GPIO8.
#define LED_DEMO_SPI_ENABLE 1
#define LED_SPI_DEVNO 0
#define LED_SPI_RATE 3333333
#define LED_SPI_PINS { \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 8, ESP32_PINOP_SPI2_MOSI, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// UART progress output can be disabled after hardware validation.
#define LED_DEMO_TRACE 1
#define UART_BAUDRATE 1000000
// *** ESP32-C3-DevKitM-1
#define UART_DEVNO          0

#define UART_RX_PORT        0
#define UART_RX_PIN         20
#define UART_RX_PINOP       ESP32_PINOP_U0RXD

#define UART_TX_PORT        0
#define UART_TX_PIN         21
#define UART_TX_PINOP       ESP32_PINOP_U0TXD

#define UART_CTS_PORT       -1
#define UART_CTS_PIN        -1
#define UART_CTS_PINOP      IOPINOP_GPIO

#define UART_RTS_PORT       -1
#define UART_RTS_PIN        -1
#define UART_RTS_PINOP      IOPINOP_GPIO

#define UART_PINS           { \
	{ UART_RX_PORT,  UART_RX_PIN,  UART_RX_PINOP,  IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_TX_PORT,  UART_TX_PIN,  UART_TX_PINOP,  IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT,  IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
}

#define UART_FLOWCTRL       UART_FLWCTRL_NONE


#endif // __BOARD_H__
