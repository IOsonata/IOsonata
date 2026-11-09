/**-------------------------------------------------------------------------
@file board.h
@brief ESP32C5 external UART bridge on GPIO4 TX / GPIO5 RX.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "esp32xx_uart.h"

#define UART_DEVNO 0
#define UART_BAUDRATE 1000000
#define UART_INT_MODE true
#define UART_DMA_MODE false
#define UART_PINS { \
	{0, 5, ESP32_PINOP_U0RXD, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 4, ESP32_PINOP_U0TXD, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif
