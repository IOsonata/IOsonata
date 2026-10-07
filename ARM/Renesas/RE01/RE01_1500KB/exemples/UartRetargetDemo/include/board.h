// SPDX-License-Identifier: MIT
// Sample MCU pin map; adjust to the application wiring.
#ifndef __BOARD_H__
#define __BOARD_H__

#define UART_DEVNO		4
#define UART_RX_PORT		2
#define UART_RX_PIN		2
#define UART_RX_PINOP		IOPINOP_FUNC3
#define UART_TX_PORT		2
#define UART_TX_PIN		3
#define UART_TX_PINOP		IOPINOP_FUNC3
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO

#endif
