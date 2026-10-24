#ifndef __BOARD_H__
#define __BOARD_H__

#include "blueio_board.h"

// Console pin mapping follows the nRF52832 UART examples.
#define UART_DEVNO		0
#define UART_RX_PORT	BLUEIO_UART_RX_PORT
#define UART_RX_PIN		BLUEIO_UART_RX_PIN
#define UART_RX_PINOP	BLUEIO_UART_RX_PINOP
#define UART_TX_PORT	BLUEIO_UART_TX_PORT
#define UART_TX_PIN		BLUEIO_UART_TX_PIN
#define UART_TX_PINOP	BLUEIO_UART_TX_PINOP

#endif
