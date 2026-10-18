// STM32WBA65I-DK1: USART1 on the ST-LINK virtual COM port.
#ifndef __BOARD_H__
#define __BOARD_H__

#define BOARD STM32WBA65I_DK1
#include "stm32_boards.h"

#define UART_DEVNO BOARD_VCP_DEVNO

#define UART_RX_PORT IOPORTA
#define UART_RX_PIN 8
#define UART_RX_PINOP IOPINOP_FUNC7
#define UART_TX_PORT IOPORTB
#define UART_TX_PIN 12
#define UART_TX_PINOP IOPINOP_FUNC7
#define UART_CTS_PORT -1
#define UART_CTS_PIN -1
#define UART_CTS_PINOP 0
#define UART_RTS_PORT -1
#define UART_RTS_PIN -1
#define UART_RTS_PINOP 0

#endif
