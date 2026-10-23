#ifndef __BOARD_H__
#define __BOARD_H__

#define BOARD NUCLEO_F401RE
#include "stm32_boards.h"

// USART2 over the ST-LINK virtual COM port.
#define UART_DEVNO BOARD_VCP_DEVNO
#define UART_PINS BOARD_VCP_PINS

#endif
