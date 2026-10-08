// STM32F0308-DISCO application wiring. Change here for another board.
#ifndef __BOARD_H__
#define __BOARD_H__
#include "stm32f0xx.h"
#include "coredev/iopincfg.h"

// USART1 (virtual DevNo 0). External 3.3 V adapter; common GND, no flow control.
#define UART_DEVNO 0
#define UART_NO UART_DEVNO
#define UART_BAUDRATE 115200
#define UART_INT_MODE true
#define UART_DMA_MODE false
#define UART_INT_PRIO 2
#define UART_RX_PORT 0
#define UART_RX_PIN 10
#define UART_RX_PINOP 0x12
#define UART_TX_PORT 0
#define UART_TX_PIN 9
#define UART_TX_PINOP 0x12
#define UART_CTS_PORT -1
#define UART_CTS_PIN -1
#define UART_CTS_PINOP 0
#define UART_RTS_PORT -1
#define UART_RTS_PIN -1
#define UART_RTS_PINOP 0
#define UART_PINS { \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
#define UART_PORTPINS IOPinCfg_t s_UartPortPins[] = UART_PINS
#define UART_PORTPIN_COUNT (sizeof(s_UartPortPins) / sizeof(s_UartPortPins[0]))

#define UARTFIFOSIZE CFIFO_MEMSIZE(256)

#endif
