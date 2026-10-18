#ifndef __BOARD_H__
#define __BOARD_H__
#define BOARD NUCLEO_L152RE
#include "stm32_boards.h"
#define UART_DEVNO BOARD_VCP_DEVNO
#define UART_PINS BOARD_VCP_PINS
#define UART_INT_MODE true
#define UART_DMA_MODE false
#define LED_PINS_MAP BOARD_LED_PINS
// DevNo 5 selects TIM2 with four triggers.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO 5
#endif
#define TIMER_DEMO_FREQ 100000
#define TIMER_DEMO_UART
#define UART_RX_PORT IOPORTA
#define UART_RX_PIN 3
#define UART_RX_PINOP IOPINOP_FUNC7
#define UART_TX_PORT IOPORTA
#define UART_TX_PIN 2
#define UART_TX_PINOP IOPINOP_FUNC7
#endif
