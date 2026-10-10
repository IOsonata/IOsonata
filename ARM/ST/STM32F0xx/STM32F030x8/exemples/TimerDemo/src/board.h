// STM32F0308-DISCO application wiring (UM1658). Change here for another board.
#ifndef __BOARD_H__
#define __BOARD_H__
#include "stm32f0xx.h"

#define LED_PINS_MAP { \
	{2, 9, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{2, 8, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
// Select virtual devices 0-6 to exercise every peripheral timer.
// 10 kHz accommodates the demo's longest period even on basic TIM6.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO 0
#endif
#define TIMER_DEMO_FREQ 10000
#define TIMER_DEMO_INT_PRIO 2
// UART output requires an external 3.3 V USB-UART adapter, TX PA9 / RX PA10.
#define TIMER_DEMO_UART
#define UART_DEVNO 0
#define UART_RX_PORT 0
#define UART_RX_PIN 10
#define UART_RX_PINOP IOPINOP_FUNC1
#define UART_TX_PORT 0
#define UART_TX_PIN 9
#define UART_TX_PINOP IOPINOP_FUNC1
#endif
