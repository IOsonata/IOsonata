#ifndef __BOARD_H__
#define __BOARD_H__
#define BOARD NUCLEO_F401RE
#include "stm32_boards.h"
#define LED_PINS_MAP BOARD_LED_PINS
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO 0
#endif
#define TIMER_DEMO_FREQ 100000
#define UART_DEVNO BOARD_VCP_DEVNO
#define UART_RX_PORT IOPORTA
#define UART_RX_PIN 3
#define UART_RX_PINOP IOPINOP_FUNC7
#define UART_TX_PORT IOPORTA
#define UART_TX_PIN 2
#define UART_TX_PINOP IOPINOP_FUNC7

// The port lacks generic runtime selectors. This project checks startup readback and timer operation only.
#define CLOCK_DEMO_SOURCES { {"Startup readback", OSC_TYPE_RC, 0U, 1U, UINT32_MAX, false} }
#define CLOCK_DEMO_APB_DIVIDERS {}
#define CLOCK_DEMO_TIMER_DEVNO TimerGetLowFreqDevCount()
#define CLOCK_DEMO_CYCLES 3U
#define CLOCK_DEMO_IRQ_PRIORITY 1
#define CLOCK_DEMO_STOP_UART 1
#define CLOCK_DEMO_DEFER_TIMER 1
#define CLOCK_DEMO_CORE_READONLY 1
#define CLOCK_DEMO_APB_READONLY 1

#endif
