/**-------------------------------------------------------------------------
@file board.h
@brief ESP32-C3-DevKitM-1 UART bridge configuration.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"
#include "esp32xx_uart.h"

#define UART_DEVNO 0
#define UART_BAUDRATE 1000000
#define UART_RX_PORT 0
#define UART_TX_PORT 0
#define UART_RX_PIN 20
#define UART_TX_PIN 21
#define UART_RX_PINOP ESP32_PINOP_U0RXD
#define UART_TX_PINOP ESP32_PINOP_U0TXD

#if defined(ESP32C3_CPU_160MHZ)
#define CLOCK_DEMO_XTAL_CPU 160000000U
#else
#define CLOCK_DEMO_XTAL_CPU 40000000U
#endif

// Core source sequence and accepted frequency ranges. RC_FAST is measured against XTAL.
#define CLOCK_DEMO_SOURCES { \
	{"XTAL", OSC_TYPE_XTAL, 40000000U, CLOCK_DEMO_XTAL_CPU, CLOCK_DEMO_XTAL_CPU}, \
	{"RC_FAST", OSC_TYPE_RC, 0U, 10000000U, 30000000U}, \
	{"XTAL restore", OSC_TYPE_XTAL, 40000000U, CLOCK_DEMO_XTAL_CPU, CLOCK_DEMO_XTAL_CPU} \
}
// C3 divides CPU and APB together outside PLL mode.
#if defined(ESP32C3_CPU_160MHZ)
// APB is fixed at 80 MHz when PLL feeds the CPU.
#define CLOCK_DEMO_PERIPH_DIVIDERS {1U}
#else
#define CLOCK_DEMO_PERIPH_DIVIDERS {1U, 2U, 4U, 1U}
#endif
// Internal sources only; no assumption about an onboard 32 kHz crystal.
#define CLOCK_DEMO_LF_SOURCES { \
	{"RC_SLOW", OSC_TYPE_RC, 136000U} \
}
#ifndef CLOCK_DEMO_UART_RX_TEST
#endif

// ClockDemo never enters sleep; BOOT wake is tested by PowerOffDemo.

#endif // __BOARD_H__
