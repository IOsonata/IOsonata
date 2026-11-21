/**-------------------------------------------------------------------------
@file board.h
@brief ESP32-C6-DevKitC-1 N8 UART bridge configuration.
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
#define UART_RX_PIN 17
#define UART_TX_PIN 16
#define UART_RX_PINOP ESP32_PINOP_U0RXD
#define UART_TX_PINOP ESP32_PINOP_U0TXD

#if defined(ESP32C6_CPU_160MHZ)
#define CLOCK_DEMO_XTAL_CPU 160000000U
#else
#define CLOCK_DEMO_XTAL_CPU 40000000U
#endif

// Core source sequence and accepted frequency ranges. RC_FAST is calibrated.
// Internal sources only; no assumption about an onboard 32 kHz crystal.
#ifndef CLOCK_DEMO_UART_RX_TEST
#endif

// ClockDemo never enters sleep; BOOT wake is tested by PowerOffDemo.

#endif // __BOARD_H__
