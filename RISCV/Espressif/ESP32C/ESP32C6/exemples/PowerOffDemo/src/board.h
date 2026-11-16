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

// BOOT button. Release it before resetting the board.
#define WAKE_BUT_PORT 0
#define WAKE_BUT_PIN 9
#define WAKE_BUT_PINOP IOPINOP_GPIO
#define POWER_DEMO_RESUME_WAKE 1
#define POWER_DEMO_STANDBY_RAM_TEST 1
#define POWER_DEMO_UART_DMA false

// SystemSleep uses CPU WFI with peripheral clocks and SRAM retained.
// This project does not enter PMU Light-sleep or Deep-sleep.

#endif // __BOARD_H__
