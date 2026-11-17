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

// External momentary switch between GPIO3 (header label 3) and GND.
// Release the switch before sleep; press it to restart from Deep-sleep.
#define WAKE_BUT_PORT 0
#define WAKE_BUT_PIN 3
#define WAKE_BUT_PINOP IOPINOP_GPIO
#define POWER_DEMO_RESUME_WAKE 0
#define POWER_DEMO_STANDBY_RAM_TEST 1
#define POWER_DEMO_UART_DMA false
#define POWER_DEMO_UART_OFF 1
// To restore the validated BOOT-button Light-sleep configuration, set
// WAKE_BUT_PIN to 9 and POWER_DEMO_RESUME_WAKE to 1.
// UART clock gating passed nine ClockDemo wakes; PowerOff passed three Deep-sleep restarts.
// C6 GPIO9 Light-sleep: repeated wake and SRAM samples passed with UART output drained.
// GPIO3 Deep-sleep: seven consecutive wake/restart cycles passed with UART output drained.

#endif // __BOARD_H__
