/**-------------------------------------------------------------------------
@file	board.h
@brief	STM32WBA65I-DK1 PowerOffDemo with external PC13 wake button

Connect Arduino D4 (PC13) momentarily to GND to wake from Standby.
The VCOM is USART1 on PA8 RX and PB12 TX.
MIT License
Copyright (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

#define UART_DEVNO		1
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN		8
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN		12
#define UART_TX_PINOP		IOPINOP_FUNC7

// External button between PC13 (Arduino D4) and GND.
#define WAKE_BUT_PORT		IOPORTC
#define WAKE_BUT_PIN		13
#define WAKE_BUT_PINOP		IOPINOP_GPIO

// RAMRET is SRAM2. RAMDROP is the end of SRAM1, not .data/.bss/stack.
#define POWER_DEMO_RAM_RET_TEST	1
#define POWER_DEMO_RAM_RET_ADDR	0x20070000UL
#define POWER_DEMO_RAM_RET_SIZE	32768UL
#define POWER_DEMO_RAM_DROP_ADDR	0x20068000UL
#define POWER_DEMO_RAM_DROP_SIZE	32768UL

#endif
