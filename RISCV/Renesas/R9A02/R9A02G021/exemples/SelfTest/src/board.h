/**-------------------------------------------------------------------------
@file	board.h

@brief	R9A02G021 hardware self-test on the FPB-R9A02G021

Wiring (SELFTEST_DAC_EXT_DIAG true):
	P001 (DACOUT1) to P400 (ANI2), P401 to 3.3 V, P106 disconnected.
Normal self-test wiring (set diagnostic mode false):
	P001 (DACOUT1) to P106 (IVCMP0, ANI19)	tests 9, 10 and 12
	A0 (P400) to GND, A1 (P401) to 3.3 V	tests 8 and 11
Nothing on P101 to P104 (SPI20, test 6, MISO pulled up).

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

// The library defaults are needed: HOCO 48 MHz core clock and PCLKB, LOCO
// low frequency clock. Tests 2 and 5 compare the TML32 on the LOCO with the
// machine timer, which always counts the LOCO; tests 3 and 4 compare the
// TML32 on the HOCO with mcycle.

// FPB-R9A02G021 LED1 P107 and LED2 P100, active high
#define LED1_PORT			1
#define LED1_PIN			7
#define LED1_PINOP			IOPINOP_GPIO

#define LED2_PORT			1
#define LED2_PIN			0
#define LED2_PINOP			IOPINOP_GPIO

#define LED_PINS_MAP		{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// SAU UART0 on the on-board J-Link OB virtual COM port, R31 and R33 fitted
// and E56 closed (board defaults):
// TxD0_B P302, RxD0_B P303. PSEL 0x11 (SAU) is IOPINOP_FUNC16.
// Configure RX before TX in the pin map.
#define UART_DEVNO			0
#define UART_RX_PORT		3
#define UART_RX_PIN			3
#define UART_RX_PINOP		IOPINOP_FUNC16

#define UART_TX_PORT		3
#define UART_TX_PIN			2
#define UART_TX_PINOP		IOPINOP_FUNC16

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define UART_BAUDRATE		115200

// SPI20, SAU1 channel 0: SCK20 P101, SI20 P102 (MISO, pulled up), SO20 P103,
// chip select P104. Nothing connected: every received byte is 0xFF.
#define SELFTEST_SPI_DEVNO	4
#define SELFTEST_SPI_PINS	{ \
	{1, 1, IOPINOP_FUNC16, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{1, 2, IOPINOP_FUNC16, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{1, 3, IOPINOP_FUNC16, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{1, 4, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// VCC of the board in mV, the reference of the A/D and D/A converters
#define SELFTEST_VCC_MV		3300

// Diagnostic mode: P001 DACOUT1 to P400 ANI2; normal self-test when false.
#define SELFTEST_DAC_EXT_DIAG	true

// Test 12 asks to touch P106 to GND while the MCU is in Software Standby,
// false to leave it out
#define SELFTEST_CMP_WAKE	true

#endif
