/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, UART over BLE on the nRF9160 DK or the
Nordic Thingy:91

The nRF9160 has no Bluetooth radio. Bluetooth goes through the nRF52840 of
the board running the HciController firmware, over a UART between the two.
The console is the nRF9160 UART0, the SysLog output.

On the nRF9160 DK the console is the interface MCU VCOM0 port. On the
Thingy:91 the nRF52840 is also the USB bridge of the nRF9160 UARTs: with
HciController in place of the Connectivity Bridge firmware, the console
reaches USB only if the nRF52840 firmware forwards it.

@author	Hoang Nguyen Hoan
@date	Oct. 4, 2026

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

// Board selection, define one:
//	NRF9160_DK		nRF9160 DK
//	NORDIC_THINGY91	Nordic Thingy:91
#define NRF9160_DK
//#define NORDIC_THINGY91

#if defined(NORDIC_THINGY91)

// The one button, active low
#define BUT1_PORT		0
#define BUT1_PIN		26
#define BUT1_PINOP		0

// Lightwell RGB LED, red, green and blue, active high
#define LED1_PORT		0
#define LED1_PIN		29
#define LED1_PINOP		0

#define LED2_PORT		0
#define LED2_PIN		30
#define LED2_PINOP		0

#define LED3_PORT		0
#define LED3_PIN		31
#define LED3_PINOP		0

#define CONNECT_LED_PORT	LED3_PORT
#define CONNECT_LED_PIN		LED3_PIN
#define CONNECT_LED_LOGIC	1

// Console, UART0 to UART0 of the nRF52840
#define UART_RX_PORT		0
#define UART_RX_PIN			19
#define UART_RX_PINOP		1

#define UART_TX_PORT		0
#define UART_TX_PIN			18
#define UART_TX_PINOP		1

#define UART_CTS_PORT		0
#define UART_CTS_PIN		21
#define UART_CTS_PINOP		1

#define UART_RTS_PORT		0
#define UART_RTS_PIN		20
#define UART_RTS_PINOP		1

// HCI UART to the nRF52840, the lines Nordic's board file gives UART1: nRF9160
// TX P0.22, RX P0.23, RTS P0.24, CTS P0.25 to nRF52840 RX P1.00, TX P0.25,
// CTS P0.19, RTS P0.22. P0.10, active low, is the reset of the nRF52840.
#define HCI_UART_DEVNO		2
#define HCI_UART_RATE		1000000

#define HCI_UART_RX_PORT	0
#define HCI_UART_RX_PIN		23
#define HCI_UART_RX_PINOP	1

#define HCI_UART_TX_PORT	0
#define HCI_UART_TX_PIN		22
#define HCI_UART_TX_PINOP	1

#define HCI_UART_CTS_PORT	0
#define HCI_UART_CTS_PIN	25
#define HCI_UART_CTS_PINOP	1

#define HCI_UART_RTS_PORT	0
#define HCI_UART_RTS_PIN	24
#define HCI_UART_RTS_PINOP	1

#define BUTTON_PINS		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define LED_PINS	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#elif defined(NRF9160_DK)

#define BUT1_PORT		0
#define BUT1_PIN		6
#define BUT1_PINOP		0

#define BUT2_PORT		0
#define BUT2_PIN		7
#define BUT2_PINOP		0

#define LED1_PORT		0
#define LED1_PIN		2
#define LED1_PINOP		0

#define LED2_PORT		0
#define LED2_PIN		3
#define LED2_PINOP		0

#define LED3_PORT		0
#define LED3_PIN		4
#define LED3_PINOP		0

#define LED4_PORT		0
#define LED4_PIN		5
#define LED4_PINOP		0

#define CONNECT_LED_PORT	LED4_PORT
#define CONNECT_LED_PIN		LED4_PIN
#define CONNECT_LED_LOGIC	1

// Console, UART0 to the interface MCU (VCOM0)
#define UART_RX_PORT		0
#define UART_RX_PIN			28
#define UART_RX_PINOP		1

#define UART_TX_PORT		0
#define UART_TX_PIN			29
#define UART_TX_PINOP		1

#define UART_CTS_PORT		0
#define UART_CTS_PIN		26
#define UART_CTS_PINOP		1

#define UART_RTS_PORT		0
#define UART_RTS_PIN		27
#define UART_RTS_PINOP		1

// HCI UART to the nRF52840 running HciController, on the DK interface lines
// 0 to 3 (nRF9160 P0.17, P0.18, P0.19, P0.21). These are the pins Nordic's
// LTE/BLE gateway sample uses for the same link; check them against the
// board revision. The nRF52840 has to route the interface lines and use the
// matching pins, its TX on line 0 and its RTS on line 2.
#define HCI_UART_DEVNO		2
#define HCI_UART_RATE		1000000

#define HCI_UART_RX_PORT	0
#define HCI_UART_RX_PIN		17
#define HCI_UART_RX_PINOP	1

#define HCI_UART_TX_PORT	0
#define HCI_UART_TX_PIN		18
#define HCI_UART_TX_PINOP	1

#define HCI_UART_CTS_PORT	0
#define HCI_UART_CTS_PIN	19
#define HCI_UART_CTS_PINOP	1

#define HCI_UART_RTS_PORT	0
#define HCI_UART_RTS_PIN	21
#define HCI_UART_RTS_PINOP	1

#define BUTTON_PINS		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define LED_PINS	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#else
#error "Select the board: NRF9160_DK or NORDIC_THINGY91"
#endif

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#define HCI_UART_PINS		{ \
	{HCI_UART_RX_PORT, HCI_UART_RX_PIN, HCI_UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{HCI_UART_TX_PORT, HCI_UART_TX_PIN, HCI_UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{HCI_UART_CTS_PORT, HCI_UART_CTS_PIN, HCI_UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{HCI_UART_RTS_PORT, HCI_UART_RTS_PIN, HCI_UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#endif
