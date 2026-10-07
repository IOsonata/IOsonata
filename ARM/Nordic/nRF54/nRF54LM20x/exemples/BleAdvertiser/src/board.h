/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

// Nordic nRF54LM20 DK (PCA10184)
// IOsonata button/LED 1..4 correspond to DK button/LED 0..3.

#ifndef UART_DEVNO
#define UART_DEVNO			1
#endif

// Buttons, active low
#define BUT1_PORT		1
#define BUT1_PIN		26
#define BUT1_PINOP		0

#define BUT2_PORT		1
#define BUT2_PIN		9
#define BUT2_PINOP		0

#define BUT3_PORT		1
#define BUT3_PIN		8
#define BUT3_PINOP		0

#define BUT4_PORT		0
#define BUT4_PIN		5
#define BUT4_PINOP		0

// LEDs, active high
#define LED1_PORT		1
#define LED1_PIN		22
#define LED1_PINOP		0

#define LED2_PORT		1
#define LED2_PIN		25
#define LED2_PINOP		0

#define LED3_PORT		1
#define LED3_PIN		27
#define LED3_PINOP		0

#define LED4_PORT		1
#define LED4_PIN		28
#define LED4_PINOP		0

// Debugger serial port 0: UARTE30 on P0.
#if UART_DEVNO == 0

#define NRFX_UART_INST	30

#define UART_RX_PORT		0
#define UART_RX_PIN			7
#define UART_RX_PINOP		1

#define UART_TX_PORT		0
#define UART_TX_PIN			6
#define UART_TX_PINOP		1

#define UART_CTS_PORT		0
#define UART_CTS_PIN		9
#define UART_CTS_PINOP		1

#define UART_RTS_PORT		0
#define UART_RTS_PIN		8
#define UART_RTS_PINOP		1

// Debugger serial port 1: UARTE20 on P1.
#elif UART_DEVNO == 1

#define NRFX_UART_INST	20

#define UART_RX_PORT		1
#define UART_RX_PIN			17
#define UART_RX_PINOP		1

#define UART_TX_PORT		1
#define UART_TX_PIN			16
#define UART_TX_PINOP		1

#define UART_CTS_PORT		1
#define UART_CTS_PIN		19
#define UART_CTS_PINOP		1

#define UART_RTS_PORT		1
#define UART_RTS_PIN		18
#define UART_RTS_PINOP		1

#else
#error "Select UART_DEVNO 0 or 1 for a DevKit virtual serial port"
#endif

#define BUTTON_PINS		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BUT3_PORT, BUT3_PIN, BUT3_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BUT4_PORT, BUT4_PIN, BUT4_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// The example runs the UART without flow control: RX and TX only.
#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define TIMER_DEVNO			2

#endif // __BOARD_H__
