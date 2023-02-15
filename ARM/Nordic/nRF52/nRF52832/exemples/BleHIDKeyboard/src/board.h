/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.
Typically, the hardware used in this example is BlueIO_TAG_EVIM, i.e., I-SYST CS-BLYST-07.
This module can be found here:
https://www.i-syst.com/products/blyst-nano


@author	Duy Thinh Tran
@date	Feb. 15, 2023

@license

Copyright (c) 2023, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : info at i-syst dot com

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

#include "blueio_board.h"
#include "coredev/iopincfg.h"

/** Hardware selection */
//#define NORDIC_DK
#define BLYST832

/** Print debug data via UART **/
#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif

#ifdef NORDIC_DK
// Nordic DK PCA10040 board

#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1	//
#define UART_TX_PORT		0
#define UART_TX_PIN			6//7
#define UART_TX_PINOP		0
#define UART_CTS_PORT		0
#define UART_CTS_PIN		7//12
#define UART_CTS_PINOP		0
#define UART_RTS_PORT		0
#define UART_RTS_PIN		5//11
#define UART_RTS_PINOP		0

#define BUTTON1_PORT		0
#define BUTTON1_PIN			13
#define BUTTON2_PORT		0
#define BUTTON2_PIN			14
#define BUTTON3_PORT		0
#define BUTTON3_PIN			15
#define BUTTON4_PORT		0
#define BUTTON4_PIN			16

#else //Blyst832
// BlueIO breakout board
#define UART_RX_PORT		BLUEIO_UART_RX_PORT
#define UART_RX_PIN			BLUEIO_UART_RX_PIN
#define UART_RX_PINOP		BLUEIO_UART_RX_PINOP

#define UART_TX_PORT		BLUEIO_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_UART_TX_PINOP

#define UART_CTS_PORT		BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN		BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP		BLUEIO_UART_CTS_PINOP

#define UART_RTS_PORT		BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN		BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP		BLUEIO_UART_RTS_PINOP

#define BUTTON1_PORT		BLUEIO_BUT1_PORT
#define BUTTON1_PIN			BLUEIO_BUT1_PIN

#define BUTTON2_PORT		BLUEIO_BUT2_PORT
#define BUTTON2_PIN			BLUEIO_BUT2_PIN

#define LED1_PORT		BLUEIO_LED1_PORT
#define LED1_PIN		BLUEIO_LED1_PIN
#define LED1_PINOP		BLUEIO_LED1_PINOP

#define LED2_PORT		BLUEIO_LED2_PORT
#define LED2_PIN		BLUEIO_LED2_PIN
#define LED2_PINOP		BLUEIO_LED2_PINOP

#define LED3_PORT		BLUEIO_LED3_PORT
#define LED3_PIN		BLUEIO_LED3_PIN
#define LED3_PINOP		BLUEIO_LED3_PINOP

#define LED4_PORT		BLUEIO_LED4_PORT
#define LED4_PIN		BLUEIO_LED4_PIN
#define LED4_PINOP		BLUEIO_LED4_PINOP

#endif


typedef enum {
	RTC0_LFCLK = 0,
	RTC1_LFCLK,
	RTC2_LFCLK,
	TIMER0_HFCLK,
	TIMER1_HFCLK,
	TIMER2_HFCLK,
	TIMER3_HFCLK,
	TIMER4_HFCLK
} TIMER_SOURCES;


int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#endif // __BOARD_H__

