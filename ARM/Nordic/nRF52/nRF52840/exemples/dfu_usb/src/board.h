/*--------------------------------------------------------------------------
File   : custom_board.h

Author : ThinhTran          Dec. 4, 2016

Desc   : Custom board for Nordic SDK bsp compilation.
		 Contains definitions for BlueIO series boards for the IMM-NRF5x
		 series modules

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

//#include "nrf.h"
//#include "blueio_board.h"
#include "coredev/iopincfg.h"

/******** IBK_NRF52840 board **************************/
// Buttons
#define BUT1_PORT		0
#define BUT1_PIN		13
#define BUT1_PINOP		0

#define BUT2_PORT		0
#define BUT2_PIN		4
#define BUT2_PINOP		0

#define BUTTON_PINS_MAP		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define BUT1_SENSE			IOPINSENSE_LOW_TRANSITION
#define BUT2_SENSE			IOPINSENSE_LOW_TRANSITION

#define BUT1_INT		0
#define BUT1_INT_PRIO	6

#define BUT2_INT		1
#define BUT2_INT_PRIO	6

// LEDs
#define LED1_PORT		0
#define LED1_PIN		30
#define LED1_PINOP		0 // logic low

#define LED2_PORT		1
#define LED2_PIN		8
#define LED2_PINOP		1 // logic high

#define LED3_PORT		1
#define LED3_PIN		9
#define LED3_PINOP		1 // logic high

#define LED_PINS_MAP		IBK_NRF52840_LED_PINS_CFG

// UART
#define UART_TX_PORT		0
#define UART_TX_PIN			7
#define UART_TX_PINOP		0

#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		0

#define UART_CTS_PORT		0
#define UART_CTS_PIN		12
#define UART_CTS_PINOP		0

#define UART_RTS_PORT		0
#define UART_RTS_PIN		11
#define UART_RTS_PINOP		0

#define UART_PINS_MAP		{\
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	\
}


#if 0
#define LEDS_NUMBER    3

#define LED_START      BLUEIO_LED3_PIN
#define LED_1          BLUEIO_LED3_PIN
#define LED_2          BLUEIO_LED2_PIN
#define LED_3          BLUEIO_LED1_PIN
#define LED_STOP       BLUEIO_LED1_PIN

#define LEDS_ACTIVE_STATE 0

#define LEDS_INV_MASK  LEDS_MASK

#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3

#define BUTTONS_NUMBER 2

#define BUTTON_START   BLUEIO_BUT1_PIN
#define BUTTON_1       BLUEIO_BUT1_PIN
#define BUTTON_2       BLUEIO_BUT2_PIN
#define BUTTON_STOP    BLUEIO_BUT2_PIN
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_1
#define BSP_BUTTON_2   BUTTON_2		// Remap for Nordic DFU
//#define BSP_BUTTON_3   BUTTON_3		// Remap for Nordic DFU

#define RX_PIN_NUMBER		BLUEIO_UART_RX_PIN
#define TX_PIN_NUMBER		BLUEIO_UART_TX_PIN
#define RTS_PIN_NUMBER		BLUEIO_UART_RTS_PIN
#define CTS_PIN_NUMBER		BLUEIO_UART_CTS_PIN

// Low frequency clock source to be used by the SoftDevice
// I don't like this kind of define to set the default data
// but it is used by Nordic SDK
#ifdef IMM_NRF51822
// Does not have crystal, use internal RC
#define NRF_CLOCK_LFCLKSRC      { NRF_CLOCK_LF_SRC_RC, 1, 1, 0 }
#else
// IMM-NRF51422 & IMM-NRF52832 have 32KHz crystal
#define NRF_CLOCK_LFCLKSRC      { NRF_CLOCK_LF_SRC_XTAL, 0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM }
#endif
#endif // original configuration
#endif // __BOARD_H__
