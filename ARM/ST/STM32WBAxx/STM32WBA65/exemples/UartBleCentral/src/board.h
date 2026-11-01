/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

@license

Copyright (c) 2026, I-SYST inc., all rights reserved

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

// Board, one of the names in stm32_boards.h. This STM32WBA65 project builds for
// STM32WBA65I_DK1.
#define BOARD				STM32WBA65I_DK1

#include "stm32_boards.h"

// Use the DK1 LSE crystal for host timers and the radio sleep timer.
// Keep the existing HSI16 core-clock source for this comparison.
#define MCUOSC { \
	{OSC_TYPE_RC, 16000000, 20, 0}, \
	{OSC_TYPE_XTAL, 32768, 500, 0}, \
	false \
}

// LD6 green PD8 and LD5 red PD9
#define LED_PINS_MAP		BOARD_LED_PINS

// TIM16 for application timing. Bluetooth uses ST controller software timers
// and does not reserve an IOsonata timer device.
#define TIMER_DEVNO			2

// Log output on the first ST-LINK virtual COM port, USART1, RX PA8, TX PB12.
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_PINS			BOARD_VCP_PINS

#define LED_PINS			BOARD_LED_PINS
#define CONNECT_LED_PORT	IOPORTD
#define CONNECT_LED_PIN		8
#define CONNECT_LED_LOGIC	BOARD_LED_ACTIVE
#define BLUEIO_CONNECT_LED_PORT	CONNECT_LED_PORT
#define BLUEIO_CONNECT_LED_PIN	CONNECT_LED_PIN

// USART1 on the first ST-LINK VCP. No hardware flow control.
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			8
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN			12
#define UART_TX_PINOP		IOPINOP_FUNC7
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO

// Optional external button: Arduino D4 / PC13 to GND, as in Blinky.
// The onboard joystick is analog; it is not a GPIO button.
#define BUT1_INT			13
#define BUTTON_PINS		{ \
	{IOPORTC, 13, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#endif
