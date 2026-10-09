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

// Example wiring for a SAM4LS C-package application, not a supplied board.
// Connect an LED with a series resistor from VDD to PC07, or change this pin.
// The library defaults to internal oscillators; define MCUOSC for your board.
#define LED1_PORT IOPORTC
#define LED1_PIN 7
#define LED1_PINOP IOPINOP_GPIO

#define LED_PINS_MAP { \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Connect a 3.3 V UART adapter: RX to PC27, TX to PC26, and common ground.
#define TIMER_DEMO_UART
#define UART_DEVNO 1
#define UART_RX_PORT IOPORTC
#define UART_RX_PIN 26
#define UART_RX_PINOP IOPINOP_PERIPHA
#define UART_TX_PORT IOPORTC
#define UART_TX_PIN 27
#define UART_TX_PINOP IOPINOP_PERIPHA

// 0 = AST; 1..3 = TC0 channels 0..2; 4..6 = TC1 channels 0..2.
#define TIMER_DEVNO 0
#define TIMER_FREQ 0

#endif // __BOARD_H__
