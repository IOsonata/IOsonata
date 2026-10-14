/**-------------------------------------------------------------------------
@example	board.h

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

// Board, one of the names in stm32_boards.h. This STM32WBA65 project builds for
// STM32WBA65I_DK1.
#define BOARD				STM32WBA65I_DK1

#include "stm32_boards.h"

// LD6 green PD8 toggles on trigger 0, LD5 red PD9 on trigger 1
#define LED_PINS_MAP		BOARD_LED_PINS

// Timer DevNo 0 LPTIM1, 1 LPTIM2, 2 TIM16, 3 TIM17, 4 TIM2, 5 TIM3, 6 TIM4,
// 7 TIM1. An LPTIM runs from the LSI (g_McuOsc default), 32 kHz, with two
// compare triggers: 100 ms and 1 s. A TIM runs from the APB timer clock at
// 100 kHz, with up to 4 triggers: 100 ms, 1 s, 250 ms, 500 ms.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO	0
#endif
// 100 kHz on a TIM. An LPTIM takes the closest frequency it makes, 32 kHz.
#define TIMER_DEMO_FREQ		100000

// Output on the first ST-LINK virtual COM port, 115200 8N1: USART1 (DevNo 1)
// RX PA8, TX PB12, AF7, the BOARD_VCP_PINS of stm32_boards.h
#define TIMER_DEMO_UART
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			8
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN			12
#define UART_TX_PINOP		IOPINOP_FUNC7

#endif // __BOARD_H__
