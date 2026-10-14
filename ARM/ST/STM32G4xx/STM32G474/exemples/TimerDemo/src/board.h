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

// Board, one of the names in stm32_boards.h. This STM32G474 project builds for
// NUCLEO_G474RE.
#define BOARD				NUCLEO_G474RE

#include "stm32_boards.h"

// LD2 green PA5 toggles on trigger 0
#define LED_PINS_MAP		BOARD_LED_PINS

// Timer DevNo 0 LPTIM1, 1 TIM6, 2 TIM7, 3 TIM16, 4 TIM17, 5 TIM15, 6 TIM2,
// 7 TIM3, 8 TIM4, 9 TIM5, 10 TIM1, 11 TIM8, 12 TIM20. LPTIM1 runs from the
// LSE (g_McuOsc default) at the highest frequency, 32768 Hz, one compare
// trigger: 100 ms. A TIM runs from the APB timer clock at 100 kHz, with up
// to 4 triggers: 100 ms, 1 s, 250 ms, 500 ms.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO	0
#endif
// 100 kHz on a TIM: the trigger of the basic timers TIM6 and TIM7 is one
// counter period, 65536 ticks at most. An LPTIM takes the closest frequency
// it makes, 32768 Hz.
#define TIMER_DEMO_FREQ		100000

// Output on the ST-LINK virtual COM port, 115200 8N1: LPUART1 (DevNo 0) RX
// PA3, TX PA2, AF12, the BOARD_VCP_PINS of stm32_boards.h
#define TIMER_DEMO_UART
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			3
#define UART_RX_PINOP		IOPINOP_FUNC12
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			2
#define UART_TX_PINOP		IOPINOP_FUNC12

#endif
