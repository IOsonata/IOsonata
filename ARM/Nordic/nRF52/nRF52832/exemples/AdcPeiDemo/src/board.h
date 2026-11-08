/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, AdcPeiDemo on the BLYST-NANO-DK

The console is UART0 on the BLYST-NANO-DK UART pins, the SysLog output. The
analog input is AIN0, P0.02, which is BUT1: released it reads the supply with
the SAADC pull-up, pressed 0 V. The rounds are started by TIMER3, Timer DevNo 6.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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
#include "blystnano_boards.h"

// Console, UART0, the SysLog output
#define UART_DEVNO			0
#define UART_RX_PORT		BLYST_NANO_DK_UART_RX_PORT
#define UART_RX_PIN			BLYST_NANO_DK_UART_RX_PIN
#define UART_RX_PINOP		BLYST_NANO_DK_UART_RX_PINOP
#define UART_TX_PORT		BLYST_NANO_DK_UART_TX_PORT
#define UART_TX_PIN			BLYST_NANO_DK_UART_TX_PIN
#define UART_TX_PINOP		BLYST_NANO_DK_UART_TX_PINOP

// Analog input number, AIN0 is P0.02, BUT1
#define ADC_PEI_DEMO_AIN			0

// Timer of the rounds, TIMER3
#define ADC_PEI_DEMO_TIMER_DEVNO	6

#endif // __BOARD_H__
