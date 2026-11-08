/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, PeiAdcDemo on the nRF52840 DK

The console is UART0 on the interface MCU VCOM0 port of the nRF52840 DK, the
SysLog output. The analog input is AIN0, P0.02 on the P1 header: open it reads
the supply with the SAADC pull-up, connected to GND 0 V. The rounds are started
by TIMER3, Timer DevNo 6.

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

// Console, UART0 on the VCOM0 port of the interface MCU
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			6
#define UART_TX_PINOP		1

// Analog input number, AIN0 is P0.02
#define PEI_ADC_DEMO_AIN			0

// Timer of the rounds, TIMER3
#define PEI_ADC_DEMO_TIMER_DEVNO	6

#endif // __BOARD_H__
