/**-------------------------------------------------------------------------
@example	board.h

@brief	LPC54628 power-off demo pin definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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

// Flexcomm 0, LPC-Link2 VCOM pins.
#define UART_DEVNO 0
#define UART_BAUDRATE 1000000
#define UART_RX_PORT 0
#define UART_RX_PIN 29
#define UART_RX_PINOP IOPINOP_FUNC1
#define UART_TX_PORT 0
#define UART_TX_PIN 30
#define UART_TX_PINOP IOPINOP_FUNC1

// P0.4 is sampled before entry only; it cannot wake Deep Power-down.
// Use the hardware RESET pin to restart the demo.
#define WAKE_BUT_PORT 0
#define WAKE_BUT_PIN 4
#define WAKE_BUT_PINOP IOPINOP_GPIO
#define WAKE_BUT_SENSE IOPINSENSE_LOW_TRANSITION
#define POWER_DEMO_UART_DMA false
#define POWER_DEMO_UART_OFF 1


// Internal sources only; external crystal frequency is not assumed.
#define CLOCK_DEMO_SOURCES { {"FRO", OSC_TYPE_RC, 12000000U, 1U, UINT32_MAX} }
#define CLOCK_DEMO_APB_DIVIDERS {}
#define CLOCK_DEMO_APB_READONLY 1
#define CLOCK_DEMO_TIMER_DEVNO TimerGetLowFreqDevCount()
#define CLOCK_DEMO_CYCLES 3U
#define CLOCK_DEMO_IRQ_PRIORITY 1
#define CLOCK_DEMO_STOP_UART 1
#define CLOCK_DEMO_DEFER_TIMER 1

#endif
