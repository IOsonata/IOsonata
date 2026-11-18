/**-------------------------------------------------------------------------
@file	uart_sam4l.h

@brief SAM4L PICOUART wake support.

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#ifndef __UART_SAM4L_H__
#define __UART_SAM4L_H__

#include "coredev/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Sleep with USART0 RX on PB00 monitored by PICOUART at 9600 baud, 8-N-1.
 * The normal USART baud rate is preserved; only the sleep receiver uses 9600.
 * CLK32 must already run at 32768 Hz. Call from thread context with interrupts
 * enabled, TX drained and RX empty. Stop other peripherals before sleeping.
 * The sender sends one byte, then waits for the application acknowledgement.
 * Continuous reception during clock restoration is not supported.
 * Returns true when a fresh PICOUART wake byte was placed in the UART RX FIFO.
 * Returns false for an unavailable configuration, busy UART or another wake.
 * The RX callback, if installed, runs in the caller context after restoration.
 * Does not configure Backup wake or change the selected low-frequency source.
 */
bool Sam4lUartSleep(UARTDev_t *pDev);

#ifdef __cplusplus
}
#endif

#endif
