/**-------------------------------------------------------------------------
@file	board.h

@brief	Board pin map of the LPC11U35 DfuBoot example

The UART pins of the UartPrbsTxTest example, and the DfuBoot buffer
sizes: UART0 on P0.18 (RXD) and
P0.19 (TXD), CTS on P0.7, RTS on P0.17, pin function 1.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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

// SMP request and UART FIFO sizes, the defaults: the boot script places the
// SMP buffers, the SMP server and the RX FIFO in the USB SRAM, since even
// 256 and 64 leave only 1.2 KB of stack in SRAM0. Smaller values also fit
// there, and hosts then send requests of at most DFU_BOOT_BUFSIZE bytes.
#define DFU_BOOT_BUFSIZE			512
#define DFU_BOOT_FIFODEPTH			256

#define UART_DEVNO					0
#define UART_RX_PORT				0
#define UART_RX_PIN					18
#define UART_RX_PINOP				1
#define UART_TX_PORT				0
#define UART_TX_PIN					19
#define UART_TX_PINOP				1
#define UART_CTS_PORT				0
#define UART_CTS_PIN				7
#define UART_CTS_PINOP				1
#define UART_RTS_PORT				0
#define UART_RTS_PIN				17
#define UART_RTS_PINOP				1

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}



#endif
