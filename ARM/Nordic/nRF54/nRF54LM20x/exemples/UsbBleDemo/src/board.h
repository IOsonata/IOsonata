/**-------------------------------------------------------------------------
@file	board.h

@brief	nRF54LM20 DK pin definitions for the USB CDC to BLE peripheral demo

The demo only needs the LEDs. LED4 shows the Bluetooth link.

@author	Hoang Nguyen Hoan
@date	Oct. 5, 2026

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

// Nordic nRF54LM20 DK (PCA10184)
// IOsonata LED 1..4 correspond to DK LED 0..3.
#define LED1_PORT			1
#define LED1_PIN			22
#define LED1_PINOP			0

#define LED2_PORT			1
#define LED2_PIN			25
#define LED2_PINOP			0

#define LED3_PORT			1
#define LED3_PIN			27
#define LED3_PINOP			0

#define LED4_PORT			1
#define LED4_PIN			28
#define LED4_PINOP			0

#define CONNECT_LED_PORT	LED4_PORT
#define CONNECT_LED_PIN		LED4_PIN
#define CONNECT_LED_LOGIC	1	// Active high

#define LED_PINS	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__
