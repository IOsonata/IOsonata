/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the BLYST-NANO-DK

The console is UART0 on the BLYST-NANO-DK UART pins, the SysLog output. The
wake up button is BUT1.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

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

// Generic PowerOffDemo settings, matching the Microchip test profiles.
// System ON wake uses the board button; change RESUME_WAKE to 0 for System OFF.
#ifndef POWER_DEMO_RESUME_WAKE
#define POWER_DEMO_RESUME_WAKE		1
#endif
#ifndef POWER_DEMO_STANDBY_RAM_TEST
#define POWER_DEMO_STANDBY_RAM_TEST	1
#endif

// UART receive wake is not configured by this nRF52 board profile.
// Leave disabled until the UART driver supports this wake source.
#ifndef POWER_DEMO_UART_WAKE
#define POWER_DEMO_UART_WAKE		0
#endif

// RTC/timer automatic wake is a separate profile, disabled for button testing.
#ifndef POWER_DEMO_TIMER_WAKE_MS
#define POWER_DEMO_TIMER_WAKE_MS	0
#endif
#define POWER_DEMO_TIMER_WAKE_DEVNO	0

// Use the same generic console and wake settings as the Microchip test.
#define UART_BAUDRATE			1000000
#define POWER_DEMO_UART_DMA	false
#define POWER_DEMO_UART_OFF		1
#define WAKE_BUT_SENSE			IOPINSENSE_LOW_TRANSITION

// Wake up button, to ground
#define WAKE_BUT_PORT		BLYST_NANO_DK_BUT1_PORT
#define WAKE_BUT_PIN		BLYST_NANO_DK_BUT1_PIN
#define WAKE_BUT_PINOP		BLYST_NANO_DK_BUT1_PINOP

#endif
