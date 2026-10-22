/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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

// Board, one of the names in stm32_boards.h. This STM32G474 project builds for
// NUCLEO_G474RE.
#define BOARD				NUCLEO_G474RE

#include "stm32_boards.h"

// PWM of the demo on TIM2, timer DevNo 6: channel 0 is TIM2_CH1 on LD2 (PA5),
// channel 1 TIM2_CH2 on Arduino A1 (PA1), both AF1
#define PWM_DEVNO			6
#define PWM_CHAN_CFG		\
	{ .Chan = 0, .Pol = PWM_POL_HIGH, .Port = IOPORTA, .Pin = 5, .PinOp = IOPINOP_FUNC1 }, \
	{ .Chan = 1, .Pol = PWM_POL_HIGH, .Port = IOPORTA, .Pin = 1, .PinOp = IOPINOP_FUNC1 },

#endif // __BOARD_H__
