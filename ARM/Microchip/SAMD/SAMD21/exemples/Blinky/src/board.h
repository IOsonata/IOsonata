/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro LED and button

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The pins are those of the SAM D21 Xplained Pro.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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

// LED0 on PB30, active low
#define LED_PINS_MAP		{ \
	{ IOPORTB, 30, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL } \
}

// SW0 on PA15, active low, EXTINT 15 of the EIC on the press
#define BUTTON_PINS_MAP		{ \
	{ IOPORTA, 15, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL } \
}
#define BUT1_INT			15
#define BUT1_INT_PRIO		3
#define BUT1_SENSE			IOPINSENSE_LOW_TRANSITION

#endif // __BOARD_H__
