/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D11 Xplained Pro GPIO pulse train

Each pin of the map pulses in turn, to check the GPIO output with a logic
analyzer.

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

// EXT1 pins 6, 8, 11, 12, 15, 16, 17, 18: PA05, PA17, PA22, PA23, PA08,
// PA06, PA09, PA07, then LED0 PA16
#define PULSE_TRAIN_PINS_MAP { \
	{IOPORTA, 5, IOPINOP_GPIO}, {IOPORTA, 17, IOPINOP_GPIO}, {IOPORTA, 22, IOPINOP_GPIO}, \
	{IOPORTA, 23, IOPINOP_GPIO}, {IOPORTA, 8, IOPINOP_GPIO}, {IOPORTA, 6, IOPINOP_GPIO}, \
	{IOPORTA, 9, IOPINOP_GPIO}, {IOPORTA, 7, IOPINOP_GPIO}, {IOPORTA, 16, IOPINOP_GPIO}, \
}

#endif // __BOARD_H__
