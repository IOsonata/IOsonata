/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro GPIO pulse train

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

// EXT1 pins 3 to 10: PB00, PB01, PB06, PB07, PB02, PB03, PB04, PB05
#define PULSE_TRAIN_PINS_MAP { \
	{IOPORTB, 0, IOPINOP_GPIO}, {IOPORTB, 1, IOPINOP_GPIO}, {IOPORTB, 6, IOPINOP_GPIO}, \
	{IOPORTB, 7, IOPINOP_GPIO}, {IOPORTB, 2, IOPINOP_GPIO}, {IOPORTB, 3, IOPINOP_GPIO}, \
	{IOPORTB, 4, IOPINOP_GPIO}, {IOPORTB, 5, IOPINOP_GPIO}, \
}

#endif // __BOARD_H__
