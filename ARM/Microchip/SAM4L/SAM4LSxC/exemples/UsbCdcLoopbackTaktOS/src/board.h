/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM4LS C-package example USB examples.

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

// Example wiring only. Check these pins and clocks against your SAM4LS board.
// SAM4LS hardware validation has not been performed.

#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

// Requires a 12 MHz crystal and a 32.768 kHz crystal as configured below.
// Change MCUOSC to match your board; USB requires a suitable clock.
#define MCUOSC { \
	{ OSC_TYPE_XTAL, 12000000, 20, 180 }, \
	{ OSC_TYPE_XTAL, 32768, 20, 125 }, true }

#define USB_VBUS_PORT		2
#define USB_VBUS_PIN		11
#define USB_HOST_EN_PORT	2
#define USB_HOST_EN_PIN		12

#define USB_PINS { \
	{USB_VBUS_PORT, USB_VBUS_PIN, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__
