/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro USB CDC loopback

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The pins are those of the SAM D21 Xplained Pro, target USB connector.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#include "coredev/system_core_clock.h"

// USB clock: DFLL48M on the 32.768 kHz crystal, 47.97 MHz
#define MCUOSC { \
	{ OSC_TYPE_RC, 8000000, 0, 0 }, \
	{ OSC_TYPE_XTAL, 32768, 20, 0 }, true }

// Target USB VBUS detection, PA14, EIC line 14
#define USB_VBUS_PORT		IOPORTA
#define USB_VBUS_PIN		14

#define USB_PINS { \
	{USB_VBUS_PORT, USB_VBUS_PIN, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__
