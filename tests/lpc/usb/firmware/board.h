/**-------------------------------------------------------------------------
@file	board.h

@brief	LPC11U35 USB firmware link fixture

Clock and pin configuration used only to link the generic CDC example.
Physical board wiring and USB operation require separate validation.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef __LPC11_USB_LINK_BOARD_H__
#define __LPC11_USB_LINK_BOARD_H__

#include "coredev/system_core_clock.h"

// Link fixture only: these are the controller's supported clock/pin settings,
// not a verified board schematic or a hardware-ready example project.
#define MCUOSC { \
	{OSC_TYPE_XTAL, 12000000, 20}, \
	{OSC_TYPE_RC, 4000000, 20}, \
	true \
}
#define USB_PINS { \
	{0, 3, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}

#endif
