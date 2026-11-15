/**-------------------------------------------------------------------------
@file	display_intrf.h

@brief	Display console output interface.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#ifndef __DISPLAY_INTRF_H__
#define __DISPLAY_INTRF_H__

#include "display/display.h"

// Transmit-only text sink for SysLog and other DeviceIntrf users.
// Init receives an initialized Display owned by the application. The adapter
// does not reset, enable or disable the panel or its physical interface.
// Use from the foreground/task that owns the display, never an ISR. SysLog
// with an attached sink sends immediately in the caller's context.
// A positive count means text was passed to Display::printf; that void API
// cannot report a panel transfer failure. NUL bytes are consumed, not drawn.
class DisplayIntrf : public DeviceIntrf {
public:
	DisplayIntrf();
	bool Init(Display *pDisplay);
	operator DevIntrf_t * () override { return &vIntrf; }
	uint32_t Rate(uint32_t Value) override {
		(void)Value;
		return 0;
	}
	uint32_t Rate() override { return 0; }
	DisplayIntrf(const DisplayIntrf &) = delete;
	DisplayIntrf &operator=(const DisplayIntrf &) = delete;

private:
	static void DisableOutput(DevIntrf_t *pDev);
	static void EnableOutput(DevIntrf_t *pDev);
	static uint32_t GetRate(DevIntrf_t *pDev);
	static uint32_t SetRate(DevIntrf_t *pDev, uint32_t Value);
	static bool StartRxOutput(DevIntrf_t *pDev, uint32_t Address);
	static int Receive(DevIntrf_t *pDev, uint8_t *pData, int Len);
	static bool StartTxOutput(DevIntrf_t *pDev, uint32_t Address);
	static int Transmit(DevIntrf_t *pDev, const uint8_t *pData, int Len);
	static void Stop(DevIntrf_t *pDev);
	static void ResetOutput(DevIntrf_t *pDev);
	static void PowerOffOutput(DevIntrf_t *pDev);
	static void *Handle(DevIntrf_t *pDev);
	DevIntrf_t vIntrf;
	Display *vpDisplay;
	bool vbEnabled;
};

#endif
