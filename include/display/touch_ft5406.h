/**-------------------------------------------------------------------------
@file	touch_ft5406.h

@brief	FocalTech FT5406 capacitive touch panel controller, I2C

Up to 5 contacts. The controller reports pixels. Init resets it with the
reset pin when given (5 ms low, then 300 ms before it answers) and sets the
normal operating mode. Disable puts it in hibernate mode, Enable wakes it up
with a reset; without the reset pin Disable does nothing.

Used on the RK043FN02H-CT panel of the LPCXpresso54628, its X along the
panel height (TOUCH_FLAG_XY_SWAP).

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __TOUCH_FT5406_H__
#define __TOUCH_FT5406_H__

#include "display/touch.h"

#define FT5406_I2C_ADDR			0x38
#define FT5406_MAX_POINTS		5

class TouchFt5406 : public Touch {
public:
	TouchFt5406() : vbSleep(false) {}

	/**
	 * @brief	Set up the controller.
	 *
	 * @param	Cfg		: Panel, DevAddr 0 for 0x38
	 * @param	pIntrf	: I2C interface
	 *
	 * @return	true when the controller answers
	 */
	bool Init(const TouchCfg_t &Cfg, DeviceIntrf *pIntrf);
	bool Enable();
	void Disable();
	void Reset();
	int Read(TouchPoint_t *pPoints, int MaxPoints);

private:
	bool Start();

	bool vbSleep;
};

#endif // __TOUCH_FT5406_H__
