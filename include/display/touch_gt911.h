/**-------------------------------------------------------------------------
@file	touch_gt911.h

@brief	Goodix GT911 capacitive touch panel controller, I2C

Up to 10 contacts (5 on most panels). The I2C address, 0x5D or 0x14, is
selected by the level of the interrupt pin when the reset pin goes high:
with both pins Init does the reset sequence for the address set. Without
them the address is the one the board gives. The coordinate range is read
from the controller configuration, which is not changed. Disable puts it in
sleep mode with the interrupt pin held low, Enable wakes it up with the
reset sequence; without the reset and interrupt pins Disable does nothing.

A contact read only when the controller has new coordinates (buffer status
set), the last ones are returned until then.

Used on the RK043FN66HS-CTG panel of the LPCXpresso54628.

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
#ifndef __TOUCH_GT911_H__
#define __TOUCH_GT911_H__

#include "display/touch.h"

#define GT911_I2C_ADDR0			0x5D	//!< Interrupt pin low at the reset
#define GT911_I2C_ADDR1			0x14	//!< Interrupt pin high at the reset
#define GT911_MAX_POINTS		10

class TouchGt911 : public Touch {
public:
	TouchGt911() : vbSleep(false), vNbRaw(0) {}

	/**
	 * @brief	Set up the controller.
	 *
	 * @param	Cfg		: Panel, DevAddr 0x5D or 0x14, 0 for 0x5D
	 * @param	pIntrf	: I2C interface
	 *
	 * @return	true when a GT911 answers
	 */
	bool Init(const TouchCfg_t &Cfg, DeviceIntrf *pIntrf);
	bool Enable();
	void Disable();
	void Reset();
	int Read(TouchPoint_t *pPoints, int MaxPoints);

private:
	bool Start();
	int ReadReg(uint16_t Reg, uint8_t *pData, int Len);
	int WriteReg(uint16_t Reg, const uint8_t *pData, int Len);

	bool vbSleep;
	int vNbRaw;
	TouchPoint_t vRaw[GT911_MAX_POINTS];	//!< Last contacts, controller coordinates
};

#endif // __TOUCH_GT911_H__
