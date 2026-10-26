/**-------------------------------------------------------------------------
@file	pm_npm1100.h
@brief	Nordic nPM1100 pin-control driver
@author	Hoang Nguyen Hoan
@date	Oct. 24, 2026
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
#ifndef __PM_NPM1100_H__
#define __PM_NPM1100_H__

#include <stdint.h>
#include "device.h"

typedef struct __PmnPM1100_Config
{
	IOPinCfg_t IsetPin;
} PmnPM1100Cfg_t;

class PmnPM1100 : public Device
{
public:
	bool Init(const PmnPM1100Cfg_t &Cfg);
	bool Enable();
	void Disable();
	void Reset();
	void PowerOff();

	uint32_t InputCurrentLimit(uint32_t MilliAmp);
	uint32_t InputCurrentLimit() const
	{
		return vCurrentLimit;
	}

private:
	void Select100mA();

	PmnPM1100Cfg_t vCfg{};
	uint32_t vCurrentLimit = 100;
	bool vbInitialized = false;
};

#endif
