/**-------------------------------------------------------------------------
@file	pm_npm1100.cpp
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

#include "pwrmgnt/pm_npm1100.h"
#include "coredev/iopinctrl.h"

void PmnPM1100::Select100mA()
{
	IOPinClear(vCfg.IsetPin.PortNo, vCfg.IsetPin.PinNo);
	vCurrentLimit = 100;
}

bool PmnPM1100::Init(const PmnPM1100Cfg_t &Cfg)
{
	Valid(false);
	vbInitialized = false;

	if (Cfg.IsetPin.PortNo < 0 || Cfg.IsetPin.PinNo < 0)
	{
		return false;
	}

	vCfg = Cfg;
	IOPinClear(vCfg.IsetPin.PortNo, vCfg.IsetPin.PinNo);
	IOPinConfig(vCfg.IsetPin.PortNo, vCfg.IsetPin.PinNo,
				vCfg.IsetPin.PinOp, IOPINDIR_OUTPUT,
				vCfg.IsetPin.Res, vCfg.IsetPin.Type);
	Select100mA();

	vbInitialized = true;
	Valid(true);

	return true;
}

bool PmnPM1100::Enable()
{
	if (!Device::Valid() || !vbInitialized)
	{
		return false;
	}

	Select100mA();

	return true;
}

void PmnPM1100::Disable()
{
	if (vbInitialized)
	{
		Select100mA();
	}
}

void PmnPM1100::Reset()
{
	if (vbInitialized)
	{
		Select100mA();
	}
}

void PmnPM1100::PowerOff()
{
	if (vbInitialized)
	{
		Select100mA();
	}

	vbInitialized = false;
	Valid(false);
}

uint32_t PmnPM1100::InputCurrentLimit(uint32_t MilliAmp)
{
	if (!Device::Valid() || !vbInitialized)
	{
		return 0;
	}

	if (MilliAmp >= 500)
	{
		IOPinSet(vCfg.IsetPin.PortNo, vCfg.IsetPin.PinNo);
		vCurrentLimit = 500;
	}
	else
	{
		Select100mA();
	}

	return vCurrentLimit;
}
