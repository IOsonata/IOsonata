/**-------------------------------------------------------------------------
@file	board_pmic_test.cpp

@brief	Host test support: board pmic test.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include <assert.h>
#include <stdio.h>
#include "pwrmgnt/pm_adp5360.h"
#include "pwrmgnt/pm_npm6001.h"

class PmicBus : public DeviceIntrf {
public:
	uint8_t Reg[256] = {};
	unsigned Writes = 0;
	bool Fail = false;
	operator DevIntrf_t *() override
	{
		return NULL;
	}
	DEVINTRF_TYPE Type() override
	{
		return DEVINTRF_TYPE_I2C;
	}
	uint32_t Rate(uint32_t Value) override
	{
		return Value;
	}
	uint32_t Rate() override
	{
		return 100000;
	}
	int Read(uint32_t, const uint8_t *pCmd, int CmdLen,
		uint8_t *pData, int Len) override
	{
		assert(CmdLen == 1 && Len == 1);
		if (Fail)
			return 0;
		*pData = Reg[*pCmd];
		return 1;
	}
	int Write(uint32_t, const uint8_t *pCmd, int CmdLen,
		const uint8_t *pData, int Len) override
	{
		assert(CmdLen == 1 && Len == 1);
		if (Fail)
			return 0;
		Reg[*pCmd] = *pData;
		Writes++;
		return 1;
	}
};

int main()
{
	PmicBus bus;
	PmAdp5360 adp;
	PmnPM6001 npm;
	unsigned writes;

	assert(!adp.Enable() && !npm.Enable());
	adp.Disable();
	npm.Disable();
	assert(bus.Writes == 0);
	bus.Reg[0] = ADP5360_DEVICE_ID;
	bus.Reg[7] = 0xA0;
	assert(adp.Init(&bus) && adp.Valid());
	assert(npm.Init(&bus) && npm.Valid());
	assert(bus.Writes == 0);
	assert(adp.Enable());
	assert(bus.Reg[7] == 0xA1);
	adp.Disable();
	assert(bus.Reg[7] == 0xA0);
	assert(npm.Enable());
	assert(bus.Reg[NPM6001_START_BUCK3_REG] == 1);
	npm.Disable();
	assert(bus.Reg[NPM6001_STOP_BUCK3_REG] == 1);
	bus.Fail = true;
	assert(!adp.Enable() && !npm.Enable());
	bus.Fail = false;
	writes = bus.Writes;
	adp.Reset();
	npm.Reset();
	assert(bus.Writes == writes);
	assert(!adp.Valid() && !adp.Initialized());
	assert(!npm.Valid() && !npm.Initialized());
	assert(!adp.Enable() && !npm.Enable());
	assert(adp.Init(&bus) && npm.Init(&bus));
	assert(!adp.Init(NULL) && !adp.Valid());
	assert(!npm.Init(NULL) && !npm.Valid());
	puts("Board PMIC lifecycle tests: PASS");
	return 0;
}
