/**-------------------------------------------------------------------------
@file	core_cfg_test.c

@brief	Host test support: core cfg test.

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
#include "coredev/iopincfg.h"

static int s_Calls;
static int s_FailAt = -1;
static const IOPinCoreCfg_t s_Pins[] = {
	{ 0, 2, 7 },
	{ 1, 0, 19 },
	{ 2, 3, 4 }
};

bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	int i = s_Calls++;

	assert(i < 3);
	assert(CoreNo == s_Pins[i].CoreNo);
	assert(PortNo == s_Pins[i].PortNo);
	assert(PinNo == s_Pins[i].PinNo);
	return i != s_FailAt;
}

int main(void)
{
	int i;

	assert(IOPinCoreCfg(NULL, 0));
	assert(IOPinCoreCfg(s_Pins, 0));
	assert(!IOPinCoreCfg(NULL, 1));
	assert(!IOPinCoreCfg(s_Pins, -1));
	assert(s_Calls == 0);
	assert(IOPinCoreCfg(s_Pins, 3));
	assert(s_Calls == 3);
	for (i = 0; i < 3; i++)
	{
		s_Calls = 0;
		s_FailAt = i;
		assert(!IOPinCoreCfg(s_Pins, 3));
		assert(s_Calls == i + 1);
	}
	return 0;
}
