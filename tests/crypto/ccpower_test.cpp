/**-------------------------------------------------------------------------
@file	ccpower_test.cpp

@brief	Host test support: ccpower test.

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

#include "nrf.h"
#include "cc3xx_intrf.h"
#include <cassert>
TestWrapper g_Wrapper;
int main()
{
	assert(Cc3xxBase() == TEST_CC_BASE);
	assert(Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Value == 1 && g_Wrapper.ENABLE.Writes == 1);
	assert(Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Writes == 1);
	Cc3xxDisable();
	assert(g_Wrapper.ENABLE.Value == 0 && g_Wrapper.ENABLE.Writes == 2);
	g_Wrapper.ENABLE.Fail = true;
	assert(!Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Writes == 3);
	g_Wrapper.ENABLE.Fail = false;
	assert(Cc3xxEnable());
	assert(g_Wrapper.ENABLE.Writes == 4);
	Cc3xxDisable();
}
