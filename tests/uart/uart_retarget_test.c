/**-------------------------------------------------------------------------
@file	uart_retarget_test.c

@brief	UART stdio partial write and stalled transport tests.

@license

MIT

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
#include <assert.h>
#include <limits.h>
#include <string.h>
#include "coredev/uart.h"

#include "../../src/uart_retarget.c"

static uint8_t s_Output[2048];
static int s_Count;
static int s_Calls;
static int s_Mode;

int DeviceIntrfTx(DevIntrf_t * const pDev, uint32_t DevAddr,
		const uint8_t *pData, int DataLen)
{
	int cnt;

	(void)pDev;
	(void)DevAddr;
	s_Calls++;
	if (s_Mode == 1 || (s_Mode == 2 && s_Count > 0))
	{
		return 0;
	}
	if (s_Mode == 3 || (s_Mode == 4 && s_Count > 0))
	{
		return -1;
	}
	// Simulate FIFO saturation between accepted bursts.
	if (s_Calls % 7 != 1)
	{
		return 0;
	}
	cnt = DataLen > 63 ? 63 : DataLen;
	assert(s_Count + cnt <= (int)sizeof(s_Output));
	memcpy(s_Output + s_Count, pData, cnt);
	s_Count += cnt;
	return cnt;
}

int main(void)
{
	UARTDev_t dev = {0};
	uint8_t input[2048];
	int i;
	int result;

	dev.hStdOut = 1;
	for (i = 0; i < (int)sizeof(input); i++)
	{
		input[i] = (uint8_t)(i * 37);
	}
	assert(UARTStdDevWrite(&dev, 2, input, sizeof(input)) == 0);
	assert(UARTStdDevWrite(&dev, 1, input, 0) == 0);
	assert(UARTStdDevWrite(&dev, 1, NULL, 1) == -1);
	assert(UARTStdDevWrite(&dev, 1, input, (size_t)INT_MAX + 1) == -1);
	assert(s_Calls == 0);
	assert(UARTStdDevWrite(&dev, 1, input, sizeof(input)) == sizeof(input));
	assert(memcmp(input, s_Output, sizeof(input)) == 0);
	for (s_Mode = 1; s_Mode <= 4; s_Mode++)
	{
		s_Count = 0;
		s_Calls = 0;
		result = UARTStdDevWrite(&dev, 1, input, sizeof(input));
		assert(result == (s_Mode == 1 ? 0 : s_Mode == 3 ? -1 : 63));
		assert(s_Calls <= 1000001);
		if (s_Mode >= 3)
		{
			assert(s_Calls <= 2);
		}
		assert(memcmp(input, s_Output, s_Count) == 0);
	}
	return 0;
}
