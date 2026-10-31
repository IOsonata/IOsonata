/**-------------------------------------------------------------------------
@file	shared_intrf_samd.cpp

@brief	SAM D11/D21 SERCOM interrupt dispatch

UART, I2C and SPI share the SERCOM instances. The driver of a SERCOM
registers its interrupt handler with SharedIntrfSetIrqHandler, DevNo is the
SERCOM number.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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
#include <stddef.h>

#include "samdxxx.h"

#include "device_intrf.h"
#include "coredev/shared_intrf.h"
#include "sercom_samd.h"

const int g_SharedIntrfMaxCnt = SAMD_SERCOM_CNT;
SharedIntrf_t g_SharedIntrf[SAMD_SERCOM_CNT] = { { NULL, NULL }, };

static inline void SamdSercomDispatch(int DevNo)
{
	if (g_SharedIntrf[DevNo].pIntrf != NULL && g_SharedIntrf[DevNo].Handler != NULL)
	{
		g_SharedIntrf[DevNo].Handler(DevNo, g_SharedIntrf[DevNo].pIntrf);
	}
}

// Vector table entries are C symbols
extern "C" void SERCOM0_Handler(void)
{
	SamdSercomDispatch(0);
}

extern "C" void SERCOM1_Handler(void)
{
	SamdSercomDispatch(1);
}

extern "C" void SERCOM2_Handler(void)
{
	SamdSercomDispatch(2);
}

#if SAMD_SERCOM_CNT > 3
extern "C" void SERCOM3_Handler(void)
{
	SamdSercomDispatch(3);
}

extern "C" void SERCOM4_Handler(void)
{
	SamdSercomDispatch(4);
}

extern "C" void SERCOM5_Handler(void)
{
	SamdSercomDispatch(5);
}
#endif
