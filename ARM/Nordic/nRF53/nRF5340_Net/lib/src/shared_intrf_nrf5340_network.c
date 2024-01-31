/**-------------------------------------------------------------------------
@file	shared_intrf_nrf5340_network.c

@brief	Shared interface implementation on nRF5340 Network

Implementation of shared interface for nRF5340 Network MCU


@author	Nguyen Hoan Hoang
@date	Mar. 12, 2020

@lincense

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#include <string.h>

#include "nrf.h"

#include "device_intrf.h"
#include "coredev/shared_intrf.h"

#define MAX_NB_DEV		1

const int g_SharedIntrfMaxCnt = MAX_NB_DEV;
SharedIntrf_t g_SharedIntrf[MAX_NB_DEV] = { {0, 0},};

//void SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQHandler(void)
void SERIAL0_IRQHandler(void)
{
	if (g_SharedIntrf[0].pIntrf != NULL)
	{
		g_SharedIntrf[0].Handler(0, g_SharedIntrf[0].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL0_IRQn);
}


