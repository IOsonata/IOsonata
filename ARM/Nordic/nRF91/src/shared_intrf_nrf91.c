/**-------------------------------------------------------------------------
@file	shared_intrf_nrf91.c

@brief	Shared in terface nRF91 implementation

Implementation of shared interface for nRF91

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

#define MAX_NB_DEV		4

const int g_SharedIntrfMaxCnt = MAX_NB_DEV;	// Instance must be declared in MCU implementation
SharedIntrf_t g_SharedIntrf[MAX_NB_DEV] = { {0, 0}, };	// Instance must be declared in MCU implementation

void UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler(void)
{
	if (g_SharedIntrf[0].pIntrf != NULL)
	{
		g_SharedIntrf[0].Handler(0, g_SharedIntrf[0].pIntrf);
	}
	NVIC_ClearPendingIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
}

void UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler(void)
{
	if (g_SharedIntrf[1].pIntrf != NULL)
	{
		g_SharedIntrf[1].Handler(1, g_SharedIntrf[1].pIntrf);
	}
	NVIC_ClearPendingIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
}

void UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQHandler(void)
{
    if (g_SharedIntrf[2].pIntrf != NULL)
    {
    	g_SharedIntrf[2].Handler(2, g_SharedIntrf[2].pIntrf);
    }
    NVIC_ClearPendingIRQ(UARTE2_SPIM2_SPIS2_TWIM2_TWIS2_IRQn);
}

void UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQHandler(void)
{
    if (g_SharedIntrf[3].pIntrf != NULL)
    {
    	g_SharedIntrf[3].Handler(3, g_SharedIntrf[3].pIntrf);
    }
    NVIC_ClearPendingIRQ(UARTE3_SPIM3_SPIS3_TWIM3_TWIS3_IRQn);
}

