/**-------------------------------------------------------------------------
@file	shared_irq_nrf5340_application.cpp

@brief	Shared IRQ handler

Implementation of shared IRQ for nRF5340 Application MCU

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
#include "coredev/shared_irq.h"

#pragma pack(push, 4)
typedef struct {
	DevIntrf_t *pDev;				// Device interface data
	IRQHANDLER Handler ;		// Device interface interrupt handler
} IRQDATA;
#pragma pack(pop)

#define MAX_NB_DEV		4

static IRQDATA s_DevIrq[MAX_NB_DEV] = { {NULL, }, };

void SetSharedIntHandler(int DevNo, DevIntrf_t *pDev, IRQHANDLER Handler)
{
	if (DevNo < 0 || DevNo >= MAX_NB_DEV)
	{
		return;
	}

	s_DevIrq[DevNo].pDev = pDev;
	s_DevIrq[DevNo].Handler = Handler;
}

extern "C" void SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQHandler(void)
{
	if (s_DevIrq[0].pDev != NULL)
	{
		s_DevIrq[0].Handler(0, s_DevIrq[0].pDev);
	}
	NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
}

extern "C" void SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQHandler(void)
{
	if (s_DevIrq[1].pDev != NULL)
	{
		s_DevIrq[1].Handler(1, s_DevIrq[1].pDev);
	}
	NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
}

extern "C" void SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQHandler(void)
{
    if (s_DevIrq[2].pDev != NULL)
    {
        s_DevIrq[2].Handler(2, s_DevIrq[2].pDev);
    }
    NVIC_ClearPendingIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
}

extern "C" void SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQHandler(void)
{
    if (s_DevIrq[3].pDev != NULL)
    {
        s_DevIrq[3].Handler(3, s_DevIrq[3].pDev);
    }
    NVIC_ClearPendingIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
}

