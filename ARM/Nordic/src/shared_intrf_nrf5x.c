/**-------------------------------------------------------------------------
@file	shared_intrf_nrf5x.c

@brief	Shared interface implementation for nrf51 & nrf52 series

@author	Hoang Nguyen Hoan
@date	July 20, 2018

@lincense

MIT

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#ifdef NRF52840_XXAA
#define MAX_NB_DEV		4
#else
#define MAX_NB_DEV		3
#endif

const int g_SharedIntrfMaxCnt = MAX_NB_DEV;
SharedIntrf_t g_SharedIntrf[MAX_NB_DEV] = { {0, 0},};

#ifdef NRF52_SERIES
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
#else
void SPI0_TWI0_IRQHandler(void)
#endif
{
	if (g_SharedIntrf[0].pIntrf != NULL)
	{
		g_SharedIntrf[0].Handler(0, g_SharedIntrf[0].pIntrf);
	}
#ifdef NRF52_SERIES
	NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
#else
	NVIC_ClearPendingIRQ(SPI0_TWI0_IRQn);
#endif
}

#ifdef NRF52_SERIES
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
#else
void SPI1_TWI1_IRQHandler(void)
#endif
{
	if (g_SharedIntrf[1].pIntrf != NULL)
	{
		g_SharedIntrf[1].Handler(1, g_SharedIntrf[1].pIntrf);
	}
#ifdef NRF52_SERIES
	NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
#else
	NVIC_ClearPendingIRQ(SPI1_TWI1_IRQn);
#endif
}

