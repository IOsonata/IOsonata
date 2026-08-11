/**-------------------------------------------------------------------------
@file	shared_intrf_nrfx.cpp

@brief	Shared serial interface dispatch for the nRF families

UARTE / SPIM / SPIS / TWIM / TWIS on these parts share a single peripheral
instance and a single interrupt line. This file routes each shared line to the
active device interface registered for that instance.

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

// One slot per shared instance. The index a device registers with is its
// DevNo, so the slot count follows the largest DevNo any driver can hand over.
#ifdef NRF52840_XXAA
#define MAX_NB_DEV		4
#elif defined(NRF54H_SERIES)
#define MAX_NB_DEV		9
#elif defined(NRF54L_SERIES)
#ifdef NRF_UARTE23
#define MAX_NB_DEV		7
#else
#define MAX_NB_DEV		5
#endif
#elif defined(NRF91_SERIES)
#define MAX_NB_DEV		4
#else
#define MAX_NB_DEV		3
#endif

// UARTE00 on nRF54L is listed after the general purpose instances in
// uart_nrfx.cpp, so its slot moves with the instance count of the part.
#ifdef NRF_UARTE23
#define SHARED_IDX_UARTE00	6
#else
#define SHARED_IDX_UARTE00	4
#endif

const int g_SharedIntrfMaxCnt = MAX_NB_DEV;
SharedIntrf_t g_SharedIntrf[MAX_NB_DEV] = { {0, 0},};

// Vector table entries are C symbols. Keep C linkage on the handlers below so
// these definitions override the weak defaults declared in the vector table.
extern "C" {

#ifdef NRF52805_XXAA
void TWIM0_TWIS0_TWI0_IRQHandler()
#elif defined(NRF52_SERIES)
void SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler(void)
#elif defined(NRF54H_SERIES)
void SERIAL0_IRQHandler(void)
#elif defined(NRF54L_SERIES)
void SERIAL30_IRQHandler(void)
#elif defined(NRF53_SERIES)
void SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQHandler(void)
#elif defined(NRF91_SERIES)
void UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQHandler(void)
#else
void SPI0_TWI0_IRQHandler(void)
#endif
{
	if (g_SharedIntrf[0].pIntrf != NULL)
	{
		g_SharedIntrf[0].Handler(0, g_SharedIntrf[0].pIntrf);
	}
#if defined(NRF52805_XXAA) || defined(NRF52810_XXAA)
	NVIC_ClearPendingIRQ(TWIM0_TWIS0_TWI0_IRQn);
#elif defined(NRF52_SERIES)
	NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn);
#elif defined(NRF54H_SERIES)
	NVIC_ClearPendingIRQ(SERIAL0_IRQn);
#elif defined(NRF54L_SERIES)
	NVIC_ClearPendingIRQ(SERIAL30_IRQn);
#elif defined(NRF53_SERIES)
	NVIC_ClearPendingIRQ(SPIM0_SPIS0_TWIM0_TWIS0_UARTE0_IRQn);
#elif defined(NRF91_SERIES)
	NVIC_ClearPendingIRQ(UARTE0_SPIM0_SPIS0_TWIM0_TWIS0_IRQn);
#else
	NVIC_ClearPendingIRQ(SPI0_TWI0_IRQn);
#endif
}

#if !defined(NRF52805_XXAA) && !defined(NRF52810_XXAA) && !defined(NRF5340_XXAA_NETWORK)
#ifdef NRF52_SERIES
void SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler(void)
#elif defined(NRF54H_SERIES)
void SERIAL1_IRQHandler(void)
#elif defined(NRF54L_SERIES)
void SERIAL20_IRQHandler(void)
#elif defined(NRF53_SERIES)
void SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQHandler(void)
#elif defined(NRF91_SERIES)
void UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQHandler(void)
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
#elif defined(NRF54H_SERIES)
	NVIC_ClearPendingIRQ(SERIAL1_IRQn);
#elif defined(NRF54L_SERIES)
	NVIC_ClearPendingIRQ(SERIAL20_IRQn);
#elif defined(NRF53_SERIES)
	NVIC_ClearPendingIRQ(SPIM1_SPIS1_TWIM1_TWIS1_UARTE1_IRQn);
#elif defined(NRF91_SERIES)
	NVIC_ClearPendingIRQ(UARTE1_SPIM1_SPIS1_TWIM1_TWIS1_IRQn);
#else
	NVIC_ClearPendingIRQ(SPI1_TWI1_IRQn);
#endif
}
#endif

#if defined(NRF54L_SERIES)
void SERIAL21_IRQHandler(void)
{
	if (g_SharedIntrf[2].pIntrf != NULL)
	{
		g_SharedIntrf[2].Handler(2, g_SharedIntrf[2].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL21_IRQn);
}

void SERIAL22_IRQHandler(void)
{
	if (g_SharedIntrf[3].pIntrf != NULL)
	{
		g_SharedIntrf[3].Handler(3, g_SharedIntrf[3].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL22_IRQn);
}

#ifdef NRF_UARTE23
// nRF54LM20A and up
void SERIAL23_IRQHandler(void)
{
	if (g_SharedIntrf[4].pIntrf != NULL)
	{
		g_SharedIntrf[4].Handler(4, g_SharedIntrf[4].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL23_IRQn);
}

void SERIAL24_IRQHandler(void)
{
	if (g_SharedIntrf[5].pIntrf != NULL)
	{
		g_SharedIntrf[5].Handler(5, g_SharedIntrf[5].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL24_IRQn);
}
#endif

void SERIAL00_IRQHandler(void)
{
	if (g_SharedIntrf[SHARED_IDX_UARTE00].pIntrf != NULL)
	{
		g_SharedIntrf[SHARED_IDX_UARTE00].Handler(SHARED_IDX_UARTE00,
							  g_SharedIntrf[SHARED_IDX_UARTE00].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL00_IRQn);
}
#endif

#if defined(NRF54H_SERIES)
// SERIAL0 and SERIAL1 are handled above. UARTE120 sits on the fast domain and
// does not have a SERIALn vector of its own.
void SERIAL2_IRQHandler(void)
{
	if (g_SharedIntrf[2].pIntrf != NULL)
	{
		g_SharedIntrf[2].Handler(2, g_SharedIntrf[2].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL2_IRQn);
}

void SERIAL3_IRQHandler(void)
{
	if (g_SharedIntrf[3].pIntrf != NULL)
	{
		g_SharedIntrf[3].Handler(3, g_SharedIntrf[3].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL3_IRQn);
}

void SERIAL4_IRQHandler(void)
{
	if (g_SharedIntrf[4].pIntrf != NULL)
	{
		g_SharedIntrf[4].Handler(4, g_SharedIntrf[4].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL4_IRQn);
}

void SERIAL5_IRQHandler(void)
{
	if (g_SharedIntrf[5].pIntrf != NULL)
	{
		g_SharedIntrf[5].Handler(5, g_SharedIntrf[5].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL5_IRQn);
}

void SERIAL6_IRQHandler(void)
{
	if (g_SharedIntrf[6].pIntrf != NULL)
	{
		g_SharedIntrf[6].Handler(6, g_SharedIntrf[6].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL6_IRQn);
}

void SERIAL7_IRQHandler(void)
{
	if (g_SharedIntrf[7].pIntrf != NULL)
	{
		g_SharedIntrf[7].Handler(7, g_SharedIntrf[7].pIntrf);
	}
	NVIC_ClearPendingIRQ(SERIAL7_IRQn);
}

void SPIM120_UARTE120_IRQHandler(void)
{
	if (g_SharedIntrf[8].pIntrf != NULL)
	{
		g_SharedIntrf[8].Handler(8, g_SharedIntrf[8].pIntrf);
	}
	NVIC_ClearPendingIRQ(SPIM120_UARTE120_IRQn);
}
#endif

#if defined(NRF5340_XXAA_APPLICATION)
void SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQHandler(void)
{
	if (g_SharedIntrf[2].pIntrf != NULL)
	{
		g_SharedIntrf[2].Handler(2, g_SharedIntrf[2].pIntrf);
	}
	NVIC_ClearPendingIRQ(SPIM2_SPIS2_TWIM2_TWIS2_UARTE2_IRQn);
}

void SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQHandler(void)
{
	if (g_SharedIntrf[3].pIntrf != NULL)
	{
		g_SharedIntrf[3].Handler(3, g_SharedIntrf[3].pIntrf);
	}
	NVIC_ClearPendingIRQ(SPIM3_SPIS3_TWIM3_TWIS3_UARTE3_IRQn);
}
#endif

#if defined(NRF91_SERIES)
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
#endif

} // extern "C"
