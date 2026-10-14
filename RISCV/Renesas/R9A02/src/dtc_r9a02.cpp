/**-------------------------------------------------------------------------
@file	dtc_r9a02.cpp

@brief	R9A02G021 data transfer controller (DTC)

See dtc_r9a02.h. R9A02G021 User's Manual R01UH1036EJ0110, chapter 14.

Set up as table 14.9: DTCVBR, DTCCR.RRS = 0 while records change, the
records, the vector entry, IELSRn.DTCE = 1, then the peripheral. RRS stays
0: the drivers rewrite their records before every transfer, the read skip
would have to be turned off each time (14.4.1).

The DTC is out of module stop after reset (MSTPCRA.MSTPA22 = 0), it is
started again here if it was stopped.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <stdint.h>
#include <stddef.h>

#include "coredev/interrupt.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "dtc_r9a02.h"

#define R9A02_DTC_STOP_LOOPS		100000U		// A transfer of one slot to end

static_assert(sizeof(R9a02DtcInfo_t) == 16, "DTC record is 16 bytes");
static_assert(offsetof(R9a02DtcInfo_t, Mra) == 3, "MRA at +3");
static_assert(offsetof(R9a02DtcInfo_t, Cra) == 14, "CRA at +14");
static_assert(offsetof(R9A02_DTC_Type, DTCST) == 0x0C, "DTCST at 0x0C");

// Vector table, 1 KB aligned (14.2.8), placed by the linker script
alignas(1024) static uint32_t s_R9a02DtcVector[R9A02_DTC_VECTOR_CNT] __attribute__((section(".bss.dtc_vector")));

static bool s_bR9a02DtcStarted = false;

// Events marked "Invoke DTC" in table 12.4, except those table 12.7 gives as
// CPU interrupts only (TAU). Software events complete through the DTC
// before their CLIC interrupt (table 12.4).
static const uint8_t s_R9a02DtcEvt[] = {
	R9A02_EVT_PORT_IRQ0, R9A02_EVT_PORT_IRQ1, R9A02_EVT_PORT_IRQ2, R9A02_EVT_PORT_IRQ3,
	R9A02_EVT_PORT_IRQ4, R9A02_EVT_PORT_IRQ5, R9A02_EVT_PORT_IRQ6, R9A02_EVT_PORT_IRQ7,
	R9A02_EVT_SYSTEM_SNZREQ, R9A02_EVT_IIC0_ENDI, R9A02_EVT_IIC1_ENDI,
	R9A02_EVT_IOPORT_GROUP1, R9A02_EVT_IOPORT_GROUP2,
	R9A02_EVT_ELC_SWEVT0, R9A02_EVT_ELC_SWEVT1,
	R9A02_EVT_SAU0_ENDI0, R9A02_EVT_SAU0_ENDI1, R9A02_EVT_SAU0_ENDI2, R9A02_EVT_SAU0_ENDI3,
	R9A02_EVT_SAU1_ENDI0, R9A02_EVT_SAU1_ENDI1,
	R9A02_EVT_TML32_OUTI, R9A02_EVT_REMC_OUTI,
	R9A02_EVT_UARTA_TX_ENDI0, R9A02_EVT_UARTA_RX_ENDI0, R9A02_EVT_UARTA_TX_ENDI1, R9A02_EVT_UARTA_RX_ENDI1,
	R9A02_EVT_ADC_ENDI,
};

static bool R9a02DtcEvent(uint8_t EvtId)
{
	for (size_t i = 0; i < sizeof(s_R9a02DtcEvt); i++)
	{
		if (s_R9a02DtcEvt[i] == EvtId)
		{
			return true;
		}
	}

	return false;
}

// SRAM0, ECC or parity area (manual 4.1)
static bool R9a02DtcInSram(const void *p, size_t Size)
{
	uintptr_t a = (uintptr_t)p;

	return ((a & 3U) == 0U) &&
		   ((a >= R9A02_SRAM_ECC_BASE && a + Size <= R9A02_SRAM_ECC_BASE + R9A02_SRAM_ECC_SIZE) ||
			(a >= R9A02_SRAM_BASE && a + Size <= R9A02_SRAM_BASE + R9A02_SRAM_SIZE));
}

// Module on, vector table, DTCST = 1 (table 14.9)
static void R9a02DtcStart(void)
{
	R9A02_DTC_Type *reg = R9A02_DTC;

	if (!R9a02ModuleRunning(R9A02_MSTPCR_A, R9A02_MSTPCRA_DTC))
	{
		R9a02ModuleStart(R9A02_MSTPCR_A, R9A02_MSTPCRA_DTC);
	}

	reg->DTCST = 0U;
	reg->DTCCR = R9A02_DTCCR_FIXED;
	for (uint32_t i = 0; i < R9A02_DTC_VECTOR_CNT; i++)
	{
		s_R9a02DtcVector[i] = 0U;
	}
	reg->DTCVBR = (uint32_t)(uintptr_t)s_R9a02DtcVector;
	reg->DTCST = R9A02_DTCST_DTCST;
	s_bR9a02DtcStarted = true;
}

bool R9a02DtcSet(int IntNo, R9a02DtcInfo_t *pInfo)
{
	uint8_t evt = IntNo >= 0 && IntNo < (int)R9A02_DTC_VECTOR_CNT ? R9a02IntEvent(IntNo) : 0U;

	if (pInfo == NULL || !R9a02DtcInSram(pInfo, sizeof(R9a02DtcInfo_t)) || !R9a02DtcEvent(evt))
	{
		return false;
	}

	// An IRQ pin starts the DTC on an edge only (12.5.4)
	if (evt >= R9A02_EVT_PORT_IRQ0 && evt <= R9A02_EVT_PORT_IRQ7 &&
		(R9A02_ICU->IRQCR[evt - R9A02_EVT_PORT_IRQ0] & R9A02_IRQCR_IRQMD_Msk) == R9A02_IRQCR_IRQMD_LOW)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();

	if (R9A02_ICU->IELSR[IntNo] & R9A02_IELSR_DTCE)
	{
		EnableInterrupt(state);

		return false;
	}

	if (!s_bR9a02DtcStarted || R9A02_DTC->DTCVBR != (uint32_t)(uintptr_t)s_R9a02DtcVector)
	{
		R9a02DtcStart();
	}

	// RRS = 0 drops the vector kept for the read skip (14.2.7)
	R9A02_DTC->DTCCR = R9A02_DTCCR_FIXED;
	s_R9a02DtcVector[IntNo] = (uint32_t)(uintptr_t)pInfo;

	EnableInterrupt(state);

	return true;
}

bool R9a02DtcEnable(int IntNo)
{
	if (IntNo < 0 || IntNo >= (int)R9A02_DTC_VECTOR_CNT || s_R9a02DtcVector[IntNo] == 0U)
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();
	uint32_t reg = R9A02_ICU->IELSR[IntNo];
	uint32_t iels = reg & R9A02_IELSR_IELS_Msk;

	if (iels == 0U || (reg & R9A02_IELSR_DTCE))
	{
		EnableInterrupt(state);

		return false;
	}

	// The records and the vector entry are in SRAM before the first event,
	// the DTC is another bus master
	__asm volatile ("fence rw, rw" ::: "memory");
	(void)*(volatile uint32_t *)&s_R9a02DtcVector[IntNo];

	// IR cleared with DTCE at 0 (12.2.6), an old event does not start a
	// transfer, then DTCE
	R9A02_ICU->IELSR[IntNo] = iels;
	(void)R9A02_ICU->IELSR[IntNo];
	R9A02_ICU->IELSR[IntNo] = iels | R9A02_IELSR_DTCE;
	(void)R9A02_ICU->IELSR[IntNo];

	EnableInterrupt(state);

	return true;
}

void R9a02DtcDisable(int IntNo)
{
	if (IntNo < 0 || IntNo >= (int)R9A02_DTC_VECTOR_CNT)
	{
		return;
	}

	uint32_t n = R9A02_DTC_STOP_LOOPS;
	uintptr_t state = DisableInterrupt();
	uint32_t iels = R9A02_ICU->IELSR[IntNo] & R9A02_IELSR_IELS_Msk;

	// A transfer the DTC has accepted ends (14.2.9). The DTC registers are
	// not read in module stop (14.9).
	while (R9a02ModuleRunning(R9A02_MSTPCR_A, R9A02_MSTPCRA_DTC) && (R9A02_DTC->DTCSTS & R9A02_DTCSTS_ACT) &&
		   (R9A02_DTC->DTCSTS & R9A02_DTCSTS_VECN_Msk) == (uint32_t)IntNo && --n > 0U)
	{
	}

	// DTCE to 0 first, then IR (12.2.6)
	R9A02_ICU->IELSR[IntNo] = iels;
	(void)R9A02_ICU->IELSR[IntNo];
	R9A02_ICU->IELSR[IntNo] = iels;
	(void)R9A02_ICU->IELSR[IntNo];

	EnableInterrupt(state);
}

bool R9a02DtcEnabled(int IntNo)
{
	return IntNo >= 0 && IntNo < (int)R9A02_DTC_VECTOR_CNT && (R9A02_ICU->IELSR[IntNo] & R9A02_IELSR_DTCE) != 0U;
}

void R9a02DtcClear(int IntNo)
{
	if (IntNo < 0 || IntNo >= (int)R9A02_DTC_VECTOR_CNT)
	{
		return;
	}

	R9a02DtcDisable(IntNo);
	s_R9a02DtcVector[IntNo] = 0U;
}
