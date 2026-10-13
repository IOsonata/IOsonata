/**-------------------------------------------------------------------------
@file	analog_comp_r9a02.cpp

@brief	R9A02G021 comparators (CMP)

converters/analog_comp.h implementation. R9A02G021 User's Manual
R01UH1036EJ0110, chapter 31. Two comparators:

	DevNo 0 : CMP0, input IVCMP0 (P106), reference IVREF0 (P000) or the
			  internal reference 1.44 V typical (1.34 to 1.54 V)
	DevNo 1 : CMP1, input IVCMP1 (P107), reference IVREF1 (P104) or the
			  output of DAC8 channel 0

AnalogCompCfg_t:
	Mode		: ANALOG_COMP_MODE_CONTINUOUS, the comparators always run
	RefSrc		: 0 for the internal reference (CMP0) or DAC8 channel 0
				  (CMP1), 1 for the IVREFn pin
	RefVolt		: VCC in mV
	CompVolt	: reference voltage in mV. With DAC8 channel 0 it sets the
				  code, CompVolt x 256 / RefVolt. On the IVREFn pin it selects
				  the reference range, 0 to VCC - 1.4 V or 1.4 V to VCC
				  (COMPMDR.CnLVL). Not used with the internal reference.
	AnalogIn	: 0, the only input of each comparator
	bHystersys	: false, the comparators have no hysteresis
	EvtHandler	: ANALOG_COMP_EVT_HIGHER when the input goes above the
				  reference, ANALOG_COMP_EVT_LOWER when it goes below

The internal reference needs CnLVL = 0 and VCC of 2.94 V or more (COMPMDR
note 2, the internal reference maximum plus 1.4 V). DAC8 needs VCC 2.7 V or
more (table 37.51). Other settings fail Init.

Low-speed mode (COMPOCR.SPDMD = 0, common to both comparators), output delay
3 us typical. No digital filter, as Software Standby needs (31.3.2 COMPFIR
note): the comparators can then wake the MCU, with
R9a02WakeupSet(R9A02_EVT_COMP_DETn). Table 10.2 note 7 permits only VCOUT in
Software Standby while tables 10.3 and 12.4 list COMP_DETn as wake-up
sources, to be checked on hardware. Both
edges raise the interrupt COMP_DETn, the handler reads the comparator output
(CnMON) for the event.

Set up as table 31.1. Init leaves the comparator running with its interrupt
off, Start turns it on: the requests made while the comparator settles are
dropped (table 31.1 note 2).

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

#include "r9a02g021.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/interrupt.h"
#include "converters/analog_comp.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "dac_r9a02.h"

#define R9A02_CMP_CNT				2
#define R9A02_CMP_SETTLE_US			30U			// tCMP, table 37.50
#define R9A02_CMP_LVL_MV			1400		// Reference range boundary, COMPMDR.CnLVL
#define R9A02_CMP_INTREF_VCC_MV		2940		// Internal reference maximum + 1.4 V

typedef struct __R9a02_Cmp_Pin {
	uint8_t Port;
	uint8_t Pin;
} R9a02CmpPin_t;

typedef struct __R9a02_Cmp_Dev {
	AnalogCompDev_t *pDev;			// Generic data
	int IntNo;						// COMP_DETn route, -1 when none
	uint8_t EvtId;					// COMP_DETn
	R9a02CmpPin_t Input;			// IVCMPn
	R9a02CmpPin_t Ref;				// IVREFn
	uint8_t Mdr;					// COMPMDR bits of the comparator, CnENB excluded
	uint8_t DacCode;				// CMP1 on DAC8 channel 0
	bool bDac;
	bool bOn;						// Comparator on
	bool bStarted;					// Interrupt on
} R9a02CmpDev_t;

// COMPMDR as written: it reads undefined with both comparators off after
// one was on (31.3.1 note 1), it is not read back
static uint8_t s_R9a02CmpMdr = 0;

static R9a02CmpDev_t s_R9a02CmpDev[R9A02_CMP_CNT] = {
	{ NULL, -1, R9A02_EVT_COMP_DET0, { 1, 6 }, { 0, 0 }, 0, 0, false, false, false },
	{ NULL, -1, R9A02_EVT_COMP_DET1, { 1, 7 }, { 1, 4 }, 0, 0, false, false, false },
};

static R9a02CmpDev_t *R9a02CmpHandle(AnalogCompDev_t *pDev)
{
	for (int i = 0; i < R9A02_CMP_CNT; i++)
	{
		if (pDev != NULL && s_R9a02CmpDev[i].pDev == pDev)
		{
			return &s_R9a02CmpDev[i];
		}
	}

	return NULL;
}

static int R9a02CmpNo(const R9a02CmpDev_t *pCmp)
{
	return pCmp == &s_R9a02CmpDev[0] ? 0 : 1;
}

static void R9a02CmpIrqHandler(int IntNo, void *pCtx)
{
	R9a02CmpDev_t *cmp = (R9a02CmpDev_t *)pCtx;
	int n = R9a02CmpNo(cmp);

	if (cmp->pDev == NULL || !cmp->bStarted || cmp->pDev->EvtHandler == NULL)
	{
		return;
	}

	ANALOG_COMP_EVT evt = (R9A02_CMP->COMPMDR & R9A02_COMPMDR_CMON(n)) ? ANALOG_COMP_EVT_HIGHER : ANALOG_COMP_EVT_LOWER;

	cmp->pDev->EvtHandler(cmp->pDev, evt);
}

// Interrupt request on or off. Turning it on can make a request, it is
// dropped (31.3.3).
static void R9a02CmpIrq(R9a02CmpDev_t * const pCmp, bool bOn)
{
	int n = R9a02CmpNo(pCmp);
	uintptr_t state = DisableInterrupt();

	if (bOn)
	{
		R9A02_CMP->COMPOCR = (uint8_t)(R9A02_CMP->COMPOCR | R9A02_COMPOCR_CIE(n));
	}
	else
	{
		R9A02_CMP->COMPOCR = (uint8_t)(R9A02_CMP->COMPOCR & ~R9A02_COMPOCR_CIE(n));
	}
	(void)R9A02_CMP->COMPOCR;
	R9a02ClearPendingInt(pCmp->IntNo);
	pCmp->bStarted = bOn;

	EnableInterrupt(state);
}

// Reference, comparator on, settling time, edges (table 31.1 steps 3 to 8)
static bool R9a02CmpOn(R9a02CmpDev_t * const pCmp)
{
	int n = R9a02CmpNo(pCmp);

	if (pCmp->bDac)
	{
		if (!R9a02DacWrite(0, pCmp->DacCode, pCmp) || !R9a02DacOutput(0, true, pCmp))
		{
			return false;
		}
		R9a02DelayUs(R9A02_DAC_SETTLE_US);
	}

	uintptr_t state = DisableInterrupt();
	R9A02_CMP_Type *reg = R9A02_CMP;
	uint8_t msk = (uint8_t)(R9A02_COMPMDR_CLVL(n) | R9A02_COMPMDR_CVRF(n));

	s_R9a02CmpMdr = (uint8_t)((s_R9a02CmpMdr & ~(msk | R9A02_COMPMDR_CENB(n))) | pCmp->Mdr);
	reg->COMPMDR = s_R9a02CmpMdr;
	s_R9a02CmpMdr = (uint8_t)(s_R9a02CmpMdr | R9A02_COMPMDR_CENB(n));
	reg->COMPMDR = s_R9a02CmpMdr;

	EnableInterrupt(state);

	R9a02DelayUs(R9A02_CMP_SETTLE_US);

	state = DisableInterrupt();

	// No filter, both edges
	reg->COMPFIR = (uint8_t)((reg->COMPFIR & ~(R9A02_COMPFIR_CFCK_Msk(n) | R9A02_COMPFIR_CEPO(n))) |
							 R9A02_COMPFIR_CEDG(n));
	pCmp->bOn = true;

	EnableInterrupt(state);

	return true;
}

// Interrupt and comparator off, DAC8 channel 0 output off, kept open
static void R9a02CmpOff(R9a02CmpDev_t * const pCmp)
{
	int n = R9a02CmpNo(pCmp);

	R9a02CmpIrq(pCmp, false);

	uintptr_t state = DisableInterrupt();

	s_R9a02CmpMdr = (uint8_t)(s_R9a02CmpMdr & ~R9A02_COMPMDR_CENB(n));
	R9A02_CMP->COMPMDR = s_R9a02CmpMdr;
	pCmp->bOn = false;

	EnableInterrupt(state);

	if (pCmp->bDac)
	{
		R9a02DacOutput(0, false, pCmp);
	}
}

static bool R9a02CmpEnable(AnalogCompDev_t *pDev)
{
	R9a02CmpDev_t *cmp = R9a02CmpHandle(pDev);

	if (cmp == NULL)
	{
		return false;
	}

	bool started = cmp->bStarted;

	if (!cmp->bOn && !R9a02CmpOn(cmp))
	{
		return false;
	}
	if (started)
	{
		R9a02CmpIrq(cmp, true);
	}

	return true;
}

// Comparator off, its supply current stops (31.5.3). Enable turns it back
// on as it was.
static void R9a02CmpDisable(AnalogCompDev_t *pDev)
{
	R9a02CmpDev_t *cmp = R9a02CmpHandle(pDev);

	if (cmp != NULL)
	{
		bool started = cmp->bStarted;

		R9a02CmpOff(cmp);
		cmp->bStarted = started;
	}
}

static bool R9a02CmpStart(AnalogCompDev_t *pDev)
{
	R9a02CmpDev_t *cmp = R9a02CmpHandle(pDev);

	if (cmp == NULL || (!cmp->bOn && !R9a02CmpOn(cmp)))
	{
		return false;
	}

	R9a02CmpIrq(cmp, true);

	return true;
}

static void R9a02CmpPowerOff(AnalogCompDev_t *pDev)
{
	R9a02CmpDev_t *cmp = R9a02CmpHandle(pDev);

	if (cmp == NULL)
	{
		return;
	}

	R9a02CmpOff(cmp);
	if (cmp->bDac)
	{
		R9a02DacClose(0, cmp);
		cmp->bDac = false;
	}

	uintptr_t state = DisableInterrupt();

	R9a02WakeupSet(cmp->EvtId, false);
	R9a02UnregisterIntHandler(cmp->IntNo);
	cmp->IntNo = -1;
	R9a02AnalogPinUse(cmp->Input.Port, cmp->Input.Pin, false);
	if ((cmp->Mdr & R9A02_COMPMDR_CVRF(R9a02CmpNo(cmp))) == 0U)
	{
		R9a02AnalogPinUse(cmp->Ref.Port, cmp->Ref.Pin, false);
	}
	cmp->pDev = NULL;
	cmp->bStarted = false;

	if (s_R9a02CmpDev[0].pDev == NULL && s_R9a02CmpDev[1].pDev == NULL)
	{
		R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_CMP);
	}

	EnableInterrupt(state);
}

static bool R9a02CmpInit(AnalogCompDev_t *pDev, const AnalogCompCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= R9A02_CMP_CNT ||
		pCfg->Mode != ANALOG_COMP_MODE_CONTINUOUS || pCfg->RefSrc < 0 || pCfg->RefSrc > 1 ||
		pCfg->RefVolt <= 0 || pCfg->AnalogIn != 0 || pCfg->bHystersys ||
		pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)
	{
		return false;
	}

	int n = pCfg->DevNo;
	R9a02CmpDev_t *cmp = &s_R9a02CmpDev[n];
	uint8_t mdr = 0;
	bool dac = false;
	uint8_t code = 0;

	if (pCfg->RefSrc == 0 && n == 0)
	{
		// Internal reference: CnLVL = 0, VCC high enough (COMPMDR note 2)
		if (pCfg->RefVolt < R9A02_CMP_INTREF_VCC_MV)
		{
			return false;
		}
		mdr = R9A02_COMPMDR_CVRF(0);
	}
	else
	{
		// Range of the reference, CnLVL (table 37.50)
		if (pCfg->CompVolt < 0 || pCfg->CompVolt > pCfg->RefVolt)
		{
			return false;
		}
		if (pCfg->CompVolt > pCfg->RefVolt - R9A02_CMP_LVL_MV)
		{
			if (pCfg->CompVolt < R9A02_CMP_LVL_MV)
			{
				return false;
			}
			mdr = R9A02_COMPMDR_CLVL(n);
		}

		if (pCfg->RefSrc == 0)
		{
			// CMP1 on DAC8 channel 0
			uint32_t c = ((uint32_t)pCfg->CompVolt * 256U + (uint32_t)pCfg->RefVolt / 2U) / (uint32_t)pCfg->RefVolt;

			code = c > 255U ? 255U : (uint8_t)c;
			mdr |= R9A02_COMPMDR_CVRF(1);
			dac = true;
		}
	}

	uintptr_t state = DisableInterrupt();

	if ((cmp->pDev != NULL && cmp->pDev != pDev) || s_R9a02CmpDev[1 - n].pDev == pDev)
	{
		EnableInterrupt(state);

		return false;
	}

	if (cmp->pDev == pDev)
	{
		EnableInterrupt(state);
		R9a02CmpPowerOff(pDev);
		state = DisableInterrupt();
	}

	if (s_R9a02CmpDev[1 - n].pDev == NULL)
	{
		s_R9a02CmpMdr = 0U;
	}
	R9a02ModuleStart(R9A02_MSTPCR_D, R9A02_MSTPCRD_CMP);

	// DAC8 channel 0 kept from here to PowerOff (31.5.5)
	if (dac && !R9a02DacOpen(0, false, cmp))
	{
		if (s_R9a02CmpDev[1 - n].pDev == NULL)
		{
			R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_CMP);
		}
		EnableInterrupt(state);

		return false;
	}

	cmp->IntNo = R9a02RegisterIntHandler(cmp->EvtId, pCfg->IntPrio, R9a02CmpIrqHandler, cmp);
	if (cmp->IntNo < 0)
	{
		if (dac)
		{
			R9a02DacClose(0, cmp);
		}
		if (s_R9a02CmpDev[1 - n].pDev == NULL)
		{
			R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_CMP);
		}
		EnableInterrupt(state);

		return false;
	}

	cmp->pDev = pDev;
	cmp->Mdr = mdr;
	cmp->DacCode = code;
	cmp->bDac = dac;
	cmp->bOn = false;
	cmp->bStarted = false;

	// Interrupt request off before the set up
	R9A02_CMP->COMPOCR = (uint8_t)(R9A02_CMP->COMPOCR & ~(R9A02_COMPOCR_CIE(n) | R9A02_COMPOCR_COE(n)));

	pDev->Mode = pCfg->Mode;
	pDev->RefSrc = pCfg->RefSrc;
	pDev->ExtVRefPin = (uint16_t)((cmp->Ref.Port << 8) | cmp->Ref.Pin);
	pDev->CompVolt = pCfg->CompVolt;
	pDev->AnalogIn = pCfg->AnalogIn;
	pDev->bHystersys = false;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->Enable = R9a02CmpEnable;
	pDev->Disable = R9a02CmpDisable;
	pDev->Start = R9a02CmpStart;
	pDev->PowerOff = R9a02CmpPowerOff;

	EnableInterrupt(state);

	// Analog pins (table 31.1 step 2), shared with the A/D converter and
	// DAC8 (P000, P106)
	R9a02AnalogPinUse(cmp->Input.Port, cmp->Input.Pin, true);
	if (pCfg->RefSrc == 1)
	{
		R9a02AnalogPinUse(cmp->Ref.Port, cmp->Ref.Pin, true);
	}

	if (!R9a02CmpOn(cmp))
	{
		R9a02CmpPowerOff(pDev);

		return false;
	}

	return true;
}

bool AnalogCompInit(AnalogCompDev_t *pDev, AnalogCompCfg_t *pCfg)
{
	return R9a02CmpInit(pDev, pCfg);
}

bool AnalogComp::Init(const AnalogCompCfg_t &Cfg)
{
	vDevData.pPrivate = this;

	return R9a02CmpInit(&vDevData, &Cfg);
}
