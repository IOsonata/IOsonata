/**-------------------------------------------------------------------------
@file	adc_r9a02.cpp

@brief	R9A02G021 12-bit A/D converter (ADC12)

R9A02G021 User's Manual R01UH1036EJ0110, chapter 29. See adc_r9a02.h for
the channel numbering.

One-shot conversion in every mode. Software trigger no-wait mode (29.4,
table 29.14): the comparator stays on (ADCE = 1) and each ADCS = 1 converts
the channel of ADS, or the scan group, once, ADC_ENDI marks the end.
Hardware trigger no-wait mode (table 29.16): ADCS = 1 arms the trigger, the
converter stays armed after each conversion (29.6.10, 29.6.12). Snooze,
hardware trigger wait mode (table 29.20): ADCE = 1 arms the trigger, the
converter is powered at each trigger. Without interrupt ADC_ENDI is routed
to an IELSR without a CPU interrupt and its IR flag is polled, the ADC12 has
no status flag of its own.

Mode, clock, compare and AWC are written with ADCS = ADCE = 0, then 5 us
before ADCE = 1 (29.3.1). In wait mode at PCLKB above 32 MHz only fAD =
PCLKB / 32 is allowed (table 29.9).

Conversion clock fAD, at most 32 MHz: PCLKB / 2 at PCLKB 48 MHz, PCLKB from
4 to 32 MHz, ADLSP = 1 below 4 MHz (tables 29.9 and 29.10). Table 37.42 gives
the characteristics from VCC 2.4 V with PCLKB at most 32 MHz, PCLKB 48 MHz is
characterized from VCC 2.7 V only (table 37.41). Sampling is 43 fAD in normal
mode 1 or 160 fAD in normal mode 2 (table 29.3). OpenChannel takes the
fastest fAD and mode whose sampling time covers the AcqTime of every open
channel. The temperature sensor and the internal reference need normal mode
2 and 5 us (table 37.40 note 4).

Reference: VCC or AVREFP (P002, ANI0 cannot be converted then), '-' side
VSS. The internal reference as '+' side is not supported (low voltage modes,
8-bit only), Init fails with it.

Results: 12-bit right aligned, 10 and 8-bit left aligned in ADCRn (29.3.4),
returned right aligned and in volts, V = Raw x Vref / 2^n. Calibrate measures
the zero and full scale through ADTES and corrects the later results. A
port register write during a conversion can corrupt the result (29.10).

ANI18 and ANI19: the pin tables (1.7, 16.6) give P105 and P106, table 29.12
the opposite. The pin tables are used here.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include <string.h>

#include "r9a02g021.h"
#include "cfifo.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "elc_r9a02.h"
#include "adc_r9a02.h"

#define R9A02_ADC_CLOSED			0xFFU		// vAds of a closed channel
#define R9A02_ADC_FAD_MAX			32000000UL	// Tables 29.9, 29.10
#define R9A02_ADC_PCLKB_LOW			4000000UL	// ADLSP = 1 at or below
#define R9A02_ADC_INTERNAL_US		5U			// Sampling of the internal inputs, table 37.40 note 4
#define R9A02_ADC_STOP_US			1U			// After ADCE = 0, 0.2 us (29.3.1)
#define R9A02_ADC_SETUP_US			5U			// Mode change and stop to start (29.3.1)
#define R9A02_ADC_SCAN_CNT			4			// Channels of a scan group (table 29.13)
#define R9A02_ADC_SCAN_LAST			2			// ADS of the last scan group
#define R9A02_ADC_ANI_LAST			5			// Last input of the scan groups

static_assert(offsetof(R9A02_ADC12_Type, ADCR) == 0x06, "ADCR at 0x06");
static_assert(offsetof(R9A02_ADC12_Type, ADM2) == 0x90, "ADM2 at 0x90");
static_assert(offsetof(R9A02_ADC12_Type, ADCRN) == 0xA0, "ADCR0 at 0xA0");

typedef struct __R9a02_Ani_Pin {
	uint8_t Ani;
	uint8_t Port;
	uint8_t Pin;
} R9a02AniPin_t;

// Analog inputs of the 48-pin package, tables 1.13 and 16.7
static const R9a02AniPin_t s_R9a02AniPin[] = {
	{ 0, 0, 2 }, { 1, 0, 3 }, { 2, 4, 0 }, { 3, 4, 1 }, { 4, 0, 6 }, { 5, 0, 7 },
	{ 16, 0, 0 }, { 17, 0, 1 }, { 18, 1, 5 }, { 19, 1, 6 },
};

static const R9a02AniPin_t *R9a02AdcAniPin(int Ani)
{
	for (size_t i = 0; i < sizeof(s_R9a02AniPin) / sizeof(s_R9a02AniPin[0]); i++)
	{
		if (s_R9a02AniPin[i].Ani == Ani)
		{
			return &s_R9a02AniPin[i];
		}
	}

	return NULL;
}

static void R9a02AdcIrqHandler(int IntNo, void *pCtx)
{
	((AdcR9a02 *)pCtx)->IntHandler();
}

static void R9a02AdcTimerHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	((AdcR9a02 *)pContext)->TimerHandler();
}

static void R9a02AdcStandbyHook(bool bEnter, void *pCtx)
{
	((AdcR9a02 *)pCtx)->StandbyHook(bEnter);
}

// Comparator and conversion off, ADCS first (29.3.1)
static void R9a02AdcStop(void)
{
	R9A02_ADC12->ADM0 = (uint8_t)(R9A02_ADC12->ADM0 & ~R9A02_ADM0_ADCS);
	R9A02_ADC12->ADM0 = (uint8_t)(R9A02_ADC12->ADM0 & ~R9A02_ADM0_ADCE);
	R9a02DelayUs(R9A02_ADC_STOP_US);
}

// Result register value right aligned
static uint16_t R9a02AdcRaw(uint16_t d, uint8_t Adm2)
{
	switch (Adm2 & 3U)
	{
		case R9A02_ADM2_ADTYP_12BIT:
			return (uint16_t)(d & 0x0FFFU);
		case R9A02_ADM2_ADTYP_8BIT:
			return (uint16_t)(d >> 8);
		default:
			return (uint16_t)(d >> 6);
	}
}

// 1 us + 2 fAD before the first ADCS = 1 (29.7.1)
static void R9a02AdcStartWait(uint8_t Adm0)
{
	uint32_t fad = SystemPeriphClockGet(0) >> (R9A02_ADM0_FR_DIV1 - ((Adm0 & R9A02_ADM0_FR_Msk) >> R9A02_ADM0_FR_Pos));

	R9a02DelayUs(1U + (fad ? 2000000UL / fad : 0U) + 1U);
}

AdcR9a02::AdcR9a02()
{
	memset(vAds, R9A02_ADC_CLOSED, sizeof(vAds));
	memset(vAcqNs, 0, sizeof(vAcqNs));
	memset(vRaw, 0, sizeof(vRaw));
	memset(vStamp, 0, sizeof(vStamp));
	memset(vbNew, 0, sizeof(vbNew));
	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		vhFifo[i] = NULL;
	}
	vRefVolt = 0.0f;
	vAdm0 = 0;
	vAdm1 = 0;
	vAdm2 = 0;
	vFadHz = 0;
	vLoops = 0;
	vIntNo = -1;
	vCurChan = -1;
	vZero = 0;
	vFull = 0;
	vbCal = false;
	vbInternal = false;
	vbDiscard = false;
	vpTimer = NULL;
	vTimerTrig = -1;
	vRound = 0;
	vScan = -1;
	vTrig = R9A02_ADC_TRIG_SOFT;
	vElcEvt = 0;
	vAdul = 0xFF;
	vAdll = 0;
	vbWindow = false;
	vbOutside = false;
	vbSnooze = false;
	vbArmed = false;
	vbRunning = false;
	vpRefVolt = NULL;
	vNbRefVolt = 0;
	vResolution = 12;
	vRate = 0;
	vMode = ADC_CONV_MODE_SINGLE;
	vbInterrupt = false;
	vIntPrio = 0;
}

AdcR9a02::~AdcR9a02()
{
}

// ADM0 for the conversions: clock, voltage mode, scan. The wait mode of
// Snooze at PCLKB above 32 MHz takes fAD = PCLKB / 32 (table 29.9).
uint8_t AdcR9a02::Adm0()
{
	uint8_t adm0 = (uint8_t)(vAdm0 | (vScan >= 0 ? R9A02_ADM0_ADMD : 0U));

	if (vbSnooze && SystemPeriphClockGet(0) > R9A02_ADC_FAD_MAX)
	{
		adm0 &= (uint8_t)~R9A02_ADM0_FR_Msk;
	}

	return adm0;
}

// Clock, mode, trigger and window set up, then the comparator on (tables
// 29.14, 29.16). The wait mode of Snooze leaves ADCE = 0: AWC and ADCE are
// set just before Software Standby and cleared after it (table 29.20 steps 3,
// 4 and 11), by StandbyHook.
bool AdcR9a02::Configure()
{
	R9A02_ADC12_Type *reg = R9A02_ADC12;
	uint8_t adm0 = Adm0();
	uint8_t adm1 = vAdm1;
	uint8_t adm2 = vAdm2;
	bool window = HwTrig() && vbWindow;

	R9a02AdcStop();

	if (vbSnooze)
	{
		adm1 |= R9A02_ADM1_ADTMD_HW_WAIT | R9A02_ADM1_ADTRS_ELC;
	}
	else if (vTrig != R9A02_ADC_TRIG_SOFT)
	{
		uint8_t trs = vTrig == R9A02_ADC_TRIG_TAU0_CH1 ? R9A02_ADM1_ADTRS_TAU0_ENDI1 :
					  vTrig == R9A02_ADC_TRIG_RTC ? R9A02_ADM1_ADTRS_RTC :
					  vTrig == R9A02_ADC_TRIG_TML32 ? R9A02_ADM1_ADTRS_TML32 : R9A02_ADM1_ADTRS_ELC;

		adm1 |= R9A02_ADM1_ADTMD_HW_NOWAIT | trs;
	}
	if (window && vbOutside)
	{
		adm2 |= R9A02_ADM2_ADRCK;
	}

	reg->ADM0 = adm0;
	reg->ADM1 = adm1;
	reg->ADM2 = adm2;
	reg->ADUL = window ? vAdul : 0xFFU;
	reg->ADLL = window ? vAdll : 0U;
	reg->ADTES = 0U;
	reg->ADS = vScan >= 0 ? (uint8_t)vScan : 0U;
	vbInternal = false;
	vbDiscard = false;
	vbArmed = false;
	R9a02DelayUs(R9A02_ADC_SETUP_US);

	if (!vbSnooze)
	{
		reg->ADM0 = (uint8_t)(adm0 | R9A02_ADM0_ADCE);
		R9a02AdcStartWait(adm0);
	}

	return true;
}

int AdcR9a02::NextChan(int From)
{
	for (int i = From; i < R9A02_ADC_MAX_CHAN; i++)
	{
		if (vAds[i] != R9A02_ADC_CLOSED)
		{
			return i;
		}
	}

	return -1;
}

// Scan when the open channels are exactly a scan group, ANI0 not with
// AVREFP as the reference (table 29.13, 29.3.6)
void AdcR9a02::UpdateScan()
{
	bool extref = (vAdm2 & R9A02_ADM2_ADREFP_DISCHARGE) == R9A02_ADM2_ADREFP_AVREFP;
	uint32_t mask = 0;

	vScan = -1;

	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		if (vAds[i] == R9A02_ADC_CLOSED)
		{
			continue;
		}
		if ((vAds[i] & R9A02_ADS_ADISS) || vAds[i] > R9A02_ADC_ANI_LAST)
		{
			return;
		}
		mask |= 1UL << vAds[i];
	}

	for (int g = extref ? 1 : 0; g <= R9A02_ADC_SCAN_LAST; g++)
	{
		if (mask == (((1UL << R9A02_ADC_SCAN_CNT) - 1U) << g))
		{
			vScan = g;
			break;
		}
	}
}

// ADS of the channel, then ADCS. ADISS changes with the comparator off, the
// first result after ADISS = 1 is not valid (29.3.6). Select mode only.
bool AdcR9a02::StartOne(int Chan)
{
	R9A02_ADC12_Type *reg = R9A02_ADC12;
	uint8_t ads = vAds[Chan];
	bool internal = (ads & R9A02_ADS_ADISS) != 0U;
	uint8_t adm0 = Adm0();

	if (internal != vbInternal)
	{
		R9a02AdcStop();
		reg->ADS = ads;
		R9a02DelayUs(R9A02_ADC_SETUP_US);
		reg->ADM0 = (uint8_t)(adm0 | R9A02_ADM0_ADCE);
		R9a02AdcStartWait(adm0);
		vbInternal = internal;
		vbDiscard = internal;
	}
	else
	{
		// The manual does not cover a change between the two internal
		// inputs, its first result is not used either
		if (internal && reg->ADS != ads)
		{
			vbDiscard = true;
		}
		reg->ADS = ads;
	}

	R9a02ClearPendingInt(vIntNo);
	reg->ADM0 = (uint8_t)(adm0 | R9A02_ADM0_ADCE | R9A02_ADM0_ADCS);

	return true;
}

// A result of a channel, into its FIFO when it has one
void AdcR9a02::Store(int Chan, uint16_t Raw)
{
	uint32_t stamp = 0;

	if (vMode == ADC_CONV_MODE_CONTINUOUS || HwTrig())
	{
		stamp = vpTimer != NULL ? vpTimer->uSecond() : vRound;
	}

	vRaw[Chan] = Raw;
	vStamp[Chan] = stamp;
	vbNew[Chan] = true;

	if (vhFifo[Chan] != NULL)
	{
		AdcData_t *p = (AdcData_t *)CFifoPut(vhFifo[Chan]);

		if (p != NULL)
		{
			p->Timestamp = stamp;
			p->Chan = Chan;
			p->Data = ToVolt(Chan, Raw);
		}
	}
}

// Results of a scan, ADCRn of scan n (29.3.4)
void AdcR9a02::StoreScan()
{
	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		if (vAds[i] != R9A02_ADC_CLOSED)
		{
			Store(i, R9a02AdcRaw(R9A02_ADC12->ADCRN[vAds[i] - vScan], vAdm2));
		}
	}
}

// One channel, waiting for ADC_ENDI. Converted twice after an ADISS change.
bool AdcR9a02::ConvertOne(int Chan)
{
	for (int i = 0; i < 2; i++)
	{
		uint32_t n = vLoops;

		StartOne(Chan);

		while (!R9a02EventFlag(vIntNo))
		{
			if (--n == 0U)
			{
				R9A02_ADC12->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE);

				return false;
			}
		}

		uint16_t raw = R9a02AdcRaw(R9A02_ADC12->ADCRN[0], vAdm2);

		if (vbDiscard)
		{
			vbDiscard = false;
			continue;
		}

		Store(Chan, raw);

		return true;
	}

	return false;
}

// The scan group, waiting for ADC_ENDI after the fourth conversion
bool AdcR9a02::ConvertScan()
{
	uint32_t n = vLoops * R9A02_ADC_SCAN_CNT;

	R9a02ClearPendingInt(vIntNo);
	R9A02_ADC12->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE | R9A02_ADM0_ADCS);

	while (!R9a02EventFlag(vIntNo))
	{
		if (--n == 0U)
		{
			R9A02_ADC12->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE);

			return false;
		}
	}

	StoreScan();

	return true;
}

float AdcR9a02::ToVolt(int Chan, uint32_t Raw)
{
	uint32_t full = (1UL << vResolution) - 1U;
	float v = (float)Raw;

	if (vbCal && vFull > vZero)
	{
		v = ((float)Raw - (float)vZero) * (float)full / (float)(vFull - vZero);
		if (v < 0.0f)
		{
			v = 0.0f;
		}
	}

	return v * vRefVolt / (float)(full + 1U);
}

// First conversion of a round, software trigger, interrupts off
bool AdcR9a02::StartRound()
{
	int first = NextChan(0);

	if (first < 0)
	{
		return false;
	}

	vCurChan = first;

	if (vScan >= 0)
	{
		R9a02ClearPendingInt(vIntNo);
		R9A02_ADC12->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE | R9A02_ADM0_ADCS);

		return true;
	}

	return StartOne(first);
}

// Round done: event, then the next round of the continuous mode without a
// timer
void AdcR9a02::EndRound()
{
	vCurChan = -1;
	vRound++;

	EvtHandler(DEV_EVT_DATA_RDY);

	if (vMode == ADC_CONV_MODE_CONTINUOUS && vbRunning && vTimerTrig < 0)
	{
		StartRound();
	}
}

// ADC_ENDI with a hardware trigger: the scan, or the channel of ADS and the
// next channel selected for the next trigger
void AdcR9a02::HwEnd()
{
	if (!vbArmed)
	{
		return;
	}

	if (vScan >= 0)
	{
		StoreScan();
		vRound++;
		EvtHandler(DEV_EVT_DATA_RDY);

		return;
	}

	int chan = vCurChan;

	if (chan < 0)
	{
		return;
	}

	Store(chan, R9a02AdcRaw(R9A02_ADC12->ADCRN[0], vAdm2));

	int next = NextChan(chan + 1);
	bool end = next < 0;

	if (end)
	{
		next = NextChan(0);
	}
	if (next >= 0 && next != chan)
	{
		vCurChan = next;
		R9A02_ADC12->ADS = vAds[next];
	}
	if (end)
	{
		vRound++;
		EvtHandler(DEV_EVT_DATA_RDY);
	}
}

void AdcR9a02::IntHandler()
{
	if (HwTrig())
	{
		HwEnd();

		return;
	}

	int chan = vCurChan;

	if (chan < 0)
	{
		return;
	}

	if (vScan >= 0)
	{
		StoreScan();
		EndRound();

		return;
	}

	uint16_t raw = R9a02AdcRaw(R9A02_ADC12->ADCRN[0], vAdm2);

	if (vbDiscard)
	{
		vbDiscard = false;
		StartOne(chan);

		return;
	}

	Store(chan, raw);

	int next = NextChan(chan + 1);

	if (next >= 0)
	{
		vCurChan = next;
		StartOne(next);

		return;
	}

	EndRound();
}

void AdcR9a02::TimerHandler()
{
	uintptr_t state = DisableInterrupt();

	// A round still running when the next one is due is not restarted
	if (vbRunning && vCurChan < 0)
	{
		StartRound();
	}

	EnableInterrupt(state);
}

// Snooze: AWC = 1 then ADCE = 1 just before Software Standby, the converter
// waits for the trigger (table 29.20 steps 3 and 4). After the wake-up, the
// result read in the interrupt, the converter stopped and AWC = 0, else the
// next conversions do not start (step 11, note 2). ADM2 is written with ADCS
// = ADCE = 0 (29.3.3).
void AdcR9a02::StandbyHook(bool bEnter)
{
	if (!vbSnooze || !vbArmed)
	{
		return;
	}

	R9A02_ADC12_Type *reg = R9A02_ADC12;

	R9a02AdcStop();

	if (bEnter)
	{
		reg->ADM2 = (uint8_t)(reg->ADM2 | R9A02_ADM2_AWC);
		reg->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE);
	}
	else
	{
		reg->ADM2 = (uint8_t)(reg->ADM2 & ~R9A02_ADM2_AWC);
	}
}

// Hardware trigger armed: ELC link, first channel, then ADCS = 1 (no-wait,
// table 29.16). In Snooze the converter stays off until StandbyHook.
// Interrupts off.
bool AdcR9a02::Arm()
{
	R9A02_ADC12_Type *reg = R9A02_ADC12;
	int first = NextChan(0);

	if (first < 0)
	{
		return false;
	}

	if ((vbSnooze && !R9a02ElcLink(R9A02_ELC_DEST_ADC, R9A02_EVT_SYSTEM_SNZREQ)) ||
		(!vbSnooze && vTrig == R9A02_ADC_TRIG_ELC && !R9a02ElcLink(R9A02_ELC_DEST_ADC, vElcEvt)))
	{
		return false;
	}

	vCurChan = first;
	reg->ADS = vScan >= 0 ? (uint8_t)vScan : vAds[first];
	R9a02ClearPendingInt(vIntNo);

	if (!vbSnooze)
	{
		reg->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE | R9A02_ADM0_ADCS);
	}
	vbArmed = true;

	return true;
}

// Trigger off: ADCS = 0, conversion standby (no-wait), or the converter off
// (wait). Interrupts off.
void AdcR9a02::Disarm()
{
	if (vbSnooze)
	{
		R9a02AdcStop();
		R9A02_ADC12->ADM2 = (uint8_t)(R9A02_ADC12->ADM2 & ~R9A02_ADM2_AWC);
	}
	else
	{
		R9A02_ADC12->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE);
	}

	if (vbSnooze || vTrig == R9A02_ADC_TRIG_ELC)
	{
		R9a02ElcLink(R9A02_ELC_DEST_ADC, 0U);
	}

	R9a02ClearPendingInt(vIntNo);
	vbArmed = false;
	vCurChan = -1;
}

bool AdcR9a02::Init(const AdcCfg_t &Cfg, Timer * const pTimer, DeviceIntrf * const pIntrf)
{
	if ((Cfg.Mode != ADC_CONV_MODE_SINGLE && Cfg.Mode != ADC_CONV_MODE_CONTINUOUS) ||
		(Cfg.Mode == ADC_CONV_MODE_CONTINUOUS && (!Cfg.bInterrupt || (Cfg.Rate > 0 && pTimer == NULL))) ||
		Cfg.Rate < 0 || Cfg.pRefVolt == NULL || Cfg.NbRefVolt != 1 ||
		(Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_SUPPLY && Cfg.pRefVolt[0].Type != ADC_REFVOLT_TYPE_EXTERNAL) ||
		Cfg.pRefVolt[0].Voltage <= 0.0f ||
		(Cfg.bInterrupt && (Cfg.IntPrio < 0 || Cfg.IntPrio > (int)R9A02_CLIC_PRIO_MAX)))
	{
		return false;
	}

	uint32_t pclk = SystemPeriphClockGet(0);

	// PCLKB 1 to 32 MHz or 48 MHz (tables 29.9 and 29.10)
	if (pclk < 1000000UL || (pclk > R9A02_ADC_FAD_MAX && pclk != 48000000UL))
	{
		return false;
	}

	if (vIntNo >= 0)
	{
		// Initialized before: its pins released, they can be shared
		StopConversion();
		for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
		{
			CloseChannel(i);
		}
		if ((vAdm2 & R9A02_ADM2_ADREFP_DISCHARGE) == R9A02_ADM2_ADREFP_AVREFP)
		{
			R9a02AnalogPinUse(0, 2, false);
		}
	}
	R9a02StandbyHookSet(R9a02AdcStandbyHook, this, false);

	uintptr_t state = DisableInterrupt();

	if (vIntNo >= 0)
	{
		R9a02UnregisterIntHandler(vIntNo);
		vIntNo = -1;
	}

	R9a02ModuleStart(R9A02_MSTPCR_D, R9A02_MSTPCRD_ADC12);
	R9a02AdcStop();

	if (Cfg.bInterrupt)
	{
		vIntNo = R9a02RegisterIntHandler(R9A02_EVT_ADC_ENDI, Cfg.IntPrio, R9a02AdcIrqHandler, this);
	}
	else
	{
		vIntNo = R9a02RouteEvent(R9A02_EVT_ADC_ENDI);
	}

	if (vIntNo < 0)
	{
		R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_ADC12);
		EnableInterrupt(state);

		return false;
	}

	SetRefVoltage(Cfg.pRefVolt, Cfg.NbRefVolt);
	vRefVolt = Cfg.pRefVolt[0].Voltage;
	vMode = Cfg.Mode;
	vbInterrupt = Cfg.bInterrupt;
	vIntPrio = Cfg.IntPrio;
	vRate = (uint32_t)Cfg.Rate;
	vpTimer = pTimer;
	vTimerTrig = -1;
	vRound = 0;
	vCurChan = -1;
	vbCal = false;
	vScan = -1;
	vTrig = R9A02_ADC_TRIG_SOFT;
	vElcEvt = 0;
	vbWindow = false;
	vbOutside = false;
	vbSnooze = false;
	vbArmed = false;
	vbRunning = false;
	memset(vAds, R9A02_ADC_CLOSED, sizeof(vAds));
	memset(vAcqNs, 0, sizeof(vAcqNs));
	memset(vbNew, 0, sizeof(vbNew));
	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		vhFifo[i] = NULL;
	}
	SetEvtHandler(Cfg.EvtHandler);

	if (Cfg.pRefVolt[0].Type == ADC_REFVOLT_TYPE_EXTERNAL)
	{
		// AVREFP on P002 as analog input (29.3.5)
		R9a02AnalogPinUse(0, 2, true);
		vAdm2 = R9A02_ADM2_ADREFP_AVREFP;
	}
	else
	{
		vAdm2 = R9A02_ADM2_ADREFP_VCC;
	}

	// Fastest clock, normal mode 1 until channels ask for more sampling time
	vAdm1 = R9A02_ADM1_ADSCM;
	if (pclk <= R9A02_ADC_PCLKB_LOW)
	{
		vAdm1 |= R9A02_ADM1_ADLSP;
	}
	uint32_t div = pclk > R9A02_ADC_FAD_MAX ? 1U : 0U;

	vFadHz = pclk >> div;
	vAdm0 = (uint8_t)(((R9A02_ADM0_FR_DIV1 - div) << R9A02_ADM0_FR_Pos) | R9A02_ADM0_LV_NORMAL1);
	vLoops = SystemCoreClock / 1000U + 10000U;

	EnableInterrupt(state);

	Resolution(Cfg.Resolution);

	return Configure();
}

uint16_t AdcR9a02::Resolution(uint16_t Val)
{
	uint8_t typ;

	if (Val >= 11U)
	{
		vResolution = 12;
		typ = R9A02_ADM2_ADTYP_12BIT;
	}
	else if (Val >= 9U)
	{
		vResolution = 10;
		typ = R9A02_ADM2_ADTYP_10BIT;
	}
	else
	{
		vResolution = 8;
		typ = R9A02_ADM2_ADTYP_8BIT;
	}

	if ((vAdm2 & 3U) != typ)
	{
		// The calibration is in units of the old resolution
		vAdm2 = (uint8_t)((vAdm2 & ~3U) | typ);
		vbCal = false;
		if (vCurChan < 0 && !vbArmed && !vbRunning && vIntNo >= 0)
		{
			Configure();
		}
	}

	return vResolution;
}

bool AdcR9a02::OpenChannel(const AdcChanCfg_t * const pChanCfg, int NbChan)
{
	if (pChanCfg == NULL || NbChan <= 0 || vIntNo < 0 || vCurChan >= 0 || vbArmed || vbRunning)
	{
		return false;
	}

	bool extref = (vAdm2 & R9A02_ADM2_ADREFP_DISCHARGE) == R9A02_ADM2_ADREFP_AVREFP;
	uint32_t acqns = 0;
	bool internal = false;
	uint32_t chacq[R9A02_ADC_MAX_CHAN];
	bool chopen[R9A02_ADC_MAX_CHAN];

	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		chacq[i] = vAcqNs[i];
		chopen[i] = vAds[i] != R9A02_ADC_CLOSED;
		if (chopen[i] && (vAds[i] & R9A02_ADS_ADISS))
		{
			internal = true;
		}
	}

	for (int i = 0; i < NbChan; i++)
	{
		const AdcChanCfg_t *c = &pChanCfg[i];
		int in = c->PinP.PinNo;

		// The internal inputs are converted with the software trigger only
		// (29.3.6), the FIFO memory is word aligned (cfifo.h)
		if (c->Chan < 0 || c->Chan >= R9A02_ADC_MAX_CHAN || c->Type != ADC_CHAN_TYPE_SINGLE_ENDED ||
			c->RefVoltIdx != 0 || c->PinP.Conn != ADC_PIN_CONN_NONE ||
			(in != R9A02_ADC_INPUT_TEMP && in != R9A02_ADC_INPUT_VREF && R9a02AdcAniPin(in) == NULL) ||
			(extref && in == 0) ||
			((in == R9A02_ADC_INPUT_TEMP || in == R9A02_ADC_INPUT_VREF) && HwTrig()) ||
			(c->pFifoMem != NULL && ((uintptr_t)c->pFifoMem & 3U) != 0U))
		{
			return false;
		}

		uint32_t ns = c->AcqTime > 0 ? (uint32_t)c->AcqTime * 1000U : 0U;

		if (in == R9A02_ADC_INPUT_TEMP || in == R9A02_ADC_INPUT_VREF)
		{
			internal = true;
			if (ns < R9A02_ADC_INTERNAL_US * 1000U)
			{
				ns = R9A02_ADC_INTERNAL_US * 1000U;
			}
		}
		chacq[c->Chan] = ns;
		chopen[c->Chan] = true;
	}

	// The sampling time of every open channel, the ones opened before
	// included
	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		if (chopen[i] && chacq[i] > acqns)
		{
			acqns = chacq[i];
		}
	}

	// Fastest fAD and mode with enough sampling time. fAD = PCLKB / 2^div,
	// FR = 5 - div, the dividers allowed depend on PCLKB (tables 29.9, 29.10).
	uint32_t pclk = SystemPeriphClockGet(0);
	uint32_t divmin = pclk > R9A02_ADC_FAD_MAX ? 1U : 0U;
	uint32_t divmax = pclk <= R9A02_ADC_PCLKB_LOW ? (pclk >= 4000000UL ? 2U : pclk >= 2000000UL ? 1U : 0U) :
					  pclk >= 32000000UL ? 5U : pclk >= 16000000UL ? 4U : pclk >= 8000000UL ? 3U : 2U;
	bool found = false;
	uint8_t adm0 = 0;
	uint32_t fad = 0;

	for (uint32_t div = divmin; div <= divmax && !found; div++)
	{
		fad = pclk >> div;

		for (uint32_t lv = 0; lv < 2U && !found; lv++)
		{
			uint32_t cycles = lv ? R9A02_ADC_LV_NORMAL2_SAMPLING : R9A02_ADC_LV_NORMAL1_SAMPLING;

			// Internal inputs need normal mode 2 (table 37.40 note 4)
			if (internal && lv == 0U)
			{
				continue;
			}
			if ((uint64_t)cycles * 1000000000ULL >= (uint64_t)acqns * fad)
			{
				adm0 = (uint8_t)(((R9A02_ADM0_FR_DIV1 - div) << R9A02_ADM0_FR_Pos) |
								 (lv ? R9A02_ADM0_LV_NORMAL2 : R9A02_ADM0_LV_NORMAL1));
				found = true;
			}
		}
	}

	if (!found)
	{
		return false;
	}

	for (int i = 0; i < NbChan; i++)
	{
		const AdcChanCfg_t *c = &pChanCfg[i];
		int in = c->PinP.PinNo;

		// A channel open before: the pin of its input released
		if (vAds[c->Chan] != R9A02_ADC_CLOSED && (vAds[c->Chan] & R9A02_ADS_ADISS) == 0U)
		{
			const R9a02AniPin_t *p = R9a02AdcAniPin(vAds[c->Chan]);

			if (p != NULL)
			{
				R9a02AnalogPinUse(p->Port, p->Pin, false);
			}
		}

		if (in == R9A02_ADC_INPUT_TEMP)
		{
			vAds[c->Chan] = R9A02_ADS_TEMP;
		}
		else if (in == R9A02_ADC_INPUT_VREF)
		{
			vAds[c->Chan] = R9A02_ADS_VREF;
		}
		else
		{
			const R9a02AniPin_t *p = R9a02AdcAniPin(in);

			R9a02AnalogPinUse(p->Port, p->Pin, true);
			vAds[c->Chan] = (uint8_t)in;
		}
		vAcqNs[c->Chan] = chacq[c->Chan];
		vbNew[c->Chan] = false;

		// FIFO of the results, the oldest pushed out when full
		vhFifo[c->Chan] = NULL;
		if (c->pFifoMem != NULL && c->FifoMemSize >= (int)CFIFO_TOTAL_MEMSIZE(1, sizeof(AdcData_t)))
		{
			vhFifo[c->Chan] = CFifoInit(c->pFifoMem, (uint32_t)c->FifoMemSize, sizeof(AdcData_t), false);
		}
	}

	vAdm0 = adm0;
	vFadHz = fad;
	UpdateScan();

	return Configure();
}

void AdcR9a02::CloseChannel(int Chan)
{
	if (Chan < 0 || Chan >= R9A02_ADC_MAX_CHAN || vAds[Chan] == R9A02_ADC_CLOSED || vCurChan >= 0 ||
		vbArmed || vbRunning)
	{
		return;
	}

	if ((vAds[Chan] & R9A02_ADS_ADISS) == 0U)
	{
		const R9a02AniPin_t *p = R9a02AdcAniPin(vAds[Chan]);

		if (p != NULL)
		{
			R9a02AnalogPinUse(p->Port, p->Pin, false);
		}
	}

	vAds[Chan] = R9A02_ADC_CLOSED;
	vAcqNs[Chan] = 0;
	vbNew[Chan] = false;
	vhFifo[Chan] = NULL;

	int scan = vScan;

	UpdateScan();
	if (scan != vScan && vIntNo >= 0)
	{
		Configure();
	}
}

bool AdcR9a02::StartConversion()
{
	int first = NextChan(0);

	if (first < 0 || vIntNo < 0 || vCurChan >= 0 || vbArmed || vbRunning)
	{
		return false;
	}

	uintptr_t state;
	bool ok = true;

	if (HwTrig())
	{
		state = DisableInterrupt();
		ok = Arm();
		EnableInterrupt(state);

		return ok;
	}

	if (vMode == ADC_CONV_MODE_CONTINUOUS)
	{
		vRound = 0;
		vbRunning = true;

		// Round rate in mHz, period in ns
		if (vRate > 0U && vpTimer != NULL)
		{
			vTimerTrig = vpTimer->EnableTimerTrigger(1000000000000ULL / vRate, TIMER_TRIG_TYPE_CONTINUOUS,
													 R9a02AdcTimerHandler, this);
			if (vTimerTrig < 0)
			{
				vbRunning = false;

				return false;
			}
		}

		state = DisableInterrupt();
		ok = StartRound();
		EnableInterrupt(state);

		return ok;
	}

	if (vbInterrupt)
	{
		state = DisableInterrupt();
		ok = StartRound();
		EnableInterrupt(state);

		return ok;
	}

	if (vScan >= 0)
	{
		return ConvertScan();
	}

	for (int i = first; i >= 0; i = NextChan(i + 1))
	{
		if (!ConvertOne(i))
		{
			return false;
		}
	}

	return true;
}

void AdcR9a02::StopConversion()
{
	if (vTimerTrig >= 0 && vpTimer != NULL)
	{
		vpTimer->DisableTimerTrigger(vTimerTrig);
	}

	uintptr_t state = DisableInterrupt();

	vTimerTrig = -1;
	vbRunning = false;

	if (vbArmed)
	{
		Disarm();
	}
	else
	{
		vCurChan = -1;
		if (vIntNo >= 0 && !vbSnooze)
		{
			R9A02_ADC12->ADM0 = (uint8_t)(Adm0() | R9A02_ADM0_ADCE);
			R9a02ClearPendingInt(vIntNo);
		}
	}

	EnableInterrupt(state);
}

bool AdcR9a02::UpdateData()
{
	// Hardware trigger without interrupt: the last result
	if (HwTrig() && vbArmed && !vbInterrupt && vIntNo >= 0 && R9a02EventFlag(vIntNo))
	{
		R9a02EventFlagClear(vIntNo);
		HwEnd();
	}

	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		if (vAds[i] != R9A02_ADC_CLOSED && (vbNew[i] || (vhFifo[i] != NULL && CFifoUsed(vhFifo[i]) > 0)))
		{
			return true;
		}
	}

	return false;
}

int AdcR9a02::Read(AdcData_t *pBuff, int Len)
{
	int cnt = 0;

	if (pBuff == NULL)
	{
		return 0;
	}

	UpdateData();

	for (int i = 0; i < R9A02_ADC_MAX_CHAN && cnt < Len; i++)
	{
		if (vAds[i] != R9A02_ADC_CLOSED && Read(i, &pBuff[cnt]))
		{
			cnt++;
		}
	}

	return cnt;
}

bool AdcR9a02::Read(int Chan, AdcData_t *pBuff)
{
	if (pBuff == NULL || Chan < 0 || Chan >= R9A02_ADC_MAX_CHAN || vAds[Chan] == R9A02_ADC_CLOSED)
	{
		return false;
	}

	UpdateData();

	bool waiting = vbNew[Chan] || (vhFifo[Chan] != NULL && CFifoUsed(vhFifo[Chan]) > 0);

	// Software trigger without interrupt: converted now
	if (!waiting)
	{
		if (vbInterrupt || HwTrig() || vCurChan >= 0 || vbRunning ||
			!(vScan >= 0 ? ConvertScan() : ConvertOne(Chan)))
		{
			return false;
		}
	}

	uintptr_t state = DisableInterrupt();
	bool ok = false;

	if (vhFifo[Chan] != NULL)
	{
		AdcData_t *p = (AdcData_t *)CFifoGet(vhFifo[Chan]);

		if (p != NULL)
		{
			*pBuff = *p;
			ok = true;
		}
		vbNew[Chan] = false;
		EnableInterrupt(state);

		return ok;
	}

	uint16_t raw = vRaw[Chan];
	uint32_t stamp = vStamp[Chan];

	ok = vbNew[Chan];
	vbNew[Chan] = false;
	EnableInterrupt(state);

	if (ok)
	{
		pBuff->Timestamp = stamp;
		pBuff->Chan = Chan;
		pBuff->Data = ToVolt(Chan, raw);
	}

	return ok;
}

bool AdcR9a02::Trigger(R9A02_ADC_TRIG Trig, uint8_t ElcEvt)
{
	if (vIntNo < 0 || vbSnooze || vCurChan >= 0 || vbArmed || vbRunning ||
		(Trig != R9A02_ADC_TRIG_SOFT && Trig != R9A02_ADC_TRIG_TAU0_CH1 && Trig != R9A02_ADC_TRIG_RTC &&
		 Trig != R9A02_ADC_TRIG_TML32 && Trig != R9A02_ADC_TRIG_ELC) ||
		(Trig == R9A02_ADC_TRIG_ELC && (ElcEvt == 0U || ElcEvt > R9A02_EVT_MAX)))
	{
		return false;
	}

	// The internal inputs need the comparator off at each change (29.3.6)
	for (int i = 0; Trig != R9A02_ADC_TRIG_SOFT && i < R9A02_ADC_MAX_CHAN; i++)
	{
		if (vAds[i] != R9A02_ADC_CLOSED && (vAds[i] & R9A02_ADS_ADISS))
		{
			return false;
		}
	}

	vTrig = Trig;
	vElcEvt = Trig == R9A02_ADC_TRIG_ELC ? ElcEvt : 0U;

	return Configure();
}

// ADUL and ADLL compare with ADCRn [11:4] in 12 bits, [15:8] in 10 bits,
// the 8-bit result in 8 bits (29.3.7, 29.3.8). The manual does not give the
// bits for 8 bits.
bool AdcR9a02::Window(float LowVolt, float HighVolt, bool bOutside)
{
	if (vIntNo < 0 || vRefVolt <= 0.0f || vbArmed || vbRunning || vCurChan >= 0)
	{
		return false;
	}

	float lo = LowVolt * 256.0f / vRefVolt;
	float hi = HighVolt * 256.0f / vRefVolt;

	lo = lo < 0.0f ? 0.0f : lo > 255.0f ? 255.0f : lo;
	hi = hi < 0.0f ? 0.0f : hi > 255.0f ? 255.0f : hi;

	// ADUL above ADLL (29.3.8)
	if ((uint8_t)hi <= (uint8_t)lo)
	{
		return false;
	}

	vAdll = (uint8_t)lo;
	vAdul = (uint8_t)hi;
	vbOutside = bOutside;
	vbWindow = true;

	return Configure();
}

void AdcR9a02::WindowOff()
{
	vbWindow = false;
	vbOutside = false;
	if (vIntNo >= 0 && !vbArmed && !vbRunning && vCurChan < 0)
	{
		Configure();
	}
}

bool AdcR9a02::Snooze(bool bEnable)
{
	if (vIntNo < 0 || vCurChan >= 0 || vbArmed || vbRunning)
	{
		return false;
	}

	if (bEnable)
	{
		int cnt = 0;
		uint32_t pclk = SystemPeriphClockGet(0);

		// ICLK:PCLKB up to 8:1 for the A/D converter in Snooze (10.9.10)
		if (pclk == 0U || SystemCoreClock > 8U * pclk)
		{
			return false;
		}

		// One channel or a scan group, external inputs (29.3.6, 29.8)
		for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
		{
			if (vAds[i] != R9A02_ADC_CLOSED)
			{
				if (vAds[i] & R9A02_ADS_ADISS)
				{
					return false;
				}
				cnt++;
			}
		}
		if (cnt == 0 || (vScan < 0 && cnt != 1))
		{
			return false;
		}
	}

	if (!R9a02StandbyHookSet(R9a02AdcStandbyHook, this, bEnable) && bEnable)
	{
		return false;
	}

	vbSnooze = bEnable;

	return Configure();
}

// Zero scale ('-' reference) and full scale ('+' reference) conversions
// (29.3.9, table 29.19), each with the test target set while stopped,
// software trigger, select mode, no window
bool AdcR9a02::Calibrate()
{
	if (vIntNo < 0 || vCurChan >= 0 || vbArmed || vbRunning)
	{
		return false;
	}

	R9A02_ADC12_Type *reg = R9A02_ADC12;
	uint16_t res[2] = { 0, 0 };
	bool ok = true;
	uintptr_t state = DisableInterrupt();

	for (int i = 0; i < 2 && ok; i++)
	{
		uint32_t n = vLoops;

		R9a02AdcStop();
		reg->ADM0 = vAdm0;
		reg->ADM1 = vAdm1;
		reg->ADM2 = vAdm2;
		reg->ADUL = 0xFFU;
		reg->ADLL = 0U;
		reg->ADS = 0U;
		reg->ADTES = i ? R9A02_ADTES_FULL_SCALE : R9A02_ADTES_ZERO_SCALE;
		R9a02DelayUs(R9A02_ADC_SETUP_US);
		reg->ADM0 = (uint8_t)(vAdm0 | R9A02_ADM0_ADCE);
		R9a02AdcStartWait(vAdm0);
		R9a02ClearPendingInt(vIntNo);
		reg->ADM0 = (uint8_t)(vAdm0 | R9A02_ADM0_ADCE | R9A02_ADM0_ADCS);

		while (!R9a02EventFlag(vIntNo))
		{
			if (--n == 0U)
			{
				ok = false;
				break;
			}
		}
		res[i] = R9a02AdcRaw(reg->ADCRN[0], vAdm2);
	}

	R9a02ClearPendingInt(vIntNo);
	EnableInterrupt(state);

	Configure();

	if (ok && res[1] > res[0])
	{
		vZero = res[0];
		vFull = res[1];
		vbCal = true;
	}

	return ok && vbCal;
}

bool AdcR9a02::Enable()
{
	if (vIntNo < 0)
	{
		return false;
	}

	R9a02ModuleStart(R9A02_MSTPCR_D, R9A02_MSTPCRD_ADC12);

	return Configure();
}

// Comparator off, internal inputs off. Before Software Standby ADCS, ADCE
// and ADS.ADISS must be 0 (29.3.6, 29.10), except in Snooze.
void AdcR9a02::Disable()
{
	StopConversion();
	R9a02AdcStop();
	R9A02_ADC12->ADS = 0U;
	vbInternal = false;
	vbDiscard = false;
}

void AdcR9a02::Reset()
{
	StopConversion();
	Configure();
}

// Converter off, pins disconnected, event route removed, module stopped. A
// new Init is needed to use it again.
void AdcR9a02::PowerOff()
{
	Disable();

	for (int i = 0; i < R9A02_ADC_MAX_CHAN; i++)
	{
		CloseChannel(i);
	}

	if ((vAdm2 & R9A02_ADM2_ADREFP_DISCHARGE) == R9A02_ADM2_ADREFP_AVREFP)
	{
		R9a02AnalogPinUse(0, 2, false);
	}

	uintptr_t state = DisableInterrupt();

	R9a02StandbyHookSet(R9a02AdcStandbyHook, this, false);
	R9a02UnregisterIntHandler(vIntNo);
	vIntNo = -1;
	vbSnooze = false;
	vTrig = R9A02_ADC_TRIG_SOFT;
	R9a02ModuleStop(R9A02_MSTPCR_D, R9A02_MSTPCRD_ADC12);

	EnableInterrupt(state);
}
