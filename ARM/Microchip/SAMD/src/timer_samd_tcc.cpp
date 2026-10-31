/**-------------------------------------------------------------------------
@file	timer_samd_tcc.cpp

@brief	SAM D11/D21 TCC timers, after the TC

		TCC in normal frequency mode, top at the counter maximum, the
		compare channels as compares, clocked by the main clock on
		generator 0. The count is read with the READSYNC command,
		synchronized in a few main clock cycles. No waveform output.

		External trigger: the last compare channel captures the count on
		its event input, enabled from the setup (EVCTRL is enable
		protected). The capture enable of CTRLA is enable protected too: a
		change stops the counter for the synchronizations, about a
		microsecond.

		SAM D11: DevNo 3 TCC0, 24-bit, 4 compares
		SAM D21: DevNo 6 TCC0, 24-bit, 4 compares; DevNo 7 TCC1, 24-bit,
		2 compares; DevNo 8 TCC2, 16-bit, 2 compares

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"

#include "clock_samd.h"
#include "timer_samd.h"

#define SAMD_TCC_SYNC_TIMEOUT		100000U

static inline tcc_registers_t *SamdTccReg(SamdTimerData_t *pData)
{
	return (tcc_registers_t *)pData->pReg;
}

static inline uint32_t SamdTccMax(const SamdTimerData_t *pData)
{
	return (uint32_t)((1ULL << pData->CntBits) - 1U);
}

static bool SamdTccSync(tcc_registers_t *pReg, uint32_t Mask)
{
	for (uint32_t i = 0; i < SAMD_TCC_SYNC_TIMEOUT; i++)
	{
		if ((pReg->TCC_SYNCBUSY & Mask) == 0U)
		{
			return true;
		}
	}

	return false;
}

static inline uint32_t SamdTccCptMask(const SamdTimerData_t *pData)
{
	return TCC_CTRLA_CPTEN0_Msk << (pData->NbTrig - 1);
}

static inline uint32_t SamdTccCtrla(SamdTimerData_t *pData, bool bCapture, bool bRun)
{
	return TCC_CTRLA_PRESCALER(pData->Prescaler) | TCC_CTRLA_PRESCSYNC_GCLK |
		   (bCapture ? SamdTccCptMask(pData) : 0U) | (bRun ? TCC_CTRLA_ENABLE_Msk : 0U);
}

bool SamdTccSetup(SamdTimerData_t *pData)
{
	tcc_registers_t *reg = SamdTccReg(pData);

	PM_REGS->PM_APBCMASK |= pData->ApbMask;

	// Shared channels: TCC0 and TCC1, TCC2 and TC3 of SAM D21, all on the
	// main clock
	if (!SamdGclkChannelEnable(pData->GclkId, SAMD_GCLK_MAIN))
	{
		return false;
	}

	reg->TCC_INTENCLR = TCC_INTENCLR_Msk;

	reg->TCC_CTRLA = TCC_CTRLA_SWRST_Msk;
	if (!SamdTccSync(reg, TCC_SYNCBUSY_SWRST_Msk))
	{
		return false;
	}

	// Enable protected fields, written with the counter stopped. Event
	// input of the capture channel.
	reg->TCC_CTRLA = SamdTccCtrla(pData, pData->bCapture, false);
	reg->TCC_EVCTRL = TCC_EVCTRL_MCEI0_Msk << (pData->NbTrig - 1);
	reg->TCC_WAVE = TCC_WAVE_WAVEGEN_NFRQ;
	if (!SamdTccSync(reg, TCC_SYNCBUSY_WAVE_Msk))
	{
		return false;
	}
	reg->TCC_PER = SamdTccMax(pData);
	if (!SamdTccSync(reg, TCC_SYNCBUSY_PER_Msk))
	{
		return false;
	}
	reg->TCC_INTFLAG = TCC_INTFLAG_Msk;
	pData->StopCount = 0;

	if (pData->bCapture)
	{
		reg->TCC_INTENSET = TCC_INTFLAG_MC0_Msk << (pData->NbTrig - 1);
	}

	return true;
}

bool SamdTccRun(SamdTimerData_t *pData, bool bRun)
{
	tcc_registers_t *reg = SamdTccReg(pData);
	uint32_t count;

	if (bRun)
	{
		reg->TCC_INTENSET = TCC_INTENSET_OVF_Msk;
	}
	else
	{
		reg->TCC_INTENCLR = TCC_INTENCLR_OVF_Msk;

		// The count where it stops, read while the counter runs
		if ((reg->TCC_CTRLA & TCC_CTRLA_ENABLE_Msk) != 0U && SamdTccRead(pData, &count))
		{
			pData->StopCount = count;
		}
	}

	if (!SamdTccSync(reg, TCC_SYNCBUSY_ENABLE_Msk))
	{
		return false;
	}
	reg->TCC_CTRLA = SamdTccCtrla(pData, (reg->TCC_CTRLA & SamdTccCptMask(pData)) != 0U, bRun);

	return SamdTccSync(reg, TCC_SYNCBUSY_ENABLE_Msk);
}

bool SamdTccReset(SamdTimerData_t *pData)
{
	tcc_registers_t *reg = SamdTccReg(pData);

	if (!SamdTccSync(reg, TCC_SYNCBUSY_COUNT_Msk))
	{
		return false;
	}
	reg->TCC_COUNT = 0;
	if (!SamdTccSync(reg, TCC_SYNCBUSY_COUNT_Msk))
	{
		return false;
	}
	reg->TCC_INTFLAG = TCC_INTFLAG_Msk;
	pData->StopCount = 0;

	return true;
}

// A READSYNC command each time, the count is current within a few cycles.
// A stopped counter keeps the count read when it stopped.
bool SamdTccRead(SamdTimerData_t *pData, uint32_t *pCount)
{
	tcc_registers_t *reg = SamdTccReg(pData);

	if ((reg->TCC_CTRLA & TCC_CTRLA_ENABLE_Msk) == 0U)
	{
		*pCount = pData->StopCount;

		return true;
	}

	if (!SamdTccSync(reg, TCC_SYNCBUSY_CTRLB_Msk))
	{
		return false;
	}
	reg->TCC_CTRLBSET = TCC_CTRLBSET_CMD_READSYNC;
	for (uint32_t i = 0; (reg->TCC_SYNCBUSY & TCC_SYNCBUSY_CTRLB_Msk) != 0U ||
						 (reg->TCC_CTRLBSET & TCC_CTRLBSET_CMD_Msk) != 0U; i++)
	{
		if (i >= SAMD_TCC_SYNC_TIMEOUT)
		{
			return false;
		}
	}
	*pCount = reg->TCC_COUNT & SamdTccMax(pData);

	return true;
}

bool SamdTccOverflow(SamdTimerData_t *pData)
{
	tcc_registers_t *reg = SamdTccReg(pData);

	if ((reg->TCC_INTFLAG & TCC_INTFLAG_OVF_Msk) == 0U)
	{
		return false;
	}
	reg->TCC_INTFLAG = TCC_INTFLAG_OVF_Msk;

	return true;
}

// The flag is cleared after the compare is set, the core reads the count
// after it to find a deadline reached in between
bool SamdTccArm(SamdTimerData_t *pData, int TrigNo, bool bEnable)
{
	tcc_registers_t *reg = SamdTccReg(pData);

	if (TrigNo < 0 || TrigNo >= pData->NbTrig)
	{
		return false;
	}

	uint32_t flag = TCC_INTFLAG_MC0_Msk << TrigNo;
	uint32_t sync = TCC_SYNCBUSY_CC0_Msk << TrigNo;

	reg->TCC_INTENCLR = flag;

	if (bEnable)
	{
		if (!SamdTccSync(reg, sync))
		{
			return false;
		}
		reg->TCC_CC[TrigNo] = (uint32_t)pData->pTrig[TrigNo].Deadline & SamdTccMax(pData);
		if (!SamdTccSync(reg, sync))
		{
			return false;
		}
	}

	reg->TCC_INTFLAG = flag;

	if (bEnable)
	{
		reg->TCC_INTENSET = flag;
	}

	return true;
}

bool SamdTccMatched(SamdTimerData_t *pData, int TrigNo)
{
	tcc_registers_t *reg = SamdTccReg(pData);

	if (TrigNo < 0 || TrigNo >= pData->NbTrig)
	{
		return false;
	}

	uint32_t flag = TCC_INTFLAG_MC0_Msk << TrigNo;

	if ((reg->TCC_INTFLAG & flag) == 0U)
	{
		return false;
	}
	reg->TCC_INTFLAG = flag;

	return true;
}

// The last compare channel captures or compares. Interrupts disabled.
bool SamdTccCapture(SamdTimerData_t *pData, bool bEnable)
{
	tcc_registers_t *reg = SamdTccReg(pData);
	uint32_t flag = TCC_INTFLAG_MC0_Msk << (pData->NbTrig - 1);

	reg->TCC_INTENCLR = flag;

	if (((reg->TCC_CTRLA & SamdTccCptMask(pData)) != 0U) != bEnable)
	{
		bool run = (reg->TCC_CTRLA & TCC_CTRLA_ENABLE_Msk) != 0U;

		if (run && !SamdTccRun(pData, false))
		{
			return false;
		}
		reg->TCC_CTRLA = SamdTccCtrla(pData, bEnable, false);
		if (run && !SamdTccRun(pData, true))
		{
			return false;
		}
	}

	reg->TCC_INTFLAG = flag | TCC_INTFLAG_ERR_Msk;

	if (bEnable)
	{
		reg->TCC_INTENSET = flag;
	}

	return true;
}

// The count captured, read from CC, which clears MC. A capture lost to a
// capture not yet read sets ERR, cleared here.
bool SamdTccCaptured(SamdTimerData_t *pData, uint32_t *pCount)
{
	tcc_registers_t *reg = SamdTccReg(pData);
	int ch = pData->NbTrig - 1;

	reg->TCC_INTFLAG = TCC_INTFLAG_ERR_Msk;
	if ((reg->TCC_INTFLAG & (TCC_INTFLAG_MC0_Msk << ch)) == 0U)
	{
		return false;
	}
	*pCount = reg->TCC_CC[ch] & SamdTccMax(pData);

	return true;
}

extern "C" void TCC0_Handler(void)
{
	SamdTimerIrq(SAMD_RTC_TIMER_MAXCNT + SAMD_TC_TIMER_MAXCNT);
}

#if SAMD_TCC_TIMER_MAXCNT > 1
extern "C" void TCC1_Handler(void)
{
	SamdTimerIrq(SAMD_RTC_TIMER_MAXCNT + SAMD_TC_TIMER_MAXCNT + 1);
}

extern "C" void TCC2_Handler(void)
{
	SamdTimerIrq(SAMD_RTC_TIMER_MAXCNT + SAMD_TC_TIMER_MAXCNT + 2);
}
#endif
