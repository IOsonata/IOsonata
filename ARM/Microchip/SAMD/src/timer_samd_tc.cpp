/**-------------------------------------------------------------------------
@file	timer_samd_tc.cpp

@brief	SAM D11/D21 TC timers, DevNo 1 and up

		TC in 16-bit normal frequency mode, CC0 and CC1 as compares,
		clocked by the main clock on generator 0. The count is read with a
		read request, synchronized in a few main clock cycles.

		External trigger: CC1 captures the count on the event input, the
		event input enabled from the setup (EVCTRL is enable protected), the
		event action off. CC1 is then not a compare.

		SAM D11: DevNo 1 TC1, DevNo 2 TC2
		SAM D21: DevNo 1 TC3 to DevNo 5 TC7

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
#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"

#include "clock_samd.h"
#include "timer_samd.h"

#define SAMD_TC_SYNC_TIMEOUT		100000U

static const uint8_t s_SamdTcMcFlag[SAMD_TC_TIMER_TRIG_MAXCNT] = {
	TC_INTFLAG_MC0_Msk, TC_INTFLAG_MC1_Msk
};

static bool SamdTcSync(tc_count16_registers_t *pReg)
{
	for (uint32_t i = 0; i < SAMD_TC_SYNC_TIMEOUT; i++)
	{
		if ((pReg->TC_STATUS & TC_STATUS_SYNCBUSY_Msk) == 0U)
		{
			return true;
		}
	}

	return false;
}

static uint16_t SamdTcCtrla(SamdTimerData_t *pData, bool bRun)
{
	return (uint16_t)(TC_CTRLA_MODE(TC_CTRLA_MODE_COUNT16_Val) | TC_CTRLA_WAVEGEN(TC_CTRLA_WAVEGEN_NFRQ_Val) |
					  TC_CTRLA_PRESCALER(pData->Prescaler) | (bRun ? TC_CTRLA_ENABLE_Msk : 0U));
}

bool SamdTcSetup(SamdTimerData_t *pData)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	PM_REGS->PM_APBCMASK |= pData->ApbMask;

	// Shared channels of SAM D21 pairs: both on the main clock
	if (!SamdGclkChannelEnable(pData->GclkId, SAMD_GCLK_MAIN))
	{
		return false;
	}

	reg->TC_INTENCLR = TC_INTENCLR_Msk;

	if (!SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_CTRLA = TC_CTRLA_SWRST_Msk;
	for (uint32_t i = 0; (reg->TC_CTRLA & TC_CTRLA_SWRST_Msk) != 0U; i++)
	{
		if (i >= SAMD_TC_SYNC_TIMEOUT)
		{
			return false;
		}
	}
	if (!SamdTcSync(reg))
	{
		return false;
	}

	reg->TC_CTRLA = SamdTcCtrla(pData, false);
	if (!SamdTcSync(reg))
	{
		return false;
	}

	// Event input for the capture, no event action
	reg->TC_EVCTRL = (uint16_t)(TC_EVCTRL_EVACT_OFF | TC_EVCTRL_TCEI_Msk);
	reg->TC_INTFLAG = TC_INTFLAG_Msk;

	return SamdTcCapture(pData, pData->bCapture);
}

bool SamdTcRun(SamdTimerData_t *pData, bool bRun)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	if (bRun)
	{
		reg->TC_INTENSET = TC_INTENSET_OVF_Msk;
	}
	else
	{
		reg->TC_INTENCLR = TC_INTENCLR_OVF_Msk;
	}

	if (!SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_CTRLA = SamdTcCtrla(pData, bRun);

	return SamdTcSync(reg);
}

bool SamdTcReset(SamdTimerData_t *pData)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	if (!SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_COUNT = 0;
	if (!SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_INTFLAG = TC_INTFLAG_Msk;

	return true;
}

// A read request each time, the count is current within a few cycles
bool SamdTcRead(SamdTimerData_t *pData, uint32_t *pCount)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	if (!SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_READREQ = (uint16_t)(TC_READREQ_RREQ_Msk | TC_READREQ_ADDR(TC_COUNT16_COUNT_REG_OFST));
	if (!SamdTcSync(reg))
	{
		return false;
	}
	*pCount = reg->TC_COUNT;

	return true;
}

bool SamdTcOverflow(SamdTimerData_t *pData)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	if ((reg->TC_INTFLAG & TC_INTFLAG_OVF_Msk) == 0U)
	{
		return false;
	}
	reg->TC_INTFLAG = TC_INTFLAG_OVF_Msk;

	return true;
}

// The flag is cleared after the compare is set, the core reads the count
// after it to find a deadline reached in between
bool SamdTcArm(SamdTimerData_t *pData, int TrigNo, bool bEnable)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	if (TrigNo < 0 || TrigNo >= SAMD_TC_TIMER_TRIG_MAXCNT)
	{
		return false;
	}

	uint8_t flag = s_SamdTcMcFlag[TrigNo];

	reg->TC_INTENCLR = flag;

	if (bEnable)
	{
		if (!SamdTcSync(reg))
		{
			return false;
		}
		reg->TC_CC[TrigNo] = (uint16_t)pData->pTrig[TrigNo].Deadline;
		if (!SamdTcSync(reg))
		{
			return false;
		}
	}

	reg->TC_INTFLAG = flag;

	if (bEnable)
	{
		reg->TC_INTENSET = flag;
	}

	return true;
}

bool SamdTcMatched(SamdTimerData_t *pData, int TrigNo)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	if (TrigNo < 0 || TrigNo >= SAMD_TC_TIMER_TRIG_MAXCNT)
	{
		return false;
	}

	uint8_t flag = s_SamdTcMcFlag[TrigNo];

	if ((reg->TC_INTFLAG & flag) == 0U)
	{
		return false;
	}
	reg->TC_INTFLAG = flag;

	return true;
}

// CC1 captures or compares. Interrupts disabled.
bool SamdTcCapture(SamdTimerData_t *pData, bool bEnable)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	reg->TC_INTENCLR = TC_INTENCLR_MC1_Msk;
	if (!SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_CTRLC = bEnable ? TC_CTRLC_CPTEN1_Msk : 0U;
	if (!SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_INTFLAG = TC_INTFLAG_MC1_Msk | TC_INTFLAG_ERR_Msk;

	if (bEnable)
	{
		reg->TC_INTENSET = TC_INTENSET_MC1_Msk;
	}

	return true;
}

// The count captured in CC1, read with a read request, which clears MC1.
// A capture lost to a capture not yet read sets ERR, cleared here.
bool SamdTcCaptured(SamdTimerData_t *pData, uint32_t *pCount)
{
	tc_count16_registers_t *reg = (tc_count16_registers_t *)pData->pReg;

	reg->TC_INTFLAG = TC_INTFLAG_ERR_Msk;
	if ((reg->TC_INTFLAG & TC_INTFLAG_MC1_Msk) == 0U || !SamdTcSync(reg))
	{
		return false;
	}
	reg->TC_READREQ = (uint16_t)(TC_READREQ_RREQ_Msk | TC_READREQ_ADDR(TC_COUNT16_CC_REG_OFST + 2U));
	if (!SamdTcSync(reg))
	{
		return false;
	}
	*pCount = reg->TC_CC[1];

	return true;
}

#if !defined(TC3_REGS)
extern "C" void TC1_Handler(void)
{
	SamdTimerIrq(1);
}

extern "C" void TC2_Handler(void)
{
	SamdTimerIrq(2);
}
#else
extern "C" void TC3_Handler(void)
{
	SamdTimerIrq(1);
}

extern "C" void TC4_Handler(void)
{
	SamdTimerIrq(2);
}

extern "C" void TC5_Handler(void)
{
	SamdTimerIrq(3);
}

#if SAMD_TC_TIMER_MAXCNT > 3
extern "C" void TC6_Handler(void)
{
	SamdTimerIrq(4);
}

extern "C" void TC7_Handler(void)
{
	SamdTimerIrq(5);
}
#endif
#endif
