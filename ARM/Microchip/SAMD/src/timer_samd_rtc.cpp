/**-------------------------------------------------------------------------
@file	timer_samd_rtc.cpp

@brief	SAM D11/D21 RTC timer, DevNo 0

		RTC mode 0, 32-bit counter, compare 0, clocked by generator 1 at
		32.768 kHz. The count is continuously synchronized. Any RTC register
		write stops the continuous synchronization: SamdRtcResume starts it
		again after the writes, from the SYNCRDY interrupt when a
		synchronization is in progress, so the next read does not wait.
		SYNCRDY is not set after an enable or a software reset: SamdRtcWait
		starts it then, and when it is stopped otherwise.

		The registers written with a synchronization (CTRL, COUNT, COMP)
		take up to 6 periods of 32.768 kHz, about 200 us, and the bus stalls
		on an access during that time. Run, Reset, Read and Arm do not wait:
		the core calls them after SamdRtcWait, with interrupts disabled, one
		such write at a time, then SamdRtcResume. SamdRtcWait and
		SamdRtcSetup wait with the interrupts as the caller has them.

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

#include "coredev/interrupt.h"
#include "clock_samd.h"
#include "evsys_samd.h"
#include "timer_samd.h"

// Bounded wait, several 32 kHz synchronizations at 48 MHz
#define SAMD_RTC_SYNC_TIMEOUT		100000U

// Prescaler of the periodic event 0
#define SAMD_RTC_PER_PRESC			3U

// Continuous read of the count
#define SAMD_RTC_READREQ			(RTC_READREQ_RREQ_Msk | RTC_READREQ_RCONT_Msk | \
									 RTC_READREQ_ADDR(RTC_MODE0_COUNT_REG_OFST))

static inline rtc_mode0_registers_t *SamdRtcReg(void)
{
	return &RTC_REGS->MODE0;
}

static bool SamdRtcSync(void)
{
	for (uint32_t i = 0; i < SAMD_RTC_SYNC_TIMEOUT; i++)
	{
		if ((SamdRtcReg()->RTC_STATUS & RTC_STATUS_SYNCBUSY_Msk) == 0U)
		{
			return true;
		}
	}

	return false;
}

static uint16_t SamdRtcCtrl(SamdTimerData_t *pData, bool bRun)
{
	return (uint16_t)(RTC_MODE0_CTRL_MODE(0U) | RTC_MODE0_CTRL_PRESCALER(pData->Prescaler) |
					  (bRun ? RTC_MODE0_CTRL_ENABLE_Msk : 0U));
}

// Counter stopped, the waits with the interrupts as the caller has them
bool SamdRtcSetup(SamdTimerData_t *pData)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	PM_REGS->PM_APBAMASK |= PM_APBAMASK_RTC_Msk;

	if (!SamdGclkChannelEnable(GCLK_CLKCTRL_ID_RTC_Val, SAMD_GCLK_LF))
	{
		return false;
	}

	reg->RTC_INTENCLR = RTC_MODE0_INTENCLR_Msk;

	if (!SamdRtcSync())
	{
		return false;
	}
	reg->RTC_CTRL = RTC_MODE0_CTRL_SWRST_Msk;
	for (uint32_t i = 0; (reg->RTC_CTRL & RTC_MODE0_CTRL_SWRST_Msk) != 0U; i++)
	{
		if (i >= SAMD_RTC_SYNC_TIMEOUT)
		{
			return false;
		}
	}
	if (!SamdRtcSync())
	{
		return false;
	}

	// Enable protected fields, written with the counter stopped. The tick is
	// the periodic event at the count frequency.
	reg->RTC_CTRL = SamdRtcCtrl(pData, false);
	reg->RTC_EVCTRL = SamdRtcTickGen(pData) != 0U ?
					  (uint16_t)(RTC_MODE0_EVCTRL_PEREO0_Msk << (pData->Prescaler - SAMD_RTC_PER_PRESC)) : 0U;
	reg->RTC_INTFLAG = RTC_MODE0_INTFLAG_Msk;

	return SamdRtcSync();
}

// Periodic event n runs at 32768 Hz / 2^(n + 3), the count at 32768 Hz /
// 2^Prescaler: the tick is the periodic event n = Prescaler - 3
uint32_t SamdRtcTickGen(SamdTimerData_t *pData)
{
	if (!pData->bTick || pData->Prescaler < SAMD_RTC_PER_PRESC)
	{
		return 0;
	}

	return SAMD_EVSYS_GEN_RTC_PER(pData->Prescaler - SAMD_RTC_PER_PRESC);
}

// No synchronization in progress and the count continuously synchronized.
// Fresh: a new read request after the synchronization in progress, the next
// read is from after it. The checks and the read request run with
// interrupts disabled, the waits between them with the interrupts as the
// caller has them.
bool SamdRtcWait(SamdTimerData_t *pData, bool bFresh)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();
	bool req = bFresh;

	(void)pData;

	for (uint32_t i = 0; i < SAMD_RTC_SYNC_TIMEOUT; i++)
	{
		uint32_t state = DisableInterrupt();
		bool idle = (reg->RTC_STATUS & RTC_STATUS_SYNCBUSY_Msk) == 0U;
		bool cont = (reg->RTC_READREQ & RTC_READREQ_RCONT_Msk) != 0U;

		if (idle && (req || !cont))
		{
			// The SYNCRDY interrupt of SamdRtcResume is not needed
			reg->RTC_INTENCLR = RTC_MODE0_INTENCLR_SYNCRDY_Msk;
			reg->RTC_READREQ = SAMD_RTC_READREQ;
			req = false;
			cont = false;
		}
		EnableInterrupt(state);

		if (idle && cont)
		{
			return true;
		}
	}

	return false;
}

// Interrupts disabled, after register writes: the continuous read started
// again now if no synchronization is in progress, else from the SYNCRDY
// interrupt at the end of it
void SamdRtcResume(SamdTimerData_t *pData)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	(void)pData;

	if ((reg->RTC_READREQ & RTC_READREQ_RCONT_Msk) != 0U)
	{
		return;
	}

	if ((reg->RTC_STATUS & RTC_STATUS_SYNCBUSY_Msk) != 0U)
	{
		reg->RTC_INTFLAG = RTC_MODE0_INTFLAG_SYNCRDY_Msk;
		reg->RTC_INTENSET = RTC_MODE0_INTENSET_SYNCRDY_Msk;

		// The synchronization can end before the flag is cleared
		if ((reg->RTC_STATUS & RTC_STATUS_SYNCBUSY_Msk) != 0U)
		{
			return;
		}
		reg->RTC_INTENCLR = RTC_MODE0_INTENCLR_SYNCRDY_Msk;
	}

	reg->RTC_READREQ = SAMD_RTC_READREQ;
}

// Interrupt, interrupts disabled: the SYNCRDY interrupt of SamdRtcResume.
// True when a compare or overflow interrupt is pending.
bool SamdRtcEvent(SamdTimerData_t *pData)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();
	uint8_t flag = reg->RTC_INTFLAG & reg->RTC_INTENSET;

	if ((flag & RTC_MODE0_INTFLAG_SYNCRDY_Msk) != 0U)
	{
		reg->RTC_INTENCLR = RTC_MODE0_INTENCLR_SYNCRDY_Msk;
		reg->RTC_INTFLAG = RTC_MODE0_INTFLAG_SYNCRDY_Msk;
		SamdRtcResume(pData);
	}

	return (flag & (RTC_MODE0_INTFLAG_CMP0_Msk | RTC_MODE0_INTFLAG_OVF_Msk)) != 0U;
}

// Interrupts disabled, no synchronization in progress
bool SamdRtcRun(SamdTimerData_t *pData, bool bRun)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	if (bRun)
	{
		reg->RTC_INTENSET = RTC_MODE0_INTENSET_OVF_Msk;
	}
	else
	{
		reg->RTC_INTENCLR = RTC_MODE0_INTENCLR_OVF_Msk;
	}
	reg->RTC_CTRL = SamdRtcCtrl(pData, bRun);

	return true;
}

// Interrupts disabled, no synchronization in progress. The counter keeps
// running from zero.
bool SamdRtcReset(SamdTimerData_t *pData)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	(void)pData;

	reg->RTC_COUNT = 0;
	reg->RTC_INTFLAG = RTC_MODE0_INTFLAG_Msk;

	return true;
}

// The last continuously synchronized count. False while a synchronization
// is in progress or the continuous synchronization is stopped: the value
// may be from before the last write.
bool SamdRtcRead(SamdTimerData_t *pData, uint32_t *pCount)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	(void)pData;

	if ((reg->RTC_STATUS & RTC_STATUS_SYNCBUSY_Msk) != 0U ||
		(reg->RTC_READREQ & RTC_READREQ_RCONT_Msk) == 0U)
	{
		return false;
	}
	*pCount = reg->RTC_COUNT;

	return true;
}

bool SamdRtcOverflow(SamdTimerData_t *pData)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	(void)pData;

	if ((reg->RTC_INTFLAG & RTC_MODE0_INTFLAG_OVF_Msk) == 0U)
	{
		return false;
	}
	reg->RTC_INTFLAG = RTC_MODE0_INTFLAG_OVF_Msk;

	return true;
}

// Interrupts disabled. With a compare: no synchronization in progress, the
// compare takes effect a synchronization after the write, the core checks a
// close deadline once it has. Without: no synchronized register written.
bool SamdRtcArm(SamdTimerData_t *pData, int TrigNo, bool bEnable)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	if (TrigNo != 0)
	{
		return false;
	}

	reg->RTC_INTENCLR = RTC_MODE0_INTENCLR_CMP0_Msk;
	if (bEnable)
	{
		reg->RTC_COMP = (uint32_t)pData->pTrig[0].Deadline;
	}
	reg->RTC_INTFLAG = RTC_MODE0_INTFLAG_CMP0_Msk;
	if (bEnable)
	{
		reg->RTC_INTENSET = RTC_MODE0_INTENSET_CMP0_Msk;
	}

	return true;
}

bool SamdRtcMatched(SamdTimerData_t *pData, int TrigNo)
{
	rtc_mode0_registers_t *reg = SamdRtcReg();

	(void)pData;

	if (TrigNo != 0 || (reg->RTC_INTFLAG & RTC_MODE0_INTFLAG_CMP0_Msk) == 0U)
	{
		return false;
	}
	reg->RTC_INTFLAG = RTC_MODE0_INTFLAG_CMP0_Msk;

	return true;
}

// A synchronization takes up to 6 generator periods and the count is
// synchronized after each change: up to 7 periods of 32768 Hz
uint32_t SamdRtcLag(SamdTimerData_t *pData)
{
	return (7U >> pData->Prescaler) + 1U;
}

extern "C" void RTC_Handler(void)
{
	SamdTimerIrq(0);
}
