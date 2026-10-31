/**-------------------------------------------------------------------------
@file	timer_samd.h

@brief	SAM D11/D21 timer implementation, private definitions

Timer devices, from low power to high frequency:

	DevNo 0     : RTC, 32-bit, 32.768 kHz generator 1, 1 trigger
	TC          : 16-bit, main clock, 2 triggers each
	              SAM D11 DevNo 1, 2: TC1, TC2
	              SAM D21 DevNo 1 to 5: TC3 to TC7
	TCC         : main clock, a trigger per compare channel
	              SAM D11 DevNo 3: TCC0, 24-bit, 4 triggers
	              SAM D21 DevNo 6 to 8: TCC0, 24-bit, 4 triggers; TCC1,
	              24-bit, 2 triggers; TCC2, 16-bit, 2 triggers

The counters are extended to 64 bits by the overflow interrupt. A trigger
has a 64-bit deadline, the compare register holds its low bits.

The register accesses of a device run with interrupts disabled, from the
core in timer_samd.cpp. The core waits for the 32.768 kHz synchronization
of the RTC, about 200 us, before them with SamdRtcWait, with the interrupts
as the caller has them.

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
#ifndef __TIMER_SAMD_H__
#define __TIMER_SAMD_H__

#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"
#include "coredev/timer.h"

#define SAMD_RTC_TIMER_MAXCNT		1
#define SAMD_RTC_TIMER_TRIG_MAXCNT	1

#if defined(TC7_REGS)
#define SAMD_TC_TIMER_MAXCNT		5		// TC3 to TC7
#elif defined(TC5_REGS)
#define SAMD_TC_TIMER_MAXCNT		3		// TC3 to TC5
#else
#define SAMD_TC_TIMER_MAXCNT		2		// TC1, TC2
#endif
#define SAMD_TC_TIMER_TRIG_MAXCNT	2

#if defined(TCC2_REGS)
#define SAMD_TCC_TIMER_MAXCNT		3		// TCC0 to TCC2
#define SAMD_TCC_TIMER_TRIG_CNT		(TCC0_CC_NUM + TCC1_CC_NUM + TCC2_CC_NUM)
#else
#define SAMD_TCC_TIMER_MAXCNT		1		// TCC0
#define SAMD_TCC_TIMER_TRIG_CNT		TCC0_CC_NUM
#endif

#define SAMD_TIMER_MAXCNT			(SAMD_RTC_TIMER_MAXCNT + SAMD_TC_TIMER_MAXCNT + SAMD_TCC_TIMER_MAXCNT)
#define SAMD_TIMER_TRIG_MAXCNT		4		// The most of one device, TCC0
#define SAMD_TIMER_TRIG_CNT			(SAMD_RTC_TIMER_TRIG_MAXCNT + SAMD_TC_TIMER_MAXCNT * SAMD_TC_TIMER_TRIG_MAXCNT + \
									 SAMD_TCC_TIMER_TRIG_CNT)

#pragma pack(push, 4)

typedef struct {
	TimerTrig_t Info;
	uint64_t Deadline;			//!< Count of the next event
	uint32_t Ticks;				//!< Period in counts
	uint32_t Seq;				//!< Changed each time the compare is set or cleared
	bool bActive;
	bool bDue;					//!< Deadline passed before the compare was set
} SamdTimerTrig_t;

typedef struct __Samd_Timer_Data	SamdTimerData_t;

// Register operations of a kind of counter:
//	Setup	: clock, reset, mode and Prescaler, counter stopped
//	Run		: counter started or stopped, overflow interrupt
//	Reset	: counter at zero, flags cleared
//	Read	: counter value, false if it cannot be read now
//	Overflow: overflow flag, cleared once read
//	Arm		: compare of a trigger at the low bits of its deadline, or off
//	Matched	: compare flag of a trigger, cleared once read
//	Capture	: capture of the external trigger event on the last compare
//			  channel, on or off, NULL when the device has none
//	Captured: count captured, false if none, the capture flags cleared
// Run, Reset, Read, Overflow, Arm and Matched are called with interrupts
// disabled. On the RTC they do not wait. Run, Reset, and Arm with a compare,
// write one register that needs a synchronization: each is called after
// SamdRtcWait, with no other such write in between. SamdRtcResume is called
// after the writes.
typedef struct {
	bool (*Setup)(SamdTimerData_t *pData);
	bool (*Run)(SamdTimerData_t *pData, bool bRun);
	bool (*Reset)(SamdTimerData_t *pData);
	bool (*Read)(SamdTimerData_t *pData, uint32_t *pCount);
	bool (*Overflow)(SamdTimerData_t *pData);
	bool (*Arm)(SamdTimerData_t *pData, int TrigNo, bool bEnable);
	bool (*Matched)(SamdTimerData_t *pData, int TrigNo);
	bool (*Capture)(SamdTimerData_t *pData, bool bEnable);
	bool (*Captured)(SamdTimerData_t *pData, uint32_t *pCount);
} SamdTimerOps_t;

struct __Samd_Timer_Data {
	TimerDev_t *pTimer;			//!< Timer bound to the device, NULL if free
	const SamdTimerOps_t *pOps;	//!< Register operations
	void *pReg;					//!< tc_count16_registers_t or tcc_registers_t, NULL for the RTC
	SamdTimerTrig_t *pTrig;		//!< Triggers of the device
	IRQn_Type IrqNo;
	uint32_t ApbMask;			//!< PM APBC bit of the TC or TCC
	uint8_t GclkId;				//!< Generic clock channel
	uint8_t NbTrig;				//!< Number of triggers
	uint8_t CntBits;			//!< Counter size in bits
	int8_t EvUser;				//!< Event user of the capture channel, -1 none
	uint32_t BaseFreq;			//!< Counter clock before the prescaler
	uint32_t Prescaler;			//!< PRESCALER field value
	uint32_t Epoch;				//!< Changed by a counter reset
	uint64_t Floor;				//!< Last count returned, counts never go back
	uint32_t Compare;			//!< RTC compare value written last
	uint32_t StopCount;			//!< TCC count when the counter stopped
	bool bRunning;
	bool bOverflow;				//!< Overflow event to report
	bool bResetOvf;				//!< RTC overflow flag from before a reset to drop
	bool bCapture;				//!< Last compare channel captures the external trigger
	bool bTick;					//!< RTC tick interrupt
	bool bExtOn;				//!< External trigger on, ExtChan and ExtLine valid
	bool bTickOn;				//!< Tick event channel open, TickChan valid
	int8_t ExtChan;				//!< Event channel of the external trigger
	int8_t ExtLine;				//!< EIC line of the external trigger
	int8_t TickChan;			//!< Event channel of the tick interrupt
	uint64_t ExtCount;			//!< Count captured at the last external trigger
};

#pragma pack(pop)

extern SamdTimerData_t g_SamdTimerData[SAMD_TIMER_MAXCNT];

#ifdef __cplusplus
extern "C" {
#endif

void SamdTimerIrq(int DevNo);

// RTC, DevNo 0
bool SamdRtcSetup(SamdTimerData_t *pData);
bool SamdRtcWait(SamdTimerData_t *pData, bool bFresh);	//!< Count readable, no synchronization in progress
void SamdRtcResume(SamdTimerData_t *pData);	//!< Continuous read started again after writes
bool SamdRtcEvent(SamdTimerData_t *pData);	//!< Interrupt: SYNCRDY served, compare or overflow pending
bool SamdRtcRun(SamdTimerData_t *pData, bool bRun);
bool SamdRtcReset(SamdTimerData_t *pData);
bool SamdRtcRead(SamdTimerData_t *pData, uint32_t *pCount);
bool SamdRtcOverflow(SamdTimerData_t *pData);
bool SamdRtcArm(SamdTimerData_t *pData, int TrigNo, bool bEnable);
bool SamdRtcMatched(SamdTimerData_t *pData, int TrigNo);
uint32_t SamdRtcTickGen(SamdTimerData_t *pData);	//!< Event generator of the tick, 0 none
uint32_t SamdRtcLag(SamdTimerData_t *pData);	//!< Counts a continuous read is late by

// TC
bool SamdTcSetup(SamdTimerData_t *pData);
bool SamdTcRun(SamdTimerData_t *pData, bool bRun);
bool SamdTcReset(SamdTimerData_t *pData);
bool SamdTcRead(SamdTimerData_t *pData, uint32_t *pCount);
bool SamdTcOverflow(SamdTimerData_t *pData);
bool SamdTcArm(SamdTimerData_t *pData, int TrigNo, bool bEnable);
bool SamdTcMatched(SamdTimerData_t *pData, int TrigNo);
bool SamdTcCapture(SamdTimerData_t *pData, bool bEnable);
bool SamdTcCaptured(SamdTimerData_t *pData, uint32_t *pCount);

// TCC
bool SamdTccSetup(SamdTimerData_t *pData);
bool SamdTccRun(SamdTimerData_t *pData, bool bRun);
bool SamdTccReset(SamdTimerData_t *pData);
bool SamdTccRead(SamdTimerData_t *pData, uint32_t *pCount);
bool SamdTccOverflow(SamdTimerData_t *pData);
bool SamdTccArm(SamdTimerData_t *pData, int TrigNo, bool bEnable);
bool SamdTccMatched(SamdTimerData_t *pData, int TrigNo);
bool SamdTccCapture(SamdTimerData_t *pData, bool bEnable);
bool SamdTccCaptured(SamdTimerData_t *pData, uint32_t *pCount);

#ifdef __cplusplus
}
#endif

#endif // __TIMER_SAMD_H__
