/**-------------------------------------------------------------------------
@file	syslog.cpp

@brief	System logger implementation.

 Formats a SysStatus_t record into one text line, in place in a CFifo block
 built from caller supplied memory. The store and the output
 transport are independent, so records survive a transport that is absent,
 detached, or not yet ready.

@author	Hoang Nguyen Hoan
@date	May. 29, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include <stdio.h>
#include <string.h>

#include "syslog.h"

#if defined(__arm__) || defined(__ICCARM__) || (defined(__riscv) && defined(__riscv_zicsr))
#include "coredev/interrupt.h"
#define SYSLOG_USE_IRQ_LOCK    1
#endif

//
// Status stack. IOsonata default storage for status provenance.
// Independent of the logger above.
//

// Weak linkage for override of the global accessors.
#ifndef SYSSTATUS_WEAK
#if defined(__GNUC__) || defined(__clang__)
#define SYSSTATUS_WEAK  __attribute__((weak))
#else
#define SYSSTATUS_WEAK
#endif
#endif

// IOsonata global instance. Zero initialized, so Count is 0 (empty) at
// startup with no explicit init required.
static SysStatusStack_t g_SysStatusStack;


//
// Library global logger instance. File static, one per image. Constructed
// dormant, stays inert until configured.
//
static SysLog g_SysLog;
// A generic TX-ready event may interrupt an active foreground flush.
// Preserve that notification until the active flush has retried.
static bool s_bSysLogReadyDuringFlush;


// Return one character tag for the status type field.
static char SysLogTypeTag(SysStatus_t Status)
{
	switch (StatusType(Status))
	{
	case SYSSTATUS_TYPE_WRN:	return 'W';
	case SYSSTATUS_TYPE_ERR:	return 'E';
	case SYSSTATUS_TYPE_FERR:	return 'F';
	default:					return 'R';		// Runtime
	}
}

// Write fixed status fields directly into the record. Keep the same prefix
// when truncated, and always leave space for the terminating NUL.
static int SysLogAppendText(char *pLine, int LineSize, int Pos, const char *pText)
{
	while (*pText != 0 && Pos < LineSize - 1)
	{
		pLine[Pos++] = *pText++;
	}
	pLine[Pos] = 0;
	return Pos;
}

static int SysLogAppendTick(char *pLine, int LineSize, uint64_t Tick)
{
	// A uint64_t needs at most 20 decimal digits. Only timestamped records
	// use this scratch space; un-timestamped printf records do not.
	char digits[21];
	int pos = sizeof(digits) - 1;
	digits[pos] = 0;
	do
	{
		digits[--pos] = '0' + Tick % 10U;
		Tick /= 10U;
	} while (Tick != 0U);

	int len = SysLogAppendText(pLine, LineSize, 0, "[");
	len = SysLogAppendText(pLine, LineSize, len, digits + pos);
	return SysLogAppendText(pLine, LineSize, len, "] ");
}

static uintptr_t SysIrqLock(void)
{
#if defined(SYSLOG_USE_IRQ_LOCK)
	return (uintptr_t)DisableInterrupt();
#else
	return 0;
#endif
}

static void SysIrqUnlock(uintptr_t State)
{
#if defined(SYSLOG_USE_IRQ_LOCK)
#if defined(__arm__) || defined(__ICCARM__)
	EnableInterrupt((uint32_t)State);
#else
	EnableInterrupt(State);
#endif
#else
	(void)State;
#endif
}

bool SysLogInitNoTimer(SysLog_t * const pLog, const SysLogCfg_t * const pCfg,
				DevIntrf_t * const pSink, uint32_t SinkAddr, uint32_t MinType)
{
	if (pLog == 0)
	{
		return false;
	}

	// Clear first so a failed init leaves a dormant instance rather than one
	// still pointing at whatever was there before.
	pLog->Marker = 0;
	pLog->hFifo = 0;

	if (pCfg == 0)
	{
		return false;
	}

	// One record is at least one character and the NUL that ends it. There is
	// no upper limit; every record is formatted in place in its own block.
	const uint32_t recordLen = pCfg->RecordLen;
	if (recordLen < 2U)
	{
		return false;
	}

	// Reserve one complete record outside the FIFO for in-progress TX.
	if (pCfg->pMem == nullptr ||
		recordLen > UINT32_MAX - SYSLOG_RECORD_META_SIZE ||
		pCfg->MemSize < sizeof(CFifo_t) +
			3U * (recordLen + SYSLOG_RECORD_META_SIZE))
	{
		return false;
	}
	hCFifo_t hFifo = CFifoInit(pCfg->pMem,
					pCfg->MemSize - 2U * (recordLen + SYSLOG_RECORD_META_SIZE),
					recordLen + SYSLOG_RECORD_META_SIZE, pCfg->bBlocking);
	if (hFifo == 0)
	{
		return false;
	}

	pLog->pSink = nullptr;
	pLog->SinkAddr = 0U;
	pLog->pTimer = nullptr;
	pLog->FormatTime = nullptr;
	pLog->MinType = MinType & SYSSTATUS_TYPE_MASK;
	pLog->hFifo = hFifo;
	pLog->pTxBuffer = pCfg->pMem + pCfg->MemSize - 2U * (recordLen + SYSLOG_RECORD_META_SIZE);
	pLog->pFormatBuffer = pLog->pTxBuffer + recordLen + SYSLOG_RECORD_META_SIZE;
	pLog->TxLen = 0U;
	pLog->TxSent = 0U;
	pLog->bFlushing = false;
	if (pLog == SysLogGet())
	{
		s_bSysLogReadyDuringFlush = false;
	}
	pLog->Marker = SYSLOG_INIT_MARKER;
	SysLogSetSink(pLog, pSink, SinkAddr);

	return true;
}

// Kept out of the ordinary logger path so an untimestamped logger does not
// pull in timer conversion and 64-bit decimal arithmetic.
static int SysLogFormatTime(TimerDev_t *pTimer, char *pLine, int Len, bool bTicks)
{
	uint64_t time = bTicks ? TimerGetTickCount(pTimer) : TimerGetMilisecond(pTimer);
	return SysLogAppendTick(pLine, Len, time);
}

bool SysLogInit(SysLog_t * const pLog, const SysLogCfg_t * const pCfg,
				DevIntrf_t * const pSink, uint32_t SinkAddr,
				TimerDev_t * const pTimer, uint32_t MinType)
{
	if (!SysLogInitNoTimer(pLog, pCfg, pSink, SinkAddr, MinType))
	{
		return false;
	}
	pLog->pTimer = pTimer;
	pLog->FormatTime = pTimer != nullptr ? SysLogFormatTime : nullptr;
	return true;
}

void SysLogSetSink(SysLog_t * const pLog, DevIntrf_t * const pSink,
				   uint32_t SinkAddr)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER)
	{
		return;
	}

	// Never override an application-owned event callback. Release only
	// a callback SysLog installed when detaching or switching sinks.
	if (pLog == SysLogGet() && pLog->pSink != nullptr &&
		pLog->pSink != pSink &&
		pLog->pSink->EvtCB == SysLogDevIntrfEvent)
	{
		pLog->pSink->EvtCB = nullptr;
	}
	if (pSink != pLog->pSink || SinkAddr != pLog->SinkAddr)
	{
		pLog->TxSent = 0U;
	}
	pLog->pSink = pSink;
	pLog->SinkAddr = SinkAddr;
	if (pLog == SysLogGet())
	{
		s_bSysLogReadyDuringFlush = false;
	}
	if (pLog == SysLogGet() && pSink != nullptr && pSink->EvtCB == nullptr)
	{
		pSink->EvtCB = SysLogDevIntrfEvent;
	}
}

int SysLogDevIntrfEvent(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
		uint8_t *pBuffer, int Len)
{
	SysLog_t *pLog;

	(void)pBuffer;
	(void)Len;
	if (pDev == nullptr || EvtId != DEVINTRF_EVT_TX_READY)
	{
		return 0;
	}
	pLog = SysLogGet();
	if (pLog->Marker != SYSLOG_INIT_MARKER || pLog->pSink != pDev)
	{
		return 0;
	}
	return SysLogFlush(pLog);
}

// One consumer copies records out of CFifo into a separate transfer buffer.
// DeviceIntrfTx owns every accepted byte; only the unaccepted suffix is kept.
// Submit no more than one SysLog record per call. The next invocation,
// usually a sink TX_READY event, continues a partial record or takes the
// next FIFO record. Nothing from separate printf calls is combined here.
int SysLogFlush(SysLog_t * const pLog)
{
	uintptr_t state;
	uint8_t *pBlock;
	uint32_t recordLen;
	uint32_t len;
	uint32_t sent;
	int count;
	int total;
	int err;
	bool retry;
	bool next;

	if (pLog == nullptr || pLog->Marker != SYSLOG_INIT_MARKER ||
		pLog->pSink == nullptr)
	{
		return 0;
	}

	state = SysIrqLock();
	if (pLog->bFlushing)
	{
		// An interrupt during submission must not operate on the same
		// transfer buffer. Let the active owner service its notification.
		if (pLog == SysLogGet())
		{
			s_bSysLogReadyDuringFlush = true;
		}
		SysIrqUnlock(state);
		return 0;
	}
	pLog->bFlushing = true;
	SysIrqUnlock(state);

	recordLen = CFifoBlockSize(pLog->hFifo) - SYSLOG_RECORD_META_SIZE;
	total = 0;
	err = 0;

	// Each pass owns one record. A deferred TX_READY is another logical
	// flush request, serviced here without nesting flush calls.
	do
	{
		state = SysIrqLock();
		if (pLog->TxLen == 0U)
		{
			pBlock = CFifoGet(pLog->hFifo);
			if (pBlock == nullptr)
			{
				// No record remains for a deferred ready notification.
				if (pLog == SysLogGet())
				{
					s_bSysLogReadyDuringFlush = false;
				}
				pLog->bFlushing = false;
				SysIrqUnlock(state);
				break;
			}
			memcpy(&len, pBlock, SYSLOG_RECORD_META_SIZE);
			if (len >= recordLen)
			{
				len = recordLen - 1U;
			}
			memcpy(pLog->pTxBuffer,
				pBlock + SYSLOG_RECORD_META_SIZE, recordLen);
			pLog->TxLen = len;
			pLog->TxSent = 0U;
		}
		len = pLog->TxLen;
		sent = pLog->TxSent;
		SysIrqUnlock(state);

		while (sent < len)
		{
			count = DeviceIntrfTx(pLog->pSink, pLog->SinkAddr,
				pLog->pTxBuffer + sent, (int)(len - sent));
			if (count > 0)
			{
				sent += (uint32_t)count;
				total += count;
			}

			state = SysIrqLock();
			pLog->TxSent = sent;
			// Consume a deferred event only if the current record
			// still has bytes to submit. Otherwise it belongs to
			// continuation after the completed record.
			retry = sent < len && pLog == SysLogGet() &&
				s_bSysLogReadyDuringFlush;
			if (retry)
			{
				s_bSysLogReadyDuringFlush = false;
			}
			SysIrqUnlock(state);

			if (sent < len && !retry)
			{
				if (count < 0 && total == 0)
				{
					err = count;
				}
				break;
			}
		}

		state = SysIrqLock();
		if (sent >= len)
		{
			pLog->TxLen = 0U;
			pLog->TxSent = 0U;
		}
		// Once a record completes, a suppressed TX_READY must be
		// serviced before ownership is released. Never drop this event.
		next = sent >= len && pLog == SysLogGet() &&
			s_bSysLogReadyDuringFlush;
		if (next)
		{
			s_bSysLogReadyDuringFlush = false;
		}
		else
		{
			pLog->bFlushing = false;
		}
		SysIrqUnlock(state);
	} while (next);

	return err < 0 ? err : total;
}

int SysLogStatus(SysLog_t * const pLog, SysStatus_t Status, const char *pDetail)
{
	int len = 0;

	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER)
	{
		return 0;
	}

	// Type field filter, before a block is taken so a filtered record cannot
	// evict a stored one.
	if (StatusType(Status) < pLog->MinType)
	{
		return 0;
	}

	// Prepare one complete record before publishing it to CFifo.
	const int recordLen = (int)(CFifoBlockSize(pLog->hFifo) - SYSLOG_RECORD_META_SIZE);

	uint8_t *pBlock = pLog->pFormatBuffer;
	char *pLine = (char *)(pBlock + SYSLOG_RECORD_META_SIZE);

	if (pLog->FormatTime != nullptr)
	{
		len = pLog->FormatTime(pLog->pTimer, pLine, recordLen, true);
	}

	// Type tag, three hex module digits and four hex code digits. The
	// separators and digits use shifts, without printf's format parser.
	char prefix[11];
	prefix[0] = SysLogTypeTag(Status);
	prefix[1] = ':';
	prefix[5] = ':';
	prefix[10] = 0;
	uint32_t value = StatusModId(Status);
	for (int i = 4; i >= 2; i--)
	{
		unsigned digit = value & 15U;
		prefix[i] = digit < 10U ? '0' + digit : 'A' + digit - 10U;
		value >>= 4;
	}
	value = StatusCode(Status);
	for (int i = 9; i >= 6; i--)
	{
		unsigned digit = value & 15U;
		prefix[i] = digit < 10U ? '0' + digit : 'A' + digit - 10U;
		value >>= 4;
	}
	len = SysLogAppendText(pLine, recordLen, len, prefix);
	if (pDetail != 0)
	{
		len = SysLogAppendText(pLine, recordLen, len, " ");
		len = SysLogAppendText(pLine, recordLen, len, pDetail);
	}
	len = SysLogAppendText(pLine, recordLen, len, "\r\n");

	pLine[len] = 0;
	uint32_t written = (uint32_t)len;
	memcpy(pBlock, &written, SYSLOG_RECORD_META_SIZE);
	uintptr_t state = SysIrqLock();
	uint8_t *pDst = CFifoPut(pLog->hFifo);
	if (pDst != nullptr)
	{
		memcpy(pDst, pBlock, CFifoBlockSize(pLog->hFifo));
	}
	SysIrqUnlock(state);
	if (pDst == nullptr)
	{
		return 0;
	}

	// A record goes out as soon as it is stored when a transport is attached.
	// SysLogFlush does not log, so this cannot recurse.
	if (pLog->pSink != 0)
	{
		(void)SysLogFlush(pLog);
	}

	return len;
}

int SysLogVPrintf(SysLog_t * const pLog, const char *pFormat, va_list Args)
{
	uint8_t *pBlock;
	char *pLine;
	uint32_t recordLen;
	uint32_t written;
	int pre;
	int formatted;
	int len;

	if (pLog == nullptr || pLog->Marker != SYSLOG_INIT_MARKER ||
		pFormat == nullptr)
	{
		return 0;
	}

	// Format into private staging memory, not an unpublished FIFO slot.
	recordLen = CFifoBlockSize(pLog->hFifo) - SYSLOG_RECORD_META_SIZE;
	pBlock = pLog->pFormatBuffer;
	pLine = (char *)pBlock + SYSLOG_RECORD_META_SIZE;

	pre = 0;
	if (pLog->FormatTime != nullptr)
	{
		pre = pLog->FormatTime(pLog->pTimer, pLine,
			(int)recordLen, false);
	}
	if (pre < 0)
	{
		pre = 0;
	}
	if ((uint32_t)pre >= recordLen)
	{
		pre = (int)recordLen - 1;
	}

	// vsnprintf returns the number of characters it wanted to write,
	// excluding NUL. Clamp it to the bytes actually stored before
	// publishing the record. This includes any CR/LF that fits.
	formatted = vsnprintf(pLine + pre, (size_t)(recordLen - (uint32_t)pre),
		pFormat, Args);
	if (formatted < 0)
	{
		pLine[0] = 0;
		len = 0;
	}
	else
	{
		len = formatted;
		if ((uint32_t)len >= recordLen - (uint32_t)pre)
		{
			len = (int)(recordLen - (uint32_t)pre - 1U);
		}
		len += pre;
	}
	pLine[len] = 0;
	written = (uint32_t)len;
	memcpy(pBlock, &written, SYSLOG_RECORD_META_SIZE);
	// CFifoPut publishes immediately. Keep interrupts masked until
	// the freshly published slot contains the complete formatted record.
	uintptr_t state = SysIrqLock();
	uint8_t *pDst = CFifoPut(pLog->hFifo);
	if (pDst != nullptr)
	{
		memcpy(pDst, pBlock, CFifoBlockSize(pLog->hFifo));
	}
	SysIrqUnlock(state);
	if (pDst == nullptr)
	{
		return 0;
	}

	if (pLog->pSink != nullptr)
	{
		(void)SysLogFlush(pLog);
	}

	return len;
}

int SysLogPrintf(SysLog_t * const pLog, const char *pFormat, ...)
{
	va_list args;
	int len;

	if (pFormat == 0)
	{
		return 0;
	}

	va_start(args, pFormat);
	len = SysLogVPrintf(pLog, pFormat, args);
	va_end(args);

	return len;
}

int SysLog::Printf(const char *pFormat, ...)
{
	va_list args;
	int len;

	if (pFormat == 0)
	{
		return 0;
	}

	va_start(args, pFormat);
	len = SysLogVPrintf(&vLog, pFormat, args);
	va_end(args);

	return len;
}

// C++ direct access to the global object.
SysLog *SysLogGetInstance(void)
{
	return &g_SysLog;
}

// C handle access to the same global object.
extern "C" SysLog_t *SysLogGet(void)
{
	return (SysLog_t *)g_SysLog;
}

void SysStatusStackReset(SysStatusStack_t * const pStack)
{
	if (pStack == 0)
	{
		return;
	}

	uintptr_t state = SysIrqLock();

	pStack->Count = 0;
	pStack->PoppedSincePush = false;

	SysIrqUnlock(state);
}

bool SysStatusStackPush(SysStatusStack_t * const pStack, SysStatus_t Status)
{
	bool retval = false;

	if (pStack == 0)
	{
		return false;
	}

	uintptr_t state = SysIrqLock();

	// Start a new chain if a read cycle has begun.
	if (pStack->PoppedSincePush)
	{
		pStack->Count = 0;
		pStack->PoppedSincePush = false;
	}

	// Reject when full, preserves the originating cause.
	if (pStack->Count < SYSSTATUS_STACK_DEPTH)
	{
		pStack->Entry[pStack->Count] = Status;
		pStack->Count++;
		retval = true;
	}

	SysIrqUnlock(state);

	return retval;
}

SysStatus_t SysStatusStackPop(SysStatusStack_t * const pStack)
{
	SysStatus_t retval = SYSSTATUS_OK;

	if (pStack == 0)
	{
		return SYSSTATUS_OK;
	}

	uintptr_t state = SysIrqLock();

	if (pStack->Count > 0)
	{
		pStack->Count--;
		pStack->PoppedSincePush = true;
		retval = pStack->Entry[pStack->Count];
	}

	SysIrqUnlock(state);

	return retval;
}

SysStatus_t SysStatusStackPeek(SysStatusStack_t * const pStack)
{
	SysStatus_t retval = SYSSTATUS_OK;

	if (pStack == 0)
	{
		return SYSSTATUS_OK;
	}

	uintptr_t state = SysIrqLock();

	if (pStack->Count > 0)
	{
		retval = pStack->Entry[pStack->Count - 1];
	}

	SysIrqUnlock(state);

	return retval;
}

int SysStatusStackCount(SysStatusStack_t * const pStack)
{
	int retval = 0;

	if (pStack == 0)
	{
		return 0;
	}

	uintptr_t state = SysIrqLock();

	retval = pStack->Count;

	SysIrqUnlock(state);

	return retval;
}

SYSSTATUS_WEAK SysStatusStack_t *SysStatusStackGet(void)
{
	return &g_SysStatusStack;
}

SYSSTATUS_WEAK bool SysStatusPush(SysStatus_t Status)
{
	return SysStatusStackPush(SysStatusStackGet(), Status);
}

SYSSTATUS_WEAK SysStatus_t SysStatusPop(void)
{
	return SysStatusStackPop(SysStatusStackGet());
}

SYSSTATUS_WEAK SysStatus_t SysStatusPeek(void)
{
	return SysStatusStackPeek(SysStatusStackGet());
}
