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

// Append formatted text at Pos without ever advancing beyond the usable buffer.
// Returns the new length (bytes to transmit, excluding the trailing NUL).
static int SysLogAppend(char *pLine, int LineSize, int Pos,
						const char *pFormat, ...)
{
	if (pLine == 0 || pFormat == 0 || LineSize <= 1)
	{
		return 0;
	}

	if (Pos < 0)
	{
		Pos = 0;
	}

	if (Pos >= LineSize)
	{
		return LineSize - 1;
	}

	va_list args;
	va_start(args, pFormat);
	int n = vsnprintf(&pLine[Pos], (size_t)(LineSize - Pos), pFormat, args);
	va_end(args);

	if (n < 0)
	{
		return Pos;
	}

	if (n >= (LineSize - Pos))
	{
		return LineSize - 1;
	}

	return Pos + n;
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

bool SysLogInit(SysLog_t * const pLog, const SysLogCfg_t * const pCfg,
				DevIntrf_t * const pSink, uint32_t SinkAddr,
				TimerDev_t * const pTimer, uint32_t MinType)
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

	hCFifo_t hFifo = CFifoInit(pCfg->pMem, pCfg->MemSize, recordLen,
							   pCfg->bBlocking);
	if (hFifo == 0)
	{
		return false;
	}

	pLog->pSink = pSink;
	pLog->SinkAddr = SinkAddr;
	pLog->pTimer = pTimer;
	pLog->MinType = MinType & SYSSTATUS_TYPE_MASK;
	pLog->hFifo = hFifo;
	pLog->HeadIdx = hFifo->GetIdx;
	pLog->HeadSent = 0U;
	pLog->bFlushing = false;
	pLog->Marker = SYSLOG_INIT_MARKER;

	return true;
}

void SysLogSetSink(SysLog_t * const pLog, DevIntrf_t * const pSink,
				   uint32_t SinkAddr)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER)
	{
		return;
	}

	// A different sink has none of the head record, so it gets all of it.
	if (pSink != pLog->pSink || SinkAddr != pLog->SinkAddr)
	{
		pLog->HeadSent = 0U;
	}
	pLog->pSink = pSink;
	pLog->SinkAddr = SinkAddr;
}

// Send records until the store is empty or the sink stops taking bytes.
// HeadSent counts the bytes of the head record already sent, so a sink that
// takes part of a record gets only the rest of it on the next call. HeadIdx
// is the store index of that record: when a full non-blocking store evicts
// it, the index moves and the new head starts from its first byte.
// Returns true when the sink stopped before the store was empty.
static bool SysLogDrain(SysLog_t * const pLog, int *pTotal, int *pErr)
{
	const uint32_t recordLen = CFifoBlockSize(pLog->hFifo);

	while (true)
	{
		uint32_t head = __atomic_load_n(&pLog->hFifo->GetIdx, __ATOMIC_ACQUIRE);
		uint8_t *pBlock = CFifoPeek(pLog->hFifo);
		if (pBlock == 0)
		{
			return false;
		}
		if (__atomic_load_n(&pLog->hFifo->GetIdx, __ATOMIC_ACQUIRE) != head)
		{
			// Evicted while it was being looked up, take the new head.
			continue;
		}
		if (head != pLog->HeadIdx)
		{
			pLog->HeadIdx = head;
			pLog->HeadSent = 0U;
		}

		uint32_t len = 0U;
		while (len < recordLen && pBlock[len] != 0U)
		{
			len++;
		}

		uint32_t sent = pLog->HeadSent;
		if (sent < len)
		{
			int count = DeviceIntrfTx(pLog->pSink, pLog->SinkAddr,
								   pBlock + sent, (int)(len - sent));
			if (count > 0)
			{
				*pTotal += count;
				sent += (uint32_t)count;
			}
			if (sent < len)
			{
				// Keep the record and what went out of it. A negative result
				// with nothing sent in this call is reported as it came back.
				pLog->HeadSent = sent;
				if (count < 0 && *pTotal == 0)
				{
					*pErr = count;
				}
				return true;
			}
		}

		// Sent in full, or an empty record that a failed format left behind.
		pLog->HeadSent = 0U;
		pLog->HeadIdx = head + 1U;
		(void)CFifoGet(pLog->hFifo);
	}
}

int SysLogFlush(SysLog_t * const pLog)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER || pLog->pSink == 0)
	{
		return 0;
	}

	// One flush at a time. A record stored by an interrupt while another
	// context is sending is left to that flush, which looks at the store again
	// before it lets go, so the record is not stranded.
	uintptr_t state = SysIrqLock();
	if (pLog->bFlushing)
	{
		SysIrqUnlock(state);
		return 0;
	}
	pLog->bFlushing = true;
	SysIrqUnlock(state);

	int total = 0;
	int err = 0;
	bool more;

	do {
		bool stopped = SysLogDrain(pLog, &total, &err);

		state = SysIrqLock();
		more = stopped == false && CFifoPeek(pLog->hFifo) != 0;
		pLog->bFlushing = more;
		SysIrqUnlock(state);
	} while (more);

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

	// Format in place. No scratch buffer, so the record length is free to be
	// whatever the store was configured with.
	const int recordLen = (int)CFifoBlockSize(pLog->hFifo);

	char *pLine = (char *)CFifoPut(pLog->hFifo);
	if (pLine == 0)
	{
		return 0;
	}

	// Timestamp prefix when a Timer is configured.
	if (pLog->pTimer != 0)
	{
		uint64_t tick = TimerGetTickCount(pLog->pTimer);
		len = SysLogAppend(pLine, recordLen, len, "[%llu] ",
						   (unsigned long long)tick);
	}

	// Type tag, module id, code.
	len = SysLogAppend(pLine, recordLen, len, "%c:%03lX:%04lX",
					   SysLogTypeTag(Status),
					   (unsigned long)StatusModId(Status),
					   (unsigned long)StatusCode(Status));

	// Detail string when supplied.
	if (pDetail != 0)
	{
		len = SysLogAppend(pLine, recordLen, len, " %s", pDetail);
	}

	len = SysLogAppend(pLine, recordLen, len, "\r\n");

	pLine[len] = 0;

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
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER || pFormat == 0)
	{
		return 0;
	}

	// Format straight into the block, no intermediate line buffer.
	const uint32_t recordLen = CFifoBlockSize(pLog->hFifo);

	uint8_t *pBlock = CFifoPut(pLog->hFifo);
	if (pBlock == 0)
	{
		return 0;
	}

	int len = vsnprintf((char *)pBlock, (size_t)recordLen, pFormat, Args);
	if (len < 0)
	{
		pBlock[0] = 0U;
		return 0;
	}

	if ((uint32_t)len >= recordLen)
	{
		len = (int)recordLen - 1;
	}

	if (pLog->pSink != 0)
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
