/**-------------------------------------------------------------------------
@file	syslog.cpp

@brief	System logger implementation.

 Formats a SysStatus_t record into one text line and either queues the complete
 record in an optional CFifo or transmits it through the configured DeviceIntrf
 output. The FIFO and output transport are independent.

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
#define SYSSTATUS_STACK_USE_IRQ_LOCK    1
#endif

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

static uintptr_t SysStatusStackLock(void)
{
#if defined(SYSSTATUS_STACK_USE_IRQ_LOCK)
	return (uintptr_t)DisableInterrupt();
#else
	return 0;
#endif
}

static void SysStatusStackUnlock(uintptr_t State)
{
#if defined(SYSSTATUS_STACK_USE_IRQ_LOCK)
#if defined(__arm__) || defined(__ICCARM__)
	EnableInterrupt((uint32_t)State);
#else
	EnableInterrupt(State);
#endif
#else
	(void)State;
#endif
}

// Queue one complete formatted record when a FIFO is configured. Without a
// FIFO, send it directly through DeviceIntrf. Transfer behavior belongs to the
// DeviceIntrf implementation; SysLog performs one DeviceIntrfTx call.
static int SysLogOutput(SysLog_t * const pLog, const char *pLine, int Len)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER ||
		pLine == 0 || Len <= 0)
	{
		return 0;
	}

	if (pLog->hFifo != 0)
	{
		uint32_t blockSize = CFifoBlockSize(pLog->hFifo);
		if (blockSize < 2U)
		{
			return 0;
		}

		uint8_t *pBlock = CFifoPut(pLog->hFifo);
		if (pBlock == 0)
		{
			return 0;
		}

		int count = Len;
		if ((uint32_t)count >= blockSize)
		{
			count = (int)blockSize - 1;
		}

		memcpy(pBlock, pLine, (size_t)count);
		pBlock[count] = 0;
		return count;
	}

	if (pLog->pSink == 0)
	{
		return 0;
	}

	return DeviceIntrfTx(pLog->pSink, pLog->SinkAddr,
					 (const uint8_t *)pLine, Len);
}

void SysLogInit(SysLog_t * const pLog, DevIntrf_t * const pSink,
				uint32_t SinkAddr, TimerDev_t * const pTimer, uint32_t MinType)
{
	if (pLog == 0)
	{
		return;
	}

	pLog->pSink = pSink;
	pLog->SinkAddr = SinkAddr;
	pLog->pTimer = pTimer;
	pLog->MinType = MinType & SYSSTATUS_TYPE_MASK;
	pLog->hFifo = 0;
	pLog->Marker = SYSLOG_INIT_MARKER;
}

void SysLogSetBuffer(SysLog_t * const pLog, hCFifo_t const hFifo)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER)
	{
		return;
	}

	pLog->hFifo = hFifo;
}

void SysLogSetSink(SysLog_t * const pLog, DevIntrf_t * const pSink,
				   uint32_t SinkAddr)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER)
	{
		return;
	}

	pLog->pSink = pSink;
	pLog->SinkAddr = SinkAddr;
}

int SysLogFlush(SysLog_t * const pLog)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER ||
		pLog->hFifo == 0 || pLog->pSink == 0)
	{
		return 0;
	}

	const uint32_t blockSize = CFifoBlockSize(pLog->hFifo);
	if (blockSize == 0U)
	{
		return 0;
	}

	uint8_t *pBlock = CFifoPeek(pLog->hFifo);
	if (pBlock == 0)
	{
		return 0;
	}

	uint32_t len = 0U;
	while (len < blockSize && pBlock[len] != 0U)
	{
		len++;
	}

	if (len == 0U)
	{
		// Remove an empty record so one failed format cannot block the FIFO.
		(void)CFifoGet(pLog->hFifo);
		return 0;
	}

	int count = DeviceIntrfTx(pLog->pSink, pLog->SinkAddr,
						   pBlock, (int)len);
	if (count == (int)len)
	{
		(void)CFifoGet(pLog->hFifo);
	}

	return count;
}

int SysLogStatus(SysLog_t * const pLog, SysStatus_t Status, const char *pDetail)
{
	char line[SYSLOG_LINE_MAX];
	int len = 0;

	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER)
	{
		return 0;
	}

	// Type field filter.
	if (StatusType(Status) < pLog->MinType)
	{
		return 0;
	}

	// Timestamp prefix when a Timer is configured.
	if (pLog->pTimer != 0)
	{
		uint64_t tick = TimerGetTickCount(pLog->pTimer);
		len = SysLogAppend(line, sizeof(line), len, "[%llu] ",
						   (unsigned long long)tick);
	}

	// Type tag, module id, code.
	len = SysLogAppend(line, sizeof(line), len, "%c:%03lX:%04lX",
					   SysLogTypeTag(Status),
					   (unsigned long)StatusModId(Status),
					   (unsigned long)StatusCode(Status));

	// Detail string when supplied.
	if (pDetail != 0)
	{
		len = SysLogAppend(line, sizeof(line), len, " %s", pDetail);
	}

	len = SysLogAppend(line, sizeof(line), len, "\r\n");

	return SysLogOutput(pLog, line, len);
}

int SysLogVPrintf(SysLog_t * const pLog, const char *pFormat, va_list Args)
{
	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER || pFormat == 0)
	{
		return 0;
	}

	if (pLog->hFifo != 0)
	{
		const uint32_t blockSize = CFifoBlockSize(pLog->hFifo);
		if (blockSize < 2U)
		{
			return 0;
		}

		uint8_t *pBlock = CFifoPut(pLog->hFifo);
		if (pBlock == 0)
		{
			return 0;
		}

		int len = vsnprintf((char *)pBlock, (size_t)blockSize, pFormat, Args);
		if (len < 0)
		{
			pBlock[0] = 0U;
			return 0;
		}

		if ((uint32_t)len >= blockSize)
		{
			len = (int)blockSize - 1;
		}

		return len;
	}

	if (pLog->pSink == 0)
	{
		return 0;
	}

	char line[SYSLOG_LINE_MAX];
	int len = vsnprintf(line, sizeof(line), pFormat, Args);
	if (len < 0)
	{
		return 0;
	}

	if (len >= (int)sizeof(line))
	{
		len = (int)sizeof(line) - 1;
	}

	return DeviceIntrfTx(pLog->pSink, pLog->SinkAddr,
					 (const uint8_t *)line, len);
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

//
// Library global logger instance. File static, one per image. Constructed
// dormant, stays inert until configured.
//
static SysLog g_SysLog;

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

void SysStatusStackReset(SysStatusStack_t * const pStack)
{
	if (pStack == 0)
	{
		return;
	}

	uintptr_t state = SysStatusStackLock();

	pStack->Count = 0;
	pStack->PoppedSincePush = false;

	SysStatusStackUnlock(state);
}

bool SysStatusStackPush(SysStatusStack_t * const pStack, SysStatus_t Status)
{
	bool retval = false;

	if (pStack == 0)
	{
		return false;
	}

	uintptr_t state = SysStatusStackLock();

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

	SysStatusStackUnlock(state);

	return retval;
}

SysStatus_t SysStatusStackPop(SysStatusStack_t * const pStack)
{
	SysStatus_t retval = SYSSTATUS_OK;

	if (pStack == 0)
	{
		return SYSSTATUS_OK;
	}

	uintptr_t state = SysStatusStackLock();

	if (pStack->Count > 0)
	{
		pStack->Count--;
		pStack->PoppedSincePush = true;
		retval = pStack->Entry[pStack->Count];
	}

	SysStatusStackUnlock(state);

	return retval;
}

SysStatus_t SysStatusStackPeek(SysStatusStack_t * const pStack)
{
	SysStatus_t retval = SYSSTATUS_OK;

	if (pStack == 0)
	{
		return SYSSTATUS_OK;
	}

	uintptr_t state = SysStatusStackLock();

	if (pStack->Count > 0)
	{
		retval = pStack->Entry[pStack->Count - 1];
	}

	SysStatusStackUnlock(state);

	return retval;
}

int SysStatusStackCount(SysStatusStack_t * const pStack)
{
	int retval = 0;

	if (pStack == 0)
	{
		return 0;
	}

	uintptr_t state = SysStatusStackLock();

	retval = pStack->Count;

	SysStatusStackUnlock(state);

	return retval;
}

// IOsonata global instance. Zero initialized, so Count is 0 (empty) at
// startup with no explicit init required.
static SysStatusStack_t g_SysStatusStack;

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
