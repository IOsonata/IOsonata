/**-------------------------------------------------------------------------
@file	syslog.c

@brief	System logger implementation.

 Formats a SysStatus_t record into one text line and transmits it through
 the configured DeviceIntrf output. The line is built in a stack local
 buffer, so the function is reentrant and uses no static state.

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
	pLog->MinType = MinType;
	pLog->Marker = SYSLOG_INIT_MARKER;
}

int SysLogStatus(SysLog_t * const pLog, SysStatus_t Status, const char *pDetail)
{
	char line[SYSLOG_LINE_MAX];
	int len = 0;

	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER || pLog->pSink == 0)
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
		len += snprintf(line + len, sizeof(line) - len, "[%lu] ",
						(unsigned long)tick);
	}

	// Type tag, module id, code.
	len += snprintf(line + len, sizeof(line) - len, "%c:%03lX:%04lX",
					SysLogTypeTag(Status),
					(unsigned long)StatusModId(Status),
					(unsigned long)StatusCode(Status));

	// Detail string when supplied.
	if (pDetail != 0)
	{
		len += snprintf(line + len, sizeof(line) - len, " %s", pDetail);
	}

	len += snprintf(line + len, sizeof(line) - len, "\r\n");

	// snprintf returns the untruncated length, clamp to buffer size.
	if (len > (int)sizeof(line))
	{
		len = (int)sizeof(line);
	}

	return DeviceIntrfTx(pLog->pSink, pLog->SinkAddr, (const uint8_t *)line, len);
}

int SysLogVPrintf(SysLog_t * const pLog, const char *pFormat, va_list Args)
{
	char line[SYSLOG_LINE_MAX];
	int len;

	if (pLog == 0 || pLog->Marker != SYSLOG_INIT_MARKER || pLog->pSink == 0)
	{
		return 0;
	}

	len = vsnprintf(line, sizeof(line), pFormat, Args);

	// vsnprintf returns the untruncated length, clamp to buffer size.
	if (len < 0)
	{
		return 0;
	}
	if (len > (int)sizeof(line))
	{
		len = (int)sizeof(line);
	}

	return DeviceIntrfTx(pLog->pSink, pLog->SinkAddr, (const uint8_t *)line, len);
}

int SysLogPrintf(SysLog_t * const pLog, const char *pFormat, ...)
{
	va_list args;
	int len;

	va_start(args, pFormat);
	len = SysLogVPrintf(pLog, pFormat, args);
	va_end(args);

	return len;
}

int SysLog::Printf(const char *pFormat, ...)
{
	va_list args;
	int len;

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
SysLog * const SysLogGetInstance(void)
{
	return &g_SysLog;
}

// C handle access to the same global object.
extern "C" SysLog_t * const SysLogGet(void)
{
	return g_SysLog;
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

	pStack->Count = 0;
	pStack->PoppedSincePush = false;
}

bool SysStatusStackPush(SysStatusStack_t * const pStack, SysStatus_t Status)
{
	if (pStack == 0)
	{
		return false;
	}

	// Start a new chain if a read cycle has begun.
	if (pStack->PoppedSincePush)
	{
		pStack->Count = 0;
		pStack->PoppedSincePush = false;
	}

	// Reject when full, preserves the originating cause.
	if (pStack->Count >= SYSSTATUS_STACK_DEPTH)
	{
		return false;
	}

	pStack->Entry[pStack->Count] = Status;
	pStack->Count++;

	return true;
}

SysStatus_t SysStatusStackPop(SysStatusStack_t * const pStack)
{
	if (pStack == 0 || pStack->Count <= 0)
	{
		return SYSSTATUS_OK;
	}

	pStack->Count--;
	pStack->PoppedSincePush = true;

	return pStack->Entry[pStack->Count];
}

SysStatus_t SysStatusStackPeek(SysStatusStack_t * const pStack)
{
	if (pStack == 0 || pStack->Count <= 0)
	{
		return SYSSTATUS_OK;
	}

	return pStack->Entry[pStack->Count - 1];
}

int SysStatusStackCount(SysStatusStack_t * const pStack)
{
	if (pStack == 0)
	{
		return 0;
	}

	return pStack->Count;
}

// IOsonata global instance. Zero initialized, so Count is 0 (empty) at
// startup with no explicit init required.
static SysStatusStack_t g_SysStatusStack;

SYSSTATUS_WEAK SysStatusStack_t * const SysStatusStackGet(void)
{
	return &g_SysStatusStack;
}

SYSSTATUS_WEAK bool SysStatusPush(SysStatus_t Status)
{
	return SysStatusStackPush(&g_SysStatusStack, Status);
}

SYSSTATUS_WEAK SysStatus_t SysStatusPop(void)
{
	return SysStatusStackPop(&g_SysStatusStack);
}

SYSSTATUS_WEAK SysStatus_t SysStatusPeek(void)
{
	return SysStatusStackPeek(&g_SysStatusStack);
}
