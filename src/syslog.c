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
}

int SysLogStatus(SysLog_t * const pLog, SysStatus_t Status, const char *pDetail)
{
	char line[SYSLOG_LINE_MAX];
	int len = 0;

	if (pLog == 0 || pLog->pSink == 0)
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
