/**-------------------------------------------------------------------------
@file	syslog.h

@brief	System logger.

 Emits SysStatus_t records to a DeviceIntrf output. The output is any
 DeviceIntrf implementation (UART, USB CDC, SPI, BLE, SLIP). The optional
 timestamp source is any Timer implementation. The log severity is taken
 from the status type field (SYSSTATUS_TYPE_*); no separate level field is
 defined.

 The status word supplies the type, module id and code fields. The optional
 detail string supplies additional runtime values and is formatted by the
 caller.

 Output line format, one text line per record :

    [timestamp] T:MMM:CCCC detail

 T    : one character type tag, R W E F (runtime, warning, error, fatal).
 MMM  : module id, hexadecimal.
 CCCC : code, hexadecimal.
 detail : optional caller string.

 The timestamp field is emitted only when a Timer is configured.

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
#ifndef __SYSLOG_H__
#define __SYSLOG_H__

#include <stdint.h>
#include <stdbool.h>

#include "sysstatus.h"
#include "device_intrf.h"
#include "coredev/timer.h"

// Max length of one formatted log line, including the detail string.
#ifndef SYSLOG_LINE_MAX
#define SYSLOG_LINE_MAX     128
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Logger instance. Holds configuration only.
 */
typedef struct __Sys_Log {
	DevIntrf_t	*pSink;		//!< Output interface. NULL disables output.
	uint32_t	SinkAddr;	//!< Device select id passed to DeviceIntrfTx, 0 for UART
	TimerDev_t	*pTimer;	//!< Timestamp tick source. NULL disables timestamp.
	uint32_t	MinType;	//!< Minimum type field emitted, e.g. SYSSTATUS_TYPE_WRN. 0 emits all.
} SysLog_t;

/**
 * Initialize a logger instance.
 *
 * @param	pLog	 : Logger instance.
 * @param	pSink	 : Output DeviceIntrf, NULL disables output.
 * @param	SinkAddr : Device select id for the output, 0 for UART.
 * @param	pTimer	 : Timer for timestamps, NULL disables timestamp.
 * @param	MinType	 : Minimum type field emitted, SYSSTATUS_TYPE_*. 0 emits all.
 */
void SysLogInit(SysLog_t * const pLog, DevIntrf_t * const pSink,
				uint32_t SinkAddr, TimerDev_t * const pTimer, uint32_t MinType);

/**
 * Format and emit one status record.
 * Returns without output when the type field is below MinType or pSink is NULL.
 *
 * @param	pLog	: Logger instance.
 * @param	Status	: Status word to emit.
 * @param	pDetail	: Detail string, NULL for none.
 *
 * @return	Byte count written to the output. 0 when filtered or no output.
 */
int SysLogStatus(SysLog_t * const pLog, SysStatus_t Status, const char *pDetail);

#ifdef __cplusplus
}

//
// C++ wrapper. Holds a SysLog_t and forwards to the C functions.
//
class SysLog {
public:
	SysLog() { SysLogInit(&vLog, (DevIntrf_t *)0, 0, (TimerDev_t *)0, 0); }

	void Init(DevIntrf_t * const pSink, uint32_t SinkAddr = 0,
			  TimerDev_t * const pTimer = (TimerDev_t *)0, uint32_t MinType = 0) {
		SysLogInit(&vLog, pSink, SinkAddr, pTimer, MinType);
	}

	int Log(SysStatus_t Status, const char *pDetail = (const char *)0) {
		return SysLogStatus(&vLog, Status, pDetail);
	}

	// Return the underlying C handle for use with the C API.
	operator SysLog_t * const () { return &vLog; }

	// Non copyable. A logger references a single output interface.
	SysLog(const SysLog &) = delete;
	SysLog & operator = (const SysLog &) = delete;

private:
	SysLog_t vLog;
};

#endif // __cplusplus

#endif // __SYSLOG_H__
