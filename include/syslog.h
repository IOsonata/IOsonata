/**-------------------------------------------------------------------------
@file	syslog.h

@brief	System logger.

 Emits SysStatus_t records to a DeviceIntrf output. The output is any
 DeviceIntrf implementation (UART, USB CDC, SPI, BLE, SLIP). The optional
 timestamp source is any Timer implementation. The log severity is taken
 from the status type field (SYSSTATUS_TYPE_*); no separate level field is
 defined.

 A CFifo built from caller supplied memory stores complete formatted log
 records. The store and the DeviceIntrf are independent: records are kept
 whether or not an output transport exists, so a logger works with a transport
 that comes and goes, or with none at all.

 When a transport is attached, a record is sent as soon as it is stored, so
 nothing has to be pumped. SysLogFlush is there for the case a sink refuses a
 record and no further record follows to send it, and for sending what
 accumulated while no transport was attached.

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

 Usage :

    #include "syslog.h"

    // 16 records of 64 bytes. Shorten the record to save memory, lengthen it
    // for longer trace lines.
    alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 64)];

    static const SysLogCfg_t s_LogCfg = {
        .pMem      = s_LogMem,
        .MemSize   = sizeof(s_LogMem),
        .RecordLen = 64,
        .bBlocking = true       // keep the oldest record when the store fills
    };

    // Store only, no transport yet.
    SysLogInit(SysLogGet(), &s_LogCfg, NULL, 0, NULL, 0);

    // Records are kept whether or not anything is listening.
    SysLogPrintf(SysLogGet(), "boot %u\r\n", reason);
    SysLogStatus(SysLogGet(),
                 StatusEncode(SYSSTATUS_TYPE_ERR, SYSSTATUS_MODID_BLE, 0x21),
                 "conn fail");

    // When a transport turns up, attach it and send what was stored while
    // there was none. Later records go out as they are logged.
    SysLogSetSink(SysLogGet(), (DevIntrf_t *)g_Uart, 0);
    SysLogFlush(SysLogGet());

    // Detach when it goes away again. Queued records stay queued.
    SysLogSetSink(SysLogGet(), NULL, 0);

 A logger with a transport from the start passes it to SysLogInit instead :

    SysLogInit(SysLogGet(), &s_LogCfg, (DevIntrf_t *)g_Uart, 0,
               (TimerDev_t *)g_Timer, SYSSTATUS_TYPE_WRN);

 In C++ the same thing reads :

    SysLogGetInstance()->Init(s_LogCfg, &g_Uart, 0, &g_Timer,
                              SYSSTATUS_TYPE_WRN);
    SysLogGetInstance()->Printf("boot %u\r\n", reason);

 For per file developer trace, the convention is :

    //#define DEBUG_ENABLE
    #if !defined(NDEBUG) && defined(DEBUG_ENABLE)
    #define DEBUG_PRINTF(...)	SysLogPrintf(SysLogGet(), __VA_ARGS__)
    #else
    #define DEBUG_PRINTF(...)	((void)sizeof(SysLogPrintf(SysLogGet(), __VA_ARGS__)))
    #endif

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
#include <stdarg.h>

#include "sysstatus.h"
#include "device_intrf.h"
#include "cfifo.h"
#include "coredev/timer.h"

// Marker written by SysLogInit. Used to detect an uninitialized instance.
// An instance without this marker is treated as dormant and all API calls
// are no-ops, so a SysLog_t embedded in a driver stays inert until
// SysLogInit is called.
#define SYSLOG_INIT_MARKER  0x474f4c53UL    // 'SLOG'

// Bytes of memory to reserve for NbRecord records of RecordLen bytes each,
// including the CFifo header. RecordLen is the log line length in bytes and
// includes the terminating NUL, so a 64 byte record holds 63 characters.
//
//    alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(16, 64)];  // 16 x 64
//
#define SYSLOG_MEMSIZE(NbRecord, RecordLen)    CFIFO_TOTAL_MEMSIZE(NbRecord, RecordLen)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Record store configuration.
 *
 * Size the memory with SYSLOG_MEMSIZE(NbRecord, RecordLen) and pass
 * sizeof() as MemSize. The record count is not repeated here because
 * MemSize and RecordLen already determine it, and MemSize is the one value
 * that describes the memory that exists. A shorter RecordLen puts more
 * records in the same memory; a longer one allows longer lines. There is no
 * upper limit, every record is formatted in place in its own block.
 */
typedef struct __Sys_Log_Cfg {
	uint8_t		*pMem;		//!< Word aligned memory for the record store
	uint32_t	MemSize;	//!< Size of pMem in bytes
	uint32_t	RecordLen;	//!< Bytes per record including the NUL. This is the
							//!< line length; a record longer than it is truncated.
	bool		bBlocking;	//!< Store full policy. true keeps the oldest record,
							//!< false evicts the oldest to keep the newest.
} SysLogCfg_t;

/**
 * Logger instance. Holds configuration only.
 */
typedef struct __Sys_Log {
	uint32_t	Marker;		//!< SYSLOG_INIT_MARKER when initialized, else dormant
	DevIntrf_t	*pSink;		//!< Output interface. NULL disables direct output.
	uint32_t	SinkAddr;	//!< Device select id passed to DeviceIntrfTx, 0 for UART
	TimerDev_t	*pTimer;	//!< Timestamp tick source. NULL disables timestamp.
	uint32_t	MinType;	//!< Minimum type field emitted, e.g. SYSSTATUS_TYPE_WRN. 0 emits all.
	hCFifo_t	hFifo;		//!< Record store, one complete record per block.
} SysLog_t;

/**
 * Initialize a logger instance.
 *
 * The record store is built here from the caller's memory, the same way a
 * UART is given pTxMem / TxMemSize and builds its own FIFO. One block holds
 * one NUL-terminated record.
 *
 * The store is required. The transport and the timestamp source are not, so
 * they stay as arguments rather than joining the store configuration. pSink
 * may be NULL and attached later with SysLogSetSink; records are stored
 * either way.
 *
 * @param	pLog	 : Logger instance.
 * @param	pCfg	 : Record store configuration.
 * @param	pSink	 : Output DeviceIntrf, NULL to attach one later.
 * @param	SinkAddr : Device select id for the output, 0 for UART.
 * @param	pTimer	 : Timer for timestamps, NULL disables timestamp.
 * @param	MinType	 : Minimum type field emitted, SYSSTATUS_TYPE_*. 0 emits all.
 *                     The value is masked with SYSSTATUS_TYPE_MASK.
 *
 * @return	true when initialized. false when pCfg is NULL, RecordLen is below
 *          2, or MemSize cannot hold one record. On failure the instance
 *          stays dormant and every call is a no-op.
 */
bool SysLogInit(SysLog_t * const pLog, const SysLogCfg_t * const pCfg,
				DevIntrf_t * const pSink, uint32_t SinkAddr,
				TimerDev_t * const pTimer, uint32_t MinType);

/**
 * Select the DeviceIntrf used for output.
 *
 * The sink may be changed at any time. Passing NULL detaches the output while
 * stored records stay queued.
 *
 * @param	pLog	 : Logger instance.
 * @param	pSink	 : Output DeviceIntrf, NULL to detach.
 * @param	SinkAddr : Device select id passed to DeviceIntrfTx.
 */
void SysLogSetSink(SysLog_t * const pLog, DevIntrf_t * const pSink,
				   uint32_t SinkAddr);

/**
 * Send queued records through the configured DeviceIntrf until the store is
 * empty or the sink stops taking them.
 *
 * Not normally needed. SysLogStatus and SysLogPrintf already do this whenever
 * a sink is attached. Call it after SysLogSetSink to send what was stored
 * while there was no transport, or when a sink refused the last record and no
 * further record is coming to send it.
 *
 * One store block is one log record. Each record is passed once to
 * DeviceIntrfTx and consumed only when the whole record is accepted. A zero,
 * partial, or negative result leaves that record queued and ends the call;
 * SysLog does not retry the remainder, so a later call offers the same
 * complete record again. Transport transfer handling remains in the concrete
 * interface implementation.
 *
 * Note this is the opposite of CFifoFlush, which discards. This one sends.
 *
 * @param	pLog : Logger instance.
 *
 * @return	Total bytes the sink accepted, 0 when there was nothing to send or
 *          the sink took nothing. A negative DeviceIntrfTx result is returned
 *          unchanged when no bytes went out before it.
 */
int SysLogFlush(SysLog_t * const pLog);

/**
 * Format and store one status record.
 * No-op when the instance is not initialized (no SYSLOG_INIT_MARKER), when the
 * store is full under the blocking policy, or when the type field is below
 * MinType.
 *
 * @param	pLog	: Logger instance.
 * @param	Status	: Status word to emit.
 * @param	pDetail	: Detail string, NULL for none.
 *
 * @return	Byte count stored, not the byte count sent. 0 when dormant,
 *          filtered, or the store is full.
 */
int SysLogStatus(SysLog_t * const pLog, SysStatus_t Status, const char *pDetail);

/**
 * Format and store free form trace text. No record prefix is added, the
 * output is the formatted text as given. For developer trace that has no
 * status meaning. Text longer than the configured RecordLen is truncated.
 * No-op when the instance is not initialized, when the store is full under the
 * blocking policy, or when pFormat is NULL. Not subject to the MinType filter,
 * which applies to status records only.
 *
 * @param	pLog	: Logger instance.
 * @param	pFormat	: printf style format string.
 *
 * @return	Byte count stored. 0 when dormant or the store is full.
 */
int SysLogPrintf(SysLog_t * const pLog, const char *pFormat, ...)
#if defined(__GNUC__) || defined(__clang__)
	__attribute__((format(printf, 2, 3)))
#endif
	;

/**
 * va_list form of SysLogPrintf.
 *
 * @param	pLog	: Logger instance.
 * @param	pFormat	: printf style format string.
 * @param	Args	: Variable argument list.
 *
 * @return	Byte count stored. 0 when dormant or the store is full.
 */
int SysLogVPrintf(SysLog_t * const pLog, const char *pFormat, va_list Args);

/**
 * Get the library global logger handle for use with the C API.
 * The global instance is dormant until SysLogInit succeeds on it, or in C++
 * until SysLogGetInstance()->Init does. Calls on a dormant instance are
 * no-ops.
 *
 * @return	Handle to the global logger.
 */
SysLog_t *SysLogGet(void);

//
// Status stack. IOsonata default storage for status provenance. A fixed
// depth LIFO of SysStatus_t values recording the chain of status producing
// sites as an operation unwinds. The application reads the most recent
// entry first and pops down the chain for more detail.
//
// Independent of the logger. Pushing a status and recording it to an output
// are separate operations.
//
// Policy :
//   - Push records a status. Pushes occur at status producing sites only.
//   - Read returns the most recent entry (LIFO).
//   - Pop returns and removes the most recent entry. Pop on empty returns
//     SYSSTATUS_OK.
//   - When full, a push is rejected, preserving the originating cause.
//   - The first push after any pop clears the stack, then stores, starting
//     a new chain once the previous one has been read.
//
// The default implementation guards stack mutation with the IOsonata
// interrupt mask helpers when they are available for the target. Hosted
// builds use no interrupt mask by default.
//

// Stack depth.
#ifndef SYSSTATUS_STACK_DEPTH
#define SYSSTATUS_STACK_DEPTH   4
#endif

/**
 * Status stack instance.
 */
typedef struct __Sys_Status_Stack {
	SysStatus_t	Entry[SYSSTATUS_STACK_DEPTH];	//!< Storage, index 0 is oldest
	int			Count;							//!< Number of stored entries
	bool		PoppedSincePush;				//!< Set by a pop, clears on next push
} SysStatusStack_t;

/**
 * Reset a stack to empty.
 *
 * @param	pStack : Stack instance.
 */
void SysStatusStackReset(SysStatusStack_t * const pStack);

/**
 * Push a status onto a stack. Clears first if a pop has occurred since the
 * last push. Rejected if the stack is full.
 *
 * @param	pStack	: Stack instance.
 * @param	Status	: Status word to store.
 *
 * @return	true if stored, false if rejected.
 */
bool SysStatusStackPush(SysStatusStack_t * const pStack, SysStatus_t Status);

/**
 * Pop the most recent status.
 *
 * @param	pStack : Stack instance.
 *
 * @return	Most recent status, or SYSSTATUS_OK if empty.
 */
SysStatus_t SysStatusStackPop(SysStatusStack_t * const pStack);

/**
 * Read the most recent status without removing it.
 *
 * @param	pStack : Stack instance.
 *
 * @return	Most recent status, or SYSSTATUS_OK if empty.
 */
SysStatus_t SysStatusStackPeek(SysStatusStack_t * const pStack);

/**
 * Number of stored entries.
 *
 * @param	pStack : Stack instance.
 *
 * @return	Entry count.
 */
int SysStatusStackCount(SysStatusStack_t * const pStack);

/**
 * Get the IOsonata global status stack handle.
 *
 * @return	Handle to the global stack.
 */
SysStatusStack_t *SysStatusStackGet(void);

/**
 * Push a status onto the global stack.
 *
 * @param	Status : Status word to store.
 *
 * @return	true if stored, false if rejected.
 */
bool SysStatusPush(SysStatus_t Status);

/**
 * Pop the most recent status from the global stack.
 *
 * @return	Most recent status, or SYSSTATUS_OK if empty.
 */
SysStatus_t SysStatusPop(void);

/**
 * Read the most recent status on the global stack without removing it.
 *
 * @return	Most recent status, or SYSSTATUS_OK if empty.
 */
SysStatus_t SysStatusPeek(void);

#ifdef __cplusplus
}

//
// C++ wrapper. Holds a SysLog_t and forwards to the C functions.
//
class SysLog {
public:
	// Construct dormant. The instance stays inert until Init is called.
	SysLog() : vLog{} {}

	bool Init(const SysLogCfg_t &Cfg,
			  DeviceIntrf *pSink = (DeviceIntrf *)0, uint32_t SinkAddr = 0,
			  Timer *pTimer = (Timer *)0, uint32_t MinType = 0) {
		return SysLogInit(&vLog, &Cfg,
						  pSink ? (DevIntrf_t *)*pSink : (DevIntrf_t *)0,
						  SinkAddr,
						  pTimer ? (TimerDev_t *)*pTimer : (TimerDev_t *)0,
						  MinType);
	}

	void SetSink(DeviceIntrf *pSink, uint32_t SinkAddr = 0) {
		SysLogSetSink(&vLog,
					  pSink ? (DevIntrf_t *)*pSink : (DevIntrf_t *)0,
					  SinkAddr);
	}

	int Flush() {
		return SysLogFlush(&vLog);
	}

	int Log(SysStatus_t Status, const char *pDetail = (const char *)0) {
		return SysLogStatus(&vLog, Status, pDetail);
	}

	int Printf(const char *pFormat, ...)
#if defined(__GNUC__) || defined(__clang__)
		__attribute__((format(printf, 2, 3)))
#endif
		;

	// Return the underlying C handle for use with the C API.
	operator SysLog_t * () { return &vLog; }

	// Non copyable. A logger references a single output interface.
	SysLog(const SysLog &) = delete;
	SysLog & operator = (const SysLog &) = delete;

private:
	SysLog_t vLog;
};

/**
 * Get the library global logger object for direct C++ configuration, e.g.
 * SysLogGetInstance()->Init(cfg, &uart, 0, &timer, SYSSTATUS_TYPE_WRN);
 *
 * @return	Const pointer to the global SysLog instance.
 */
SysLog *SysLogGetInstance(void);

#endif // __cplusplus

#endif // __SYSLOG_H__
