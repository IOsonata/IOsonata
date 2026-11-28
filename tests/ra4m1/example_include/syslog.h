/* REDUCED EXAMPLE TEST DOUBLE ONLY. No production project uses this header.
 * The SysLog calls of the timer demo. example_model.cpp counts the records.
 * Copyright (c) 2026 I-SYST inc. MIT License.
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../../../include/sysstatusdef.h"

#define SYSLOG_RECORD_META_SIZE		4U
#define SYSLOG_MEMSIZE(NbRecords, RecordLen) \
	(64U + ((NbRecords) + 2U) * ((RecordLen) + SYSLOG_RECORD_META_SIZE))

typedef struct {
	uint8_t *pMem;
	uint32_t MemSize;
	uint32_t RecordLen;
	bool bBlocking;
} SysLogCfg_t;

typedef struct __SysLog SysLog_t;

class DeviceIntrf {
};

class SysLog {
public:
	bool Init(const SysLogCfg_t &Cfg, DeviceIntrf *pOut);
};

struct DevIntrf_t;

extern "C" SysLog_t *SysLogGet(void);
extern "C" bool SysLogInitNoTimer(SysLog_t * const pLog, const SysLogCfg_t * const pCfg,
	DevIntrf_t * const pSink, uint32_t SinkAddr, uint32_t MinType);
SysLog *SysLogGetInstance(void);
int SysLogStatus(SysLog_t * const pLog, SysStatus_t Status, const char *pDetail);
