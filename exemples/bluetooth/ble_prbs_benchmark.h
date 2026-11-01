/* Shared post-notification benchmark for Nordic, WBA and the ST client.
 * Copyright (c) 2026 I-SYST inc. SPDX-License-Identifier: MIT
 * Notify may run in an IRQ. Receive/Poll/Reset run in the main loop.
 * One notification producer and one main-loop consumer own the fixed CFIFO.
 */
#ifndef BLE_PRBS_BENCHMARK_H
#define BLE_PRBS_BENCHMARK_H

#include "ble_prbs_check.h"
#include "cfifo.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define PRBS_BENCH_QUEUE_COUNT 12U
#define PRBS_BENCH_PAYLOAD 244U
#define PRBS_BENCH_DURATION 60000U
#define PRBS_BENCH_REPORT_MS 1000U

typedef struct {
	uint32_t ReceivedMs;
	uint16_t Len;
	uint8_t Data[PRBS_BENCH_PAYLOAD];
} PrbsNotification_t;

typedef struct {
	uint32_t (*Clock)(void);
	bool (*Schedule)(void);
	bool (*Write)(const uint8_t *pData, uint16_t Len);
	void (*Done)(void);
} PrbsBenchAdapters_t;

typedef struct {
	PrbsRxStats_t Stats;
	hCFifo_t Queue;
	uint32_t Memory[(CFIFO_TOTAL_MEMSIZE(PRBS_BENCH_QUEUE_COUNT,
		sizeof(PrbsNotification_t)) + 3U) / 4U];
	const PrbsBenchAdapters_t *Adapters;
	volatile uint32_t Dropped;
	volatile bool Failed;
	volatile bool Finished;
	volatile bool ResetPending;
	bool FinalPrinted;
	bool Complete;
	uint8_t Phy;
	uint32_t LastReportMs;
	uint32_t LastReportBytes;
} PrbsBenchmark_t;

static inline void PrbsBenchResetMeasurement(PrbsBenchmark_t *pBench)
{
	pBench->ResetPending = false;
	memset(&pBench->Stats, 0, sizeof(pBench->Stats));
	pBench->Stats.MinLen = UINT16_MAX;
	pBench->LastReportMs = 0;
	pBench->LastReportBytes = 0;
	pBench->FinalPrinted = false;
	pBench->Complete = false;
}

static inline void PrbsBenchReset(PrbsBenchmark_t *pBench, uint8_t Phy)
{
	CFifoFlush(pBench->Queue);
	PrbsBenchResetMeasurement(pBench);
	pBench->Phy = Phy;
	pBench->Dropped = 0;
	pBench->Failed = false;
	pBench->Finished = false;
}

static inline bool PrbsBenchInit(PrbsBenchmark_t *pBench,
	const PrbsBenchAdapters_t *pAdapters, uint8_t Phy)
{
	pBench->Adapters = pAdapters;
	pBench->Queue = CFifoInit((uint8_t *)pBench->Memory, sizeof(pBench->Memory),
		sizeof(PrbsNotification_t), true);
	if (pBench->Queue == NULL)
	{
		return false;
	}
	PrbsBenchReset(pBench, Phy);
	return true;
}

static inline void PrbsBenchPrint(PrbsBenchmark_t *pBench, const char *pFormat, ...)
{
	char line[160];
	va_list args;
	int len;

	va_start(args, pFormat);
	len = vsnprintf(line, sizeof(line), pFormat, args);
	va_end(args);
	if (len < 0 || (size_t)len >= sizeof(line) ||
		!pBench->Adapters->Write((const uint8_t *)line, (uint16_t)len))
	{
		pBench->Failed = true;
	}
}

static inline void PrbsBenchReport(PrbsBenchmark_t *pBench, uint32_t Now)
{
	const PrbsRxStats_t *stats;
	PrbsReportValues_t values;
	bool final;

	stats = &pBench->Stats;
	if (pBench->FinalPrinted)
	{
		return;
	}
	final = pBench->Finished || pBench->Failed || (stats->Started &&
		(uint32_t)(Now - stats->FirstMs) >= PRBS_BENCH_DURATION);
	if (!final && (!stats->Started ||
		(uint32_t)(Now - pBench->LastReportMs) < PRBS_BENCH_REPORT_MS))
	{
		return;
	}
	pBench->Finished = final;
	PrbsReportValues(stats, Now, pBench->LastReportMs, pBench->LastReportBytes,
		PRBS_BENCH_DURATION, pBench->Phy, pBench->Failed, pBench->Dropped, &values);
	PrbsBenchPrint(pBench, "%sPHY=%u Bytes/sec=%lu average=%lu elapsed_ms=%lu\r\n",
		final ? "Final " : "", pBench->Phy, (unsigned long)values.Rate,
		(unsigned long)values.Average, (unsigned long)values.Elapsed);
	PrbsBenchPrint(pBench, "bytes=%lu notifications=%lu drop=%lu out_of_frame=%lu\r\n",
		(unsigned long)stats->Bytes, (unsigned long)stats->Notifications,
		(unsigned long)stats->Breaks, (unsigned long)stats->OutOfFrame);
	PrbsBenchPrint(pBench, "len_min=%u len_max=%u notifications/15ms=%lu.%03lu\r\n",
		stats->Notifications ? stats->MinLen : 0, stats->MaxLen,
		(unsigned long)(values.PerInterval / 1000), (unsigned long)(values.PerInterval % 1000));
	pBench->LastReportMs = values.Time;
	pBench->LastReportBytes = stats->Bytes;
	if (final)
	{
		pBench->FinalPrinted = true;
		pBench->Complete = values.Complete && !pBench->Failed;
		PrbsBenchPrint(pBench, "Notification queue dropped=%lu\r\n", (unsigned long)pBench->Dropped);
		PrbsBenchPrint(pBench, "PRBS integrity: %s\r\n", values.Valid && !pBench->Failed ? "PASS" : "FAIL");
		PrbsBenchPrint(pBench, "Notification target=%lu: %s (average only)\r\n",
			(unsigned long)values.Target, values.PerInterval >= values.Target * 1000 ? "MET" : "NOT MET");
		PrbsBenchPrint(pBench, "PHY=%u RESULT: %s\r\n", pBench->Phy,
			values.Passed && !pBench->Failed ? "PASS" : "FAIL");
		PrbsBenchPrint(pBench, "PHY=%u %s\r\n", pBench->Phy,
			pBench->Complete ? "trial complete." : "trial aborted; benchmark stopped.");
		if (pBench->Failed)
		{
			pBench->Complete = false;
		}
		pBench->Adapters->Done();
	}
}

static inline void PrbsBenchPoll(PrbsBenchmark_t *pBench, uint32_t Now)
{
	if (pBench->ResetPending)
	{
		PrbsBenchResetMeasurement(pBench);
	}
	// Report only after all copied packets have been checked, including
	// packets that arrived before the deadline but were dispatched later.
	if (CFifoUsed(pBench->Queue) != 0)
	{
		(void)pBench->Adapters->Schedule();
		return;
	}
	PrbsBenchReport(pBench, Now);
}

static inline void PrbsBenchCheck(PrbsBenchmark_t *pBench,
	const uint8_t *pData, uint16_t Len, uint32_t Now)
{
	bool started;

	if (pBench->ResetPending)
	{
		PrbsBenchResetMeasurement(pBench);
	}
	if (pBench->Finished || Len == 0 || pData == NULL)
	{
		return;
	}
	started = pBench->Stats.Started;
	if (!PrbsCheckPacket(&pBench->Stats, pData, Len, Now, PRBS_BENCH_DURATION))
	{
		pBench->Finished = true;
	}
	if (!started && pBench->Stats.Started)
	{
		pBench->LastReportMs = pBench->Stats.FirstMs;
	}
}

static inline void PrbsBenchReceive(PrbsBenchmark_t *pBench)
{
	PrbsNotification_t *packet;

	packet = (PrbsNotification_t *)CFifoPeek(pBench->Queue);
	if (packet != NULL)
	{
		PrbsBenchCheck(pBench, packet->Data, packet->Len, packet->ReceivedMs);
		CFifoRelease(pBench->Queue, 1);
	}
	PrbsBenchPoll(pBench, pBench->Adapters->Clock());
}

static inline void PrbsBenchNotify(PrbsBenchmark_t *pBench,
	const uint8_t *pData, uint16_t Len)
{
	PrbsNotification_t *packet;
	uint32_t now;

	if (pBench->Finished)
	{
		return;
	}
	now = pBench->Adapters->Clock();
	packet = Len > 0 && Len <= PRBS_BENCH_PAYLOAD && pData != NULL ?
		(PrbsNotification_t *)CFifoResv(pBench->Queue) : NULL;
	if (packet == NULL)
	{
		pBench->Dropped++;
		pBench->Failed = true;
		(void)pBench->Adapters->Schedule();
		return;
	}
	packet->ReceivedMs = now;
	packet->Len = Len;
	memcpy(packet->Data, pData, Len);
	// Publish only after the payload is complete. The callback never checks
	// PRBS, formats output, or retains the stack's payload pointer.
	(void)CFifoPut(pBench->Queue);
	(void)pBench->Adapters->Schedule();
}

#endif
