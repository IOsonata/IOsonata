/* Shared firmware PRBS checker for IOsonata and the ST comparison client.
 * State is owned by the main loop. No stack calls, clock reads or allocation.
 * Copyright (c) 2026 I-SYST inc. SPDX-License-Identifier: MIT
 */
#ifndef BLE_PRBS_CHECK_H
#define BLE_PRBS_CHECK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint32_t Bytes;
	uint32_t Notifications;
	uint32_t Breaks;
	uint32_t OutOfFrame;
	uint32_t FirstMs;
	uint16_t MinLen;
	uint16_t MaxLen;
	uint8_t Previous;
	uint8_t ValidRun;
	bool Started;
	bool Synced;
} PrbsRxStats_t;

static inline bool PrbsCheckPacket(PrbsRxStats_t *pStats,
	const uint8_t *pData, uint16_t Len, uint32_t Now, uint32_t Duration)
{
	uint16_t i;
	uint8_t value;
	uint8_t expected;
	bool valid;

	if (Len == 0 || pData == NULL)
	{
		return false;
	}
	if (pStats->Started && (uint32_t)(Now - pStats->FirstMs) >= Duration)
	{
		return false;
	}
	if (!pStats->Started)
	{
		pStats->FirstMs = Now;
	}
	for (i = 0; i < Len; i++)
	{
		value = pData[i];
		expected = ((pStats->Previous << 1) |
			(((pStats->Previous >> 6) ^ (pStats->Previous >> 5)) & 1)) & 0x7f;
		if (pStats->Started && value != expected)
		{
			pStats->Breaks++;
		}
		if (value > 0x7f && value != 0xff)
		{
			pStats->OutOfFrame++;
		}
		if (!pStats->Synced)
		{
			valid = (value > 0 && value <= 0x7f) || value == 0xff;
			if (!valid)
			{
				pStats->ValidRun = 0;
			}
			else if (!pStats->Started || value != expected)
			{
				pStats->ValidRun = 1;
			}
			else if (pStats->ValidRun < 8)
			{
				pStats->ValidRun++;
			}
			if (pStats->ValidRun >= 8)
			{
				pStats->Synced = true;
			}
		}
		pStats->Previous = value;
		pStats->Started = true;
	}
	pStats->Bytes += Len;
	pStats->Notifications++;
	if (Len < pStats->MinLen)
	{
		pStats->MinLen = Len;
	}
	if (Len > pStats->MaxLen)
	{
		pStats->MaxLen = Len;
	}
	return true;
}

typedef struct {
	uint32_t Elapsed;
	uint32_t Time;
	uint32_t Rate;
	uint32_t Average;
	uint32_t PerInterval;
	uint32_t Target;
	bool Complete;
	bool Valid;
	bool Passed;
} PrbsReportValues_t;

static inline void PrbsReportValues(const PrbsRxStats_t *pStats,
	uint32_t Now, uint32_t LastTime, uint32_t LastBytes,
	uint32_t Duration, uint8_t Phy, bool Failed, uint32_t Dropped,
	PrbsReportValues_t *pValues)
{
	uint32_t interval;

	pValues->Elapsed = pStats->Started ? (uint32_t)(Now - pStats->FirstMs) : 0;
	if (pValues->Elapsed > Duration)
	{
		pValues->Elapsed = Duration;
	}
	pValues->Time = pStats->FirstMs + pValues->Elapsed;
	interval = pStats->Started ? (uint32_t)(pValues->Time - LastTime) : 0;
	pValues->Rate = interval ? (uint32_t)((uint64_t)(pStats->Bytes - LastBytes) * 1000 / interval) : 0;
	pValues->Average = pValues->Elapsed ? (uint32_t)((uint64_t)pStats->Bytes * 1000 / pValues->Elapsed) : 0;
	pValues->PerInterval = pValues->Elapsed ? (uint32_t)((uint64_t)pStats->Notifications * 15000 / pValues->Elapsed) : 0;
	pValues->Target = Phy == 1 ? 4 : 8;
	pValues->Complete = pStats->Started && pValues->Elapsed == Duration && !Failed;
	pValues->Valid = pValues->Complete && pStats->Synced && pStats->Breaks == 0 &&
		pStats->OutOfFrame == 0 && Dropped == 0;
	pValues->Passed = pValues->Valid && pValues->PerInterval >= pValues->Target * 1000;
}

#endif
