/**-------------------------------------------------------------------------
@file	bt_psync.cpp

@brief	Periodic advertising sync events and report reassembly

The receiving half of periodic advertising, minus the commands. Kept apart
from bt_psync_hci.cpp so the HCI event dispatcher can call in without needing
an HCI device or the application state.

A periodic advertising report may be split across several events exactly as an
extended advertising report is, so the reassembly here is the same shape as
the one in bt_hci_host.cpp, keyed by Sync_Handle rather than by address and
SID. The dropped-set tracking matters for the same reason: the terminating
fragment of an abandoned reassembly is indistinguishable from a complete
single-fragment report, so delivering it hands the application a truncated
suffix as if it were whole advertising data.

Core spec Vol 4 Part E 7.7.65.14 to 7.7.65.16.

@author	Hoang Nguyen Hoan
@date	Aug. 15, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include <string.h>

#include "bluetooth/bt_psync.h"

// How many trains can be mid-reassembly at once, and how many abandoned ones
// are remembered. Both are small because a report is completed or abandoned
// within one periodic advertising event; they are not a subscription limit.
#define BT_PSYNC_REASM_COUNT		2
#define BT_PSYNC_DROPPED_COUNT		4

// Fixed part of each event, Vol 4 Part E 7.7.65.14 to 7.7.65.16. The subevent
// code is consumed by the dispatcher, so these count from the first parameter.
#define BT_PSYNC_EVT_ESTABLISHED_V1_LEN		15
#define BT_PSYNC_EVT_ESTABLISHED_V2_LEN		19
#define BT_PSYNC_EVT_REPORT_V1_HDR_LEN		7
#define BT_PSYNC_EVT_REPORT_V2_HDR_LEN		10
#define BT_PSYNC_EVT_LOST_LEN				2

typedef struct __Bt_Psync_Reasm {
	bool     Active;						//!< Slot holds a partial report
	uint16_t SyncHdl;						//!< Train the partial report belongs to
	int8_t   TxPwr;							//!< Transmit power of the first fragment
	uint8_t  CteType;						//!< Constant Tone Extension type of the first fragment
	uint16_t Len;							//!< Bytes accumulated so far
	uint8_t  Buf[BTPSYNC_REPORT_MAX];		//!< Accumulated advertising data
} BtPsyncReasm_t;

typedef struct __Bt_Psync_Dropped {
	bool     Active;						//!< Slot names an abandoned reassembly
	uint16_t SyncHdl;						//!< Train whose reassembly was abandoned
} BtPsyncDropped_t;

static BtPsyncReasm_t s_BtPsyncReasm[BT_PSYNC_REASM_COUNT];
static BtPsyncDropped_t s_BtPsyncDropped[BT_PSYNC_DROPPED_COUNT];
static uint8_t s_BtPsyncDroppedNext = 0;

static inline uint16_t BtPsyncRd16(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static BtPsyncReasm_t *BtPsyncReasmFind(uint16_t SyncHdl)
{
	for (int i = 0; i < BT_PSYNC_REASM_COUNT; i++)
	{
		if (s_BtPsyncReasm[i].Active && s_BtPsyncReasm[i].SyncHdl == SyncHdl)
		{
			return &s_BtPsyncReasm[i];
		}
	}

	return nullptr;
}

static BtPsyncReasm_t *BtPsyncReasmAlloc(uint16_t SyncHdl, int8_t TxPwr,
										 uint8_t CteType)
{
	for (int i = 0; i < BT_PSYNC_REASM_COUNT; i++)
	{
		BtPsyncReasm_t *c = &s_BtPsyncReasm[i];

		if (c->Active == false)
		{
			c->Active = true;
			c->SyncHdl = SyncHdl;
			c->TxPwr = TxPwr;
			c->CteType = CteType;
			c->Len = 0;
			return c;
		}
	}

	return nullptr;
}

static BtPsyncDropped_t *BtPsyncDroppedFind(uint16_t SyncHdl)
{
	for (int i = 0; i < BT_PSYNC_DROPPED_COUNT; i++)
	{
		if (s_BtPsyncDropped[i].Active && s_BtPsyncDropped[i].SyncHdl == SyncHdl)
		{
			return &s_BtPsyncDropped[i];
		}
	}

	return nullptr;
}

// Oldest entry is overwritten. Losing the record of a very old abandoned
// reassembly risks delivering one stale suffix; holding the table open forever
// would be worse.
static void BtPsyncDroppedMark(uint16_t SyncHdl)
{
	if (BtPsyncDroppedFind(SyncHdl) != nullptr)
	{
		return;
	}

	BtPsyncDropped_t *d = &s_BtPsyncDropped[s_BtPsyncDroppedNext];

	s_BtPsyncDroppedNext = (uint8_t)((s_BtPsyncDroppedNext + 1) % BT_PSYNC_DROPPED_COUNT);
	d->Active = true;
	d->SyncHdl = SyncHdl;
}

void BtPsyncReasmReset(uint16_t SyncHdl)
{
	BtPsyncReasm_t *c = BtPsyncReasmFind(SyncHdl);

	if (c != nullptr)
	{
		c->Active = false;
	}

	BtPsyncDropped_t *d = BtPsyncDroppedFind(SyncHdl);

	if (d != nullptr)
	{
		d->Active = false;
	}
}

void BtPsyncEvtEstablished(const uint8_t *pData, int Len, bool bV2)
{
	int need = bV2 ? BT_PSYNC_EVT_ESTABLISHED_V2_LEN : BT_PSYNC_EVT_ESTABLISHED_V1_LEN;

	if (pData == nullptr || Len < need)
	{
		return;
	}

	BtPsyncInfo_t info;

	memset(&info, 0, sizeof(info));
	info.Status = pData[0];
	info.SyncHdl = BtPsyncRd16(&pData[1]);
	info.AdvSid = pData[3];
	info.AdvAddrType = pData[4];
	memcpy(info.AdvAddr, &pData[5], 6);
	info.AdvPhy = pData[11];
	info.Interval = BtPsyncRd16(&pData[12]);
	info.ClockAccuracy = pData[14];

	// A failed or cancelled attempt names no train, and the V2 trailing fields
	// describe subevents this layer does not use yet. Both leave the fixed
	// part above as the whole of what is reported.
	if (info.Status == 0)
	{
		// A handle can be reused after a train ends, so start it clean rather
		// than inherit a fragment left by whatever held it before.
		BtPsyncReasmReset(info.SyncHdl);
	}

	BtPsyncEstablished(&info);
}

void BtPsyncEvtReport(const uint8_t *pData, int Len, bool bV2)
{
	int hdr = bV2 ? BT_PSYNC_EVT_REPORT_V2_HDR_LEN : BT_PSYNC_EVT_REPORT_V1_HDR_LEN;

	if (pData == nullptr || Len < hdr)
	{
		return;
	}

	uint16_t syncHdl = BtPsyncRd16(&pData[0]);
	int8_t txPwr = (int8_t)pData[2];
	int8_t rssi = (int8_t)pData[3];
	uint8_t cteType = pData[4];
	// V2 inserts Periodic_Event_Counter(2) and Subevent(1) before Data_Status.
	uint8_t dataStatus = pData[hdr - 2];
	uint8_t dataLen = pData[hdr - 1];
	const uint8_t *pAd = &pData[hdr];

	// Data_Length is controller supplied. A report claiming more than the
	// event holds would otherwise be read past its end.
	if (Len - hdr < (int)dataLen)
	{
		return;
	}

	BtPsyncReasm_t *ctx = BtPsyncReasmFind(syncHdl);

	if (dataStatus == BTPSYNC_DATA_MORE)
	{
		if (ctx == nullptr && BtPsyncDroppedFind(syncHdl) == nullptr)
		{
			ctx = BtPsyncReasmAlloc(syncHdl, txPwr, cteType);
		}

		if (ctx == nullptr)
		{
			// No slot free, or this train is already abandoned. Either way the
			// report cannot be completed and its tail must not be delivered.
			BtPsyncDroppedMark(syncHdl);
			return;
		}

		if ((uint32_t)ctx->Len + dataLen > BTPSYNC_REPORT_MAX)
		{
			// Delivering what fits would be a truncation presented as whole
			// data, so drop the report instead.
			ctx->Active = false;
			BtPsyncDroppedMark(syncHdl);
			return;
		}

		memcpy(ctx->Buf + ctx->Len, pAd, dataLen);
		ctx->Len = (uint16_t)(ctx->Len + dataLen);
		return;
	}

	if (dataStatus == BTPSYNC_DATA_COMPLETE)
	{
		if (ctx != nullptr)
		{
			if ((uint32_t)ctx->Len + dataLen <= BTPSYNC_REPORT_MAX)
			{
				memcpy(ctx->Buf + ctx->Len, pAd, dataLen);
				ctx->Len = (uint16_t)(ctx->Len + dataLen);
				// Vol 4 Part E 7.7.65.15: the transmit power belongs to the
				// first packet of the advertisement and may read 0x7F on the
				// later ones, while the RSSI is the last packet received.
				BtPsyncReport(syncHdl, ctx->TxPwr, rssi, ctx->CteType,
							  ctx->Buf, ctx->Len);
			}

			ctx->Active = false;
			return;
		}

		// No context. Either a standalone complete report, or the tail of a
		// reassembly that was abandoned, which looks identical on the wire.
		BtPsyncDropped_t *d = BtPsyncDroppedFind(syncHdl);

		if (d != nullptr)
		{
			d->Active = false;
			return;
		}

		BtPsyncReport(syncHdl, txPwr, rssi, cteType, pAd, dataLen);
		return;
	}

	// Truncated, or a reserved status. No more data is coming and what arrived
	// is incomplete, so drop any partial state and deliver nothing. The
	// sequence ends here, so there is no later fragment to guard against.
	if (ctx != nullptr)
	{
		ctx->Active = false;
	}
}

void BtPsyncEvtLost(const uint8_t *pData, int Len)
{
	if (pData == nullptr || Len < BT_PSYNC_EVT_LOST_LEN)
	{
		return;
	}

	uint16_t syncHdl = BtPsyncRd16(&pData[0]);

	// The handle is destroyed by this event, so nothing may be held for it.
	BtPsyncReasmReset(syncHdl);

	BtPsyncLost(syncHdl);
}

//
// Weak application hooks. The defaults discard, so a build that does not use
// periodic advertising sync links without supplying them.
//

__attribute__((weak)) void BtPsyncEstablished(const BtPsyncInfo_t * const pInfo)
{
	(void)pInfo;
}

__attribute__((weak)) void BtPsyncReport(uint16_t SyncHdl, int8_t TxPwr,
										 int8_t Rssi, uint8_t CteType,
										 const uint8_t *pData, uint16_t Len)
{
	(void)SyncHdl;
	(void)TxPwr;
	(void)Rssi;
	(void)CteType;
	(void)pData;
	(void)Len;
}

__attribute__((weak)) void BtPsyncLost(uint16_t SyncHdl)
{
	(void)SyncHdl;
}
