/**-------------------------------------------------------------------------
@file	bt_padv_hci.cpp

@brief	Bluetooth LE periodic advertising over the generic HCI transport

Advertiser side of periodic advertising. The extended advertising set is
configured by bt_adv_hci.cpp; this file adds the periodic train to it with the
three commands of Core spec Vol 4 Part E sections 7.8.61 to 7.8.63.

Almost no state is kept here. Whether an advertising set exists and whether it
was configured for periodic advertising are questions the controller answers
with Unknown Advertising Identifier and Command Disallowed, and it knows the
truth where a copy here would go stale as soon as a set is reconfigured or
removed through the advertising path. The one thing tracked is the enable
state, because this layer is what changes it and the data path needs it to
know which operations the controller will accept.

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

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_padv.h"

/******** For DEBUG Trace ************/
// Define DEBUG_ENABLE to turn on trace for this file. Output goes to the
// SysLog transport the app configured. A release build defines NDEBUG, which
// strips the trace regardless.
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

// --- Packed HCI command parameter layouts, Vol 4 Part E 7.8.61 to 7.8.63 ---

#pragma pack(push, 1)

typedef struct {
	uint8_t  AdvHdl;
	uint8_t  IntervalMin[2];		//!< 1.25 ms units
	uint8_t  IntervalMax[2];		//!< 1.25 ms units
	uint8_t  Properties[2];
} BtHciLePadvParams_t;				//!< 7 octets, 7.8.61 [v1]

// 7.8.61 [v2] is a separate opcode, not a version parameter, and it returns
// the advertising handle alongside the status.
typedef struct {
	uint8_t  AdvHdl;
	uint8_t  IntervalMin[2];		//!< 1.25 ms units
	uint8_t  IntervalMax[2];		//!< 1.25 ms units
	uint8_t  Properties[2];
	uint8_t  NbSubevents;
	uint8_t  SubeventInterval;		//!< 1.25 ms units
	uint8_t  RspSlotDelay;			//!< 1.25 ms units
	uint8_t  RspSlotSpacing;		//!< 0.125 ms units
	uint8_t  NbRspSlots;
} BtHciLePadvParamsV2_t;			//!< 12 octets, 7.8.61 [v2]

typedef struct {
	uint8_t  AdvHdl;
	uint8_t  Operation;
	uint8_t  DataLen;
	uint8_t  Data[BTPADV_DATA_FRAG_MAX];	//!< Only DataLen octets are sent
} BtHciLePadvData_t;

typedef struct {
	uint8_t  Enable;
	uint8_t  AdvHdl;
} BtHciLePadvEnable_t;				//!< 2 octets

#pragma pack(pop)

// Octets before the variable data of the set-data command.
#define BTPADV_DATA_HDR_LEN			(sizeof(BtHciLePadvData_t) - BTPADV_DATA_FRAG_MAX)

// Operation values of LE Set Periodic Advertising Data, Vol 4 Part E 7.8.62.
#define BTPADV_OP_INTERMEDIATE		0x00
#define BTPADV_OP_FIRST				0x01
#define BTPADV_OP_LAST				0x02
#define BTPADV_OP_COMPLETE			0x03
#define BTPADV_OP_UNCHANGED			0x04

// Enable bit 0 of LE Set Periodic Advertising Enable, Vol 4 Part E 7.8.63.
// Bit 1 asks for the ADI field and needs the Periodic Advertising ADI Support
// feature, which a controller without it answers with an error, so it is not
// set here.
#define BTPADV_ENABLE_ON			0x01

// Which advertising set this layer last enabled a train on, and whether it is
// still enabled as far as the commands issued here go.
//
// Nothing else about the controller's advertising sets is mirrored. Whether a
// set exists, and whether it was configured for periodic advertising, is the
// controller's to answer: it has Unknown Advertising Identifier and Command
// Disallowed for those and knows the truth, while a copy here would go stale
// the moment a set is reconfigured or removed through the advertising path.
// The enable state is different, because this layer is the only thing that
// changes it, and the data path needs it to know which operations are legal.
static uint8_t s_PadvHdl;
static bool s_PadvEnabled;

// Subevents the train was last configured with. Kept because the Subevent
// Data Request event names its subevents modulo this count, so the wrap
// cannot be resolved from the event alone.
static uint8_t s_PadvNbSubevents;

static inline void BtPadvWr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline BtHciDevice_t *BtPadvHciDev(void)
{
	return g_BtAppData.AppDevice.pHciDev;
}

// The device to reach the controller through, for a handle inside the range
// Vol 4 Part E 7.8.61 gives. The range check is local because it is a property
// of the parameter, not of the controller's state.
static BtHciDevice_t *BtPadvDevForHdl(uint8_t AdvHdl)
{
	if (AdvHdl > BTPADV_ADV_HDL_MAX)
	{
		return nullptr;
	}

	return BtPadvHciDev();
}

// Whether the train on this handle is enabled as far as this layer knows.
static bool BtPadvEnabledOn(uint8_t AdvHdl)
{
	return s_PadvEnabled && AdvHdl == s_PadvHdl;
}

bool BtPadvInit(const BtPadvCfg_t * const pCfg)
{
	if (pCfg == nullptr || pCfg->AdvHdl > BTPADV_ADV_HDL_MAX)
	{
		return false;
	}

	// Vol 4 Part E 7.8.61 gives both intervals a range of 0x0006 to 0xFFFF and
	// requires min to be at most max. A controller answers a violation with
	// Unsupported Feature or Parameter Value, which the caller would then have
	// to map back to whichever of the two it got wrong.
	if (pCfg->IntervalMin < BTPADV_INTERVAL_MIN ||
		pCfg->IntervalMax < BTPADV_INTERVAL_MIN ||
		pCfg->IntervalMin > pCfg->IntervalMax)
	{
		DEBUG_PRINTF("PADV interval range refused min=%u max=%u\r\n",
					 (unsigned)pCfg->IntervalMin, (unsigned)pCfg->IntervalMax);
		return false;
	}

	// Periodic Advertising with Responses. Vol 4 Part E 7.8.61 states the
	// relations between the four PAwR parameters; the controller answers a
	// violation with one status byte covering all of them, so they are
	// checked here where each can be named. Zero subevents is a plain train
	// and the rest are ignored, which is what the [v1] command is for.
	if (pCfg->NbSubevents > 0)
	{
		if (pCfg->NbSubevents > BTPADV_SUBEVENT_COUNT_MAX)
		{
			DEBUG_PRINTF("PADV subevent count %u out of range\r\n",
						 (unsigned)pCfg->NbSubevents);
			return false;
		}

		// Subevent_Interval shall be at most Interval_Min / Num_Subevents.
		// Both are 1.25 ms units, so the comparison needs no conversion.
		if (pCfg->SubeventInterval == 0 ||
			(uint32_t)pCfg->SubeventInterval * pCfg->NbSubevents >
				(uint32_t)pCfg->IntervalMin)
		{
			DEBUG_PRINTF("PADV subevent interval does not fit the train\r\n");
			return false;
		}

		// Response_Slot_Delay shall be less than Subevent_Interval. A train
		// with no response slots ignores the delay and the spacing.
		if (pCfg->NbRspSlots > 0 &&
			(pCfg->RspSlotDelay == 0 ||
			 pCfg->RspSlotDelay >= pCfg->SubeventInterval))
		{
			DEBUG_PRINTF("PADV response slot delay does not fit the subevent\r\n");
			return false;
		}

		// Response_Slot_Spacing shall be at most
		// 10 x (Subevent_Interval - Response_Slot_Delay) / Num_Response_Slots,
		// and is ignored when there is a single slot. The 10 is the ratio of
		// the two units: the interval and the delay are 1.25 ms, the spacing
		// is 0.125 ms.
		if (pCfg->NbRspSlots > 1)
		{
			uint32_t span = (uint32_t)(pCfg->SubeventInterval -
									   pCfg->RspSlotDelay) * 10U;
			if (pCfg->RspSlotSpacing == 0 ||
				(uint32_t)pCfg->RspSlotSpacing * pCfg->NbRspSlots > span)
			{
				DEBUG_PRINTF("PADV response slot spacing does not fit\r\n");
				return false;
			}
		}
	}

	BtHciDevice_t *pDev = BtPadvHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	int rc;

	if (pCfg->NbSubevents > 0)
	{
		BtHciLePadvParamsV2_t p;
		p.AdvHdl = pCfg->AdvHdl;
		BtPadvWr16(p.IntervalMin, pCfg->IntervalMin);
		BtPadvWr16(p.IntervalMax, pCfg->IntervalMax);
		BtPadvWr16(p.Properties, pCfg->Properties);
		p.NbSubevents = pCfg->NbSubevents;
		p.SubeventInterval = pCfg->SubeventInterval;
		p.RspSlotDelay = pCfg->RspSlotDelay;
		p.RspSlotSpacing = pCfg->RspSlotSpacing;
		p.NbRspSlots = pCfg->NbRspSlots;

		// The returned advertising handle is the one that was asked for, so
		// there is nothing to read back.
		rc = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM_V2, &p,
						  sizeof(p), nullptr, 0);
	}
	else
	{
		BtHciLePadvParams_t p;
		p.AdvHdl = pCfg->AdvHdl;
		BtPadvWr16(p.IntervalMin, pCfg->IntervalMin);
		BtPadvWr16(p.IntervalMax, pCfg->IntervalMax);
		BtPadvWr16(p.Properties, pCfg->Properties);

		rc = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM, &p,
						  sizeof(p), nullptr, 0);
	}

	if (rc != 0)
	{
		// The status separates a parameter the controller rejected from a set it
		// has no room for. A controller given no periodic advertising resources
		// refuses this command whatever the parameters are, and without the
		// status that reads the same as a malformed train.
		DEBUG_PRINTF("PADV param refused hdl=%u subevents=%u status 0x%02x\r\n",
					 (unsigned)pCfg->AdvHdl, (unsigned)pCfg->NbSubevents,
					 (unsigned)rc);
		return false;
	}

	s_PadvNbSubevents = pCfg->NbSubevents;

	// Vol 4 Part E 7.8.61: the controller refuses this command with Command
	// Disallowed while periodic advertising is enabled for the set. It was
	// accepted, so the train on this handle is not enabled.
	if (s_PadvHdl == pCfg->AdvHdl)
	{
		s_PadvEnabled = false;
	}

	return true;
}

// One LE Set Periodic Advertising Data. Len may be zero, which the complete
// operation uses to clear the data.
static bool BtPadvDataCmd(BtHciDevice_t *pDev, uint8_t AdvHdl, uint8_t Operation,
						  const uint8_t *pData, uint8_t Len)
{
	BtHciLePadvData_t d;

	d.AdvHdl = AdvHdl;
	d.Operation = Operation;
	d.DataLen = Len;
	if (Len > 0)
	{
		memcpy(d.Data, pData, Len);
	}

	uint8_t rc = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA, &d,
							  (uint8_t)(BTPADV_DATA_HDR_LEN + Len), nullptr, 0);

	if (rc != 0)
	{
		// The operation is named because a fragmented write refused partway
		// leaves the controller holding a partial set, and which fragment it
		// stopped on is what says whether the retry starts from the first.
		DEBUG_PRINTF("PADV data refused hdl=%u op=%u len=%u status 0x%02x\r\n",
					 (unsigned)AdvHdl, (unsigned)Operation, (unsigned)Len,
					 (unsigned)rc);
		return false;
	}

	return true;
}

bool BtPadvDataSet(uint8_t AdvHdl, const uint8_t *pData, size_t Len)
{
	BtHciDevice_t *pDev = BtPadvDevForHdl(AdvHdl);
	if (pDev == nullptr || (Len > 0 && pData == nullptr))
	{
		return false;
	}

	if (Len <= BTPADV_DATA_FRAG_MAX)
	{
		return BtPadvDataCmd(pDev, AdvHdl, BTPADV_OP_COMPLETE, pData,
							 (uint8_t)Len);
	}

	// Vol 4 Part E 7.8.62: with periodic advertising enabled the controller
	// answers any operation other than complete or unchanged with Command
	// Disallowed. Sending the first fragment anyway would have the controller
	// discard the data it holds and then refuse the rest, leaving the train
	// with nothing to advertise.
	if (BtPadvEnabledOn(AdvHdl))
	{
		DEBUG_PRINTF("PADV fragmented data refused while enabled, len=%u\r\n",
					 (unsigned)Len);
		return false;
	}

	size_t off = 0;

	while (off < Len)
	{
		size_t chunk = Len - off;
		if (chunk > BTPADV_DATA_FRAG_MAX)
		{
			chunk = BTPADV_DATA_FRAG_MAX;
		}

		uint8_t op = off == 0 ? BTPADV_OP_FIRST :
					 (off + chunk >= Len ? BTPADV_OP_LAST : BTPADV_OP_INTERMEDIATE);

		if (BtPadvDataCmd(pDev, AdvHdl, op, pData + off, (uint8_t)chunk) == false)
		{
			// The controller holds a partial set now. It refuses an enable
			// while that is true (7.8.63), so clearing it here is what leaves
			// the train in a state the caller can retry from.
			(void)BtPadvDataCmd(pDev, AdvHdl, BTPADV_OP_COMPLETE, nullptr, 0);
			return false;
		}

		off += chunk;
	}

	return true;
}

bool BtPadvDataRefresh(uint8_t AdvHdl)
{
	BtHciDevice_t *pDev = BtPadvDevForHdl(AdvHdl);
	if (pDev == nullptr)
	{
		return false;
	}

	// Vol 4 Part E 7.8.62 answers the unchanged operation with Invalid HCI
	// Command Parameters when the train is disabled. The command would be
	// refused anyway; refusing here says which precondition failed.
	if (BtPadvEnabledOn(AdvHdl) == false)
	{
		return false;
	}

	return BtPadvDataCmd(pDev, AdvHdl, BTPADV_OP_UNCHANGED, nullptr, 0);
}

bool BtPadvStart(uint8_t AdvHdl)
{
	BtHciDevice_t *pDev = BtPadvDevForHdl(AdvHdl);
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLePadvEnable_t x;
	x.Enable = BTPADV_ENABLE_ON;
	x.AdvHdl = AdvHdl;

	uint8_t rc = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE, &x,
							  sizeof(x), nullptr, 0);

	if (rc != 0)
	{
		DEBUG_PRINTF("PADV start refused hdl=%u status 0x%02x\r\n",
					 (unsigned)AdvHdl, (unsigned)rc);
		return false;
	}

	s_PadvHdl = AdvHdl;
	s_PadvEnabled = true;

	return true;
}

bool BtPadvStop(uint8_t AdvHdl)
{
	BtHciDevice_t *pDev = BtPadvDevForHdl(AdvHdl);
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLePadvEnable_t x;
	x.Enable = 0;
	x.AdvHdl = AdvHdl;

	uint8_t rc = BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE, &x,
							  sizeof(x), nullptr, 0);

	if (rc != 0)
	{
		// The train is still running. Recording it as stopped would make a
		// later start look like a no-op while the train stays on air.
		DEBUG_PRINTF("PADV stop refused hdl=%u status 0x%02x\r\n",
					 (unsigned)AdvHdl, (unsigned)rc);
		return false;
	}

	if (AdvHdl == s_PadvHdl)
	{
		s_PadvEnabled = false;
	}

	return true;
}

bool BtPadvIsEnabled(uint8_t AdvHdl)
{
	return BtPadvEnabledOn(AdvHdl);
}

uint8_t BtPadvNbSubevents(void)
{
	return s_PadvNbSubevents;
}

bool BtPadvSubeventDataSet(uint8_t AdvHdl,
						   const BtPadvSubeventData_t * const pSubevents,
						   uint8_t NbSubevents)
{
	if (AdvHdl > BTPADV_ADV_HDL_MAX || pSubevents == nullptr ||
		NbSubevents == 0 || NbSubevents > BTPADV_SUBEVENT_DATA_MAX)
	{
		return false;
	}

	// Vol 4 Part E 5.4.1: where a command has several arrayed parameters they
	// interleave one iteration at a time, so each subevent is a record of
	// four octets and its data rather than five parallel blocks. Build it in
	// one pass and bound it against the command parameter length, which is a
	// single octet on the wire.
	// The Parameter_Total_Length of an HCI command packet is one octet, so 255
	// is the whole of what one command can hold whatever the controller's
	// buffers are.
	uint8_t buf[255];
	size_t n = 0;

	buf[n++] = AdvHdl;
	buf[n++] = NbSubevents;

	for (uint8_t i = 0; i < NbSubevents; i++)
	{
		const BtPadvSubeventData_t *s = &pSubevents[i];

		if (s->Subevent > BTPADV_SUBEVENT_MAX ||
			s->DataLen > BTPADV_SUBEVENT_DATA_LEN_MAX ||
			(s->DataLen > 0 && s->pData == nullptr))
		{
			return false;
		}

		if (n + 4 + s->DataLen > sizeof(buf))
		{
			// More than one command can hold. Splitting it here would send
			// some subevents and drop others, and the controller discards the
			// whole set of a command it refuses, so neither half would be
			// what the caller asked for.
			DEBUG_PRINTF("PADV subevent data too long for one command\r\n");
			return false;
		}

		buf[n++] = s->Subevent;
		buf[n++] = s->RspSlotStart;
		buf[n++] = s->RspSlotCount;
		buf[n++] = s->DataLen;
		if (s->DataLen > 0)
		{
			memcpy(&buf[n], s->pData, s->DataLen);
			n += s->DataLen;
		}
	}

	BtHciDevice_t *pDev = BtPadvHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	uint8_t rc = BtHciCommand(pDev,
							  BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_SUBEVENT_DATA,
							  buf, (uint8_t)n, nullptr, 0);

	if (rc != 0)
	{
		// The controller asks for subevent data every periodic advertising
		// event, so this refusal repeats. It is traced once per call rather
		// than suppressed, because the caller decides how often to answer.
		DEBUG_PRINTF("PADV subevent data refused hdl=%u count=%u status 0x%02x\r\n",
					 (unsigned)AdvHdl, (unsigned)NbSubevents, (unsigned)rc);
		return false;
	}

	return true;
}
