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
} BtHciLePadvParams_t;				//!< 7 octets

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

	BtHciDevice_t *pDev = BtPadvHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLePadvParams_t p;
	p.AdvHdl = pCfg->AdvHdl;
	BtPadvWr16(p.IntervalMin, pCfg->IntervalMin);
	BtPadvWr16(p.IntervalMax, pCfg->IntervalMax);
	BtPadvWr16(p.Properties, pCfg->Properties);

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_PARAM, &p,
					 sizeof(p), nullptr, 0) != 0)
	{
		return false;
	}

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

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_DATA, &d,
						(uint8_t)(BTPADV_DATA_HDR_LEN + Len), nullptr, 0) == 0;
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

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE, &x,
					 sizeof(x), nullptr, 0) != 0)
	{
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

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_ENABLE, &x,
					 sizeof(x), nullptr, 0) != 0)
	{
		// The train is still running. Recording it as stopped would make a
		// later start look like a no-op while the train stays on air.
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
