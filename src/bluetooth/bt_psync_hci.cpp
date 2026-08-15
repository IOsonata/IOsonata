/**-------------------------------------------------------------------------
@file	bt_psync_hci.cpp

@brief	Periodic advertising synchronization commands over the generic HCI

The command half of the receiving side. The events, the report reassembly and
the application hooks are in bt_psync.cpp, which needs no HCI device, so the
event dispatcher can reach them without the application state.

Core spec Vol 4 Part E 7.8.67 to 7.8.73 and 7.8.88.

Parameters are range checked before a command goes out. The controller answers
an out of range value with a single status byte that does not say which of
seven parameters was wrong, and Create Sync answers with Command Status rather
than Command Complete, so the caller has even less to work with.

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
#include "bluetooth/bt_psync.h"

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

// --- Packed HCI command parameter layouts ---

#pragma pack(push, 1)

typedef struct {
	uint8_t  Options;
	uint8_t  AdvSid;
	uint8_t  AdvAddrType;
	uint8_t  AdvAddr[6];
	uint8_t  Skip[2];
	uint8_t  SyncTimeout[2];		//!< 10 ms units
	uint8_t  SyncCteType;
} BtHciLePsyncCreate_t;				//!< 14 octets, 7.8.67

typedef struct {
	uint8_t  SyncHdl[2];
} BtHciLePsyncHdl_t;				//!< 2 octets, 7.8.69

typedef struct {
	uint8_t  AdvAddrType;
	uint8_t  AdvAddr[6];
	uint8_t  AdvSid;
} BtHciLePsyncListEntry_t;			//!< 8 octets, 7.8.70 and 7.8.71

typedef struct {
	uint8_t  SyncHdl[2];
	uint8_t  Enable;
} BtHciLePsyncRecvEnable_t;			//!< 3 octets, 7.8.88

#pragma pack(pop)

// Every Options bit the spec defines. A reserved bit set is a caller error.
#define BTPSYNC_OPT_ALL				(BTPSYNC_OPT_USE_LIST | \
									 BTPSYNC_OPT_REPORTING_DISABLED | \
									 BTPSYNC_OPT_DUPLICATE_FILTER)

static inline void BtPsyncWr16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline BtHciDevice_t *BtPsyncHciDev(void)
{
	return g_BtAppData.AppDevice.pHciDev;
}

// An advertiser is named by address type, address and SID in three commands,
// so the range rules live in one place.
static bool BtPsyncAdvNameValid(uint8_t AddrType, const uint8_t *pAddr, uint8_t Sid)
{
	return pAddr != nullptr && Sid <= BTPSYNC_SID_MAX &&
		   (AddrType == BTPSYNC_ADDR_PUBLIC || AddrType == BTPSYNC_ADDR_RANDOM);
}

bool BtPsyncCreate(const BtPsyncCfg_t * const pCfg)
{
	if (pCfg == nullptr)
	{
		return false;
	}

	if ((pCfg->Options & ~BTPSYNC_OPT_ALL) != 0)
	{
		DEBUG_PRINTF("PSYNC reserved option bit set, 0x%02x\r\n",
					 (unsigned)pCfg->Options);
		return false;
	}

	// With the Periodic Advertiser List in use the controller ignores the SID
	// and address, so they are only checked when they mean something.
	if ((pCfg->Options & BTPSYNC_OPT_USE_LIST) == 0 &&
		BtPsyncAdvNameValid(pCfg->AdvAddrType, pCfg->AdvAddr, pCfg->AdvSid) == false)
	{
		return false;
	}

	if (pCfg->Skip > BTPSYNC_SKIP_MAX ||
		pCfg->SyncTimeout < BTPSYNC_TIMEOUT_MIN ||
		pCfg->SyncTimeout > BTPSYNC_TIMEOUT_MAX)
	{
		DEBUG_PRINTF("PSYNC skip or timeout out of range\r\n");
		return false;
	}

	// Vol 4 Part E 7.8.67: every non-reserved Sync_CTE_Type bit set is
	// answered Command Disallowed, since it refuses every kind of packet.
	// A reserved bit set is a caller error the controller does not define.
	if (pCfg->SyncCteType == BTPSYNC_CTE_ALL ||
		(pCfg->SyncCteType & ~BTPSYNC_CTE_ALL) != 0)
	{
		DEBUG_PRINTF("PSYNC CTE type refuses everything, 0x%02x\r\n",
					 (unsigned)pCfg->SyncCteType);
		return false;
	}

	BtHciDevice_t *pDev = BtPsyncHciDev();
	if (pDev == nullptr)
	{
		return false;
	}

	BtHciLePsyncCreate_t p;

	p.Options = pCfg->Options;
	p.AdvSid = pCfg->AdvSid;
	p.AdvAddrType = pCfg->AdvAddrType;
	memcpy(p.AdvAddr, pCfg->AdvAddr, 6);
	BtPsyncWr16(p.Skip, pCfg->Skip);
	BtPsyncWr16(p.SyncTimeout, pCfg->SyncTimeout);
	p.SyncCteType = pCfg->SyncCteType;

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC, &p,
						sizeof(p), nullptr, 0) == 0;
}

bool BtPsyncCreateCancel(void)
{
	BtHciDevice_t *pDev = BtPsyncHciDev();

	if (pDev == nullptr)
	{
		return false;
	}

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_CREATE_SYNC_CANCEL,
						nullptr, 0, nullptr, 0) == 0;
}

bool BtPsyncTerminate(uint16_t SyncHdl)
{
	BtHciDevice_t *pDev = BtPsyncHciDev();

	if (pDev == nullptr || SyncHdl > BTPSYNC_HDL_MAX)
	{
		return false;
	}

	BtHciLePsyncHdl_t p;

	BtPsyncWr16(p.SyncHdl, SyncHdl);

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_TERMINATE_SYNC, &p,
					 sizeof(p), nullptr, 0) != 0)
	{
		return false;
	}

	// The handle is destroyed by a successful terminate, so anything held for
	// it has to go with it: a later train given the same handle must not
	// inherit a fragment of this one.
	BtPsyncReasmReset(SyncHdl);

	return true;
}

static bool BtPsyncListCmd(uint16_t OpCode, uint8_t AddrType,
						   const uint8_t Addr[6], uint8_t Sid)
{
	BtHciDevice_t *pDev = BtPsyncHciDev();

	if (pDev == nullptr || BtPsyncAdvNameValid(AddrType, Addr, Sid) == false)
	{
		return false;
	}

	BtHciLePsyncListEntry_t p;

	p.AdvAddrType = AddrType;
	memcpy(p.AdvAddr, Addr, 6);
	p.AdvSid = Sid;

	return BtHciCommand(pDev, OpCode, &p, sizeof(p), nullptr, 0) == 0;
}

bool BtPsyncListAdd(uint8_t AddrType, const uint8_t Addr[6], uint8_t Sid)
{
	return BtPsyncListCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_ADD_DEV,
						  AddrType, Addr, Sid);
}

bool BtPsyncListRemove(uint8_t AddrType, const uint8_t Addr[6], uint8_t Sid)
{
	return BtPsyncListCmd(BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_REMOVE_DEV,
						  AddrType, Addr, Sid);
}

bool BtPsyncListClear(void)
{
	BtHciDevice_t *pDev = BtPsyncHciDev();

	if (pDev == nullptr)
	{
		return false;
	}

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_CLEAR,
						nullptr, 0, nullptr, 0) == 0;
}

bool BtPsyncListSizeGet(uint8_t *pSize)
{
	BtHciDevice_t *pDev = BtPsyncHciDev();

	if (pDev == nullptr || pSize == nullptr)
	{
		return false;
	}

	uint8_t size = 0;

	if (BtHciCommand(pDev, BT_HCI_CMD_CTLR_PERIODIC_ADV_LIST_READ_SIZE,
					 nullptr, 0, &size, sizeof(size)) != 0)
	{
		return false;
	}

	*pSize = size;

	return true;
}

bool BtPsyncReceiveEnable(uint16_t SyncHdl, bool Enable)
{
	BtHciDevice_t *pDev = BtPsyncHciDev();

	if (pDev == nullptr || SyncHdl > BTPSYNC_HDL_MAX)
	{
		return false;
	}

	BtHciLePsyncRecvEnable_t p;

	BtPsyncWr16(p.SyncHdl, SyncHdl);
	p.Enable = Enable ? 1 : 0;

	return BtHciCommand(pDev, BT_HCI_CMD_CTLR_SET_PERIODIC_ADV_RECEIVE_ENABLE,
						&p, sizeof(p), nullptr, 0) == 0;
}
