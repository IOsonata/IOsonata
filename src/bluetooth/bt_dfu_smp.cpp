/**-------------------------------------------------------------------------
@file	bt_dfu_smp.cpp

@brief	SMP over Bluetooth LE: the SMP GATT service.

See bt_dfu_smp.h. The stack callbacks only copy and queue; requests and
responses run in the main loop or Bluetooth worker. Status checks recover
retained work, so a slot 1 erase never runs in a stack callback and there
is one sender.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <string.h>

#include "bluetooth/bt_app.h"
#include "coredev/interrupt.h"
#include "bluetooth/bt_dfu_smp.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_dev.h"

/** @addtogroup Bluetooth
  * @{
  */

// Bases, least significant byte first, the 16 bit part at bytes 12 and 13.
#define BT_DFUSMP_SRVC_BASE		{ 0x84, 0xAA, 0x60, 0x74, 0x52, 0x8A, 0x8B, 0x86, \
								  0xD3, 0x4C, 0xB7, 0x1D, 0x00, 0x00, 0x53, 0x8D }

static const uint8_t s_BtDfuSmpCharBase[16] = {
	0x48, 0x7C, 0x99, 0x74, 0x11, 0x26, 0x9E, 0xAE,
	0x01, 0x4E, 0xCE, 0xFB, 0x00, 0x00, 0x2E, 0xDA,
};

static void BtDfuSmpWrCB(BtGattChar_t *pChar, uint8_t *pData, int Offset,
						 int Len);
static void BtDfuSmpNotifCB(BtGattChar_t *pChar, bool bEnable,
							uint16_t ConnHdl);

static BtGattChar_t s_BtDfuSmpChar[] = {
	BT_CHAR(BT_DFUSMP_UUID_CHAR, BT_DFUSMP_CHAR_MAXLEN,
			BT_GATT_CHAR_PROP_WRITE_WORESP | BT_GATT_CHAR_PROP_NOTIFY,
			nullptr,
			.WrCB = BtDfuSmpWrCB,
			.SetNotifCB = BtDfuSmpNotifCB,
			.pUuidBase = s_BtDfuSmpCharBase),
};

static BtGattSrvc_t s_BtDfuSmpSrvc = BT_SRVC_CUSTOM(BT_DFUSMP_SRVC_BASE,
													BT_DFUSMP_UUID_SERVICE,
													s_BtDfuSmpChar);

static BtDfuSmpCfg_t s_BtDfuSmpCfg;

// Request: bytes reassembled so far, and whether a whole one is waiting.
static volatile uint32_t s_BtDfuSmpRxLen = 0;
static volatile bool s_bBtDfuSmpRxReady = false;

// Response: written by the main loop; stack callbacks only check its length.
static volatile uint32_t s_BtDfuSmpTxLen = 0;
static uint32_t s_BtDfuSmpTxOff = 0;
static uint16_t s_BtDfuSmpConnHdl = BT_CONN_HDL_INVALID;

static void BtDfuSmpProcess(uint32_t Evt, void *pCtx);
static void BtDfuSmpSend(uint32_t Evt, void *pCtx);

// The link the host listens on: the one that enabled notification.
static uint16_t BtDfuSmpConnHdl(void)
{
	for (uint16_t i = 0; i < BtPeerCount(); i++)
	{
		BtDevice_t *peer = BtPeerSlot(i);
		if (peer != nullptr && peer->Conn.Hdl != BT_CONN_HDL_INVALID &&
			BtGattCharNotifyEnabled(peer->Conn.Hdl, &s_BtDfuSmpChar[0]))
		{
			return peer->Conn.Hdl;
		}
	}

	return BT_CONN_HDL_INVALID;
}

// Stack context: copy, and once the header length is reached queue the
// request. A write that cannot belong to a request drops what was collected;
// the host times out and sends the request again.
static void BtDfuSmpWrCB(BtGattChar_t *pChar, uint8_t *pData, int Offset,
						 int Len)
{
	(void)pChar;

	if (s_BtDfuSmpCfg.pMgr == nullptr || pData == nullptr || Len <= 0 ||
		Offset != 0 || s_bBtDfuSmpRxReady)
	{
		return;
	}

	uint32_t have = s_BtDfuSmpRxLen;
	if ((uint32_t)Len > s_BtDfuSmpCfg.RxBufSize - have)
	{
		s_BtDfuSmpRxLen = 0;
		return;
	}
	memcpy(s_BtDfuSmpCfg.pRxBuf + have, pData, (size_t)Len);
	have += (uint32_t)Len;
	s_BtDfuSmpRxLen = have;

	if (have < DFUSMP_HDR_SIZE)
	{
		return;
	}

	const uint8_t *p = s_BtDfuSmpCfg.pRxBuf;
	uint32_t total = DFUSMP_HDR_SIZE + (((uint32_t)p[2] << 8) | p[3]);
	if (total > s_BtDfuSmpCfg.RxBufSize || have > total)
	{
		s_BtDfuSmpRxLen = 0;
		return;
	}
	if (have < total)
	{
		return;
	}

	s_bBtDfuSmpRxReady = true;
	// A full queue leaves the complete request for BtDfuSmpCheckStatus.
	(void)BtEvtQue(0, nullptr, BtDfuSmpProcess);
}

// Stack context: capacity was freed, possibly by another characteristic or
// connection. Wake the owning worker; never send from this callback.
void BtDfuSmpTxReady(void)
{
	if (s_BtDfuSmpTxLen != 0)
	{
		(void)BtEvtQue(0, nullptr, BtDfuSmpSend);
	}
}

// Stack context: the host changed the notification setting. A response on
// its way is looked at again from the main loop: turned off, the rest of it
// has nowhere to go; turned on, the stack may take what it refused.
static void BtDfuSmpNotifCB(BtGattChar_t *pChar, bool bEnable,
							uint16_t ConnHdl)
{
	(void)pChar;
	(void)bEnable;
	(void)ConnHdl;

	BtDfuSmpTxReady();
}

// Notify what the stack takes now; the rest goes on the next completion.
static void BtDfuSmpSend(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	bool sent = s_BtDfuSmpTxLen != 0;

	while (s_BtDfuSmpTxOff < s_BtDfuSmpTxLen)
	{
		BtDevice_t *peer = BtPeerFindByHdl(s_BtDfuSmpConnHdl);
		if (peer == nullptr)
		{
			// Gone: the response has nowhere to go.
			s_BtDfuSmpTxLen = 0;
			sent = false;
			break;
		}

		uint32_t mtu = peer->Conn.MaxMtu >= BT_ATT_MTU_MIN ? peer->Conn.MaxMtu :
															 BT_ATT_MTU_MIN;
		uint32_t n = s_BtDfuSmpTxLen - s_BtDfuSmpTxOff;
		if (n > mtu - 3)
		{
			n = mtu - 3;
		}
		if (n > BT_DFUSMP_CHAR_MAXLEN)
		{
			n = BT_DFUSMP_CHAR_MAXLEN;
		}

		if (BtGattCharNotifyEnabled(s_BtDfuSmpConnHdl, &s_BtDfuSmpChar[0]) ==
			false)
		{
			// The host stopped listening: the rest has nowhere to go.
			sent = false;
			break;
		}
		if (BtGattCharNotify(s_BtDfuSmpConnHdl, &s_BtDfuSmpChar[0],
							 s_BtDfuSmpCfg.pTxBuf + s_BtDfuSmpTxOff, n) == false)
		{
			// Retain the unsent bytes. Any TX completion wakes the worker;
			// its status check also recovers a refused continuation event.
			// Do not requeue here while the stack still refuses the send.
			return;
		}
		s_BtDfuSmpTxOff += n;
	}

	s_BtDfuSmpTxLen = 0;
	s_BtDfuSmpTxOff = 0;

	if (sent && s_BtDfuSmpCfg.TxDoneCB != nullptr)
	{
		s_BtDfuSmpCfg.TxDoneCB();
	}

	// A request that came in while this response was going out.
	if (s_bBtDfuSmpRxReady)
	{
		(void)BtEvtQue(0, nullptr, BtDfuSmpProcess);
	}
}

static void BtDfuSmpProcess(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	if (s_bBtDfuSmpRxReady == false)
	{
		// Nothing whole yet.
		return;
	}
	if (s_BtDfuSmpTxLen != 0)
	{
		// The last response still owns the buffer. Go on with it, the stack
		// may have refused it while no completion of ours was due.
		// BtDfuSmpSend queues this again when it is done.
		BtDfuSmpSend(0, nullptr);
		return;
	}

	int n = s_BtDfuSmpCfg.pMgr->Process(s_BtDfuSmpCfg.pRxBuf, s_BtDfuSmpRxLen,
										s_BtDfuSmpCfg.pTxBuf,
										s_BtDfuSmpCfg.TxBufSize);

	// The request buffer is free again for the host's next one.
	uint32_t state = DisableInterrupt();
	s_BtDfuSmpRxLen = 0;
	s_bBtDfuSmpRxReady = false;
	EnableInterrupt(state);

	if (n <= 0)
	{
		return;
	}

	s_BtDfuSmpConnHdl = BtDfuSmpConnHdl();
	if (s_BtDfuSmpConnHdl == BT_CONN_HDL_INVALID)
	{
		return;
	}

	s_BtDfuSmpTxLen = (uint32_t)n;
	s_BtDfuSmpTxOff = 0;
	BtDfuSmpSend(0, nullptr);
}

bool BtDfuSmpInit(const BtDfuSmpCfg_t &Cfg)
{
	if (Cfg.pMgr == nullptr || Cfg.pRxBuf == nullptr ||
		Cfg.RxBufSize < DFUSMP_HDR_SIZE || Cfg.pTxBuf == nullptr ||
		Cfg.TxBufSize < DFUSMP_HDR_SIZE)
	{
		return false;
	}

	s_BtDfuSmpCfg = Cfg;
	s_BtDfuSmpSrvc.SecType = Cfg.SecType;
	BtDfuSmpReset();

	return BtGattSrvcAdd(&s_BtDfuSmpSrvc);
}

BtGattSrvc_t *BtDfuSmpSrvc(void)
{
	return &s_BtDfuSmpSrvc;
}

void BtDfuSmpReset(void)
{
	uint32_t state = DisableInterrupt();
	s_BtDfuSmpRxLen = 0;
	s_bBtDfuSmpRxReady = false;
	EnableInterrupt(state);

	s_BtDfuSmpTxLen = 0;
	s_BtDfuSmpTxOff = 0;
	s_BtDfuSmpConnHdl = BT_CONN_HDL_INVALID;

	if (s_BtDfuSmpCfg.pMgr != nullptr)
	{
		s_BtDfuSmpCfg.pMgr->UploadAbort();
	}
}

bool BtDfuSmpTxBusy(void)
{
	return s_BtDfuSmpTxLen != 0 || s_bBtDfuSmpRxReady;
}

void BtDfuSmpCheckStatus(void)
{
	if (BtDfuSmpTxBusy())
	{
		// This is the same owner as BtDfuSmpSend's queued callback. A
		// refused send returns without queuing itself, allowing sleep.
		// Complete requests are queued by Send, keeping flash work in
		// BtDfuSmpProcess.
		BtDfuSmpSend(0, nullptr);
	}
}

/** @} */
