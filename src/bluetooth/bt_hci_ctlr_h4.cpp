/**-------------------------------------------------------------------------
@file	bt_hci_ctlr_h4.cpp

@brief	Bridge between an HCI controller and an H4 link

See bt_hci_ctlr_h4.h.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include <stddef.h>

#include "bluetooth/bt_hci_ctlr_h4.h"

// Header lengths of what the controller hands up, after the type byte
#define HDRLEN_EVT			2U
#define HDRLEN_ACL			4U

bool BtHciCtlrH4Init(BtHciCtlrH4_t * const pBridge, BtHciCtlrDev_t * const pCtlr, BtHciH4Dev_t * const pH4)
{
	if (pBridge == nullptr || pCtlr == nullptr || pH4 == nullptr)
	{
		return false;
	}

	pBridge->pCtlr = pCtlr;
	pBridge->pH4 = pH4;
	pBridge->UnsupportedCnt = 0;
	pBridge->CmdFailCnt = 0;
	pBridge->AclFailCnt = 0;
	pBridge->UpFailCnt = 0;
	pBridge->BadLenCnt = 0;

	pCtlr->pObj = pBridge;
	pH4->pCtx = pBridge;

	return true;
}

void BtHciCtlrH4FromHost(BtHciH4Dev_t * const pDev, uint8_t Type, uint8_t *pPkt, uint16_t Len)
{
	BtHciCtlrH4_t *bridge = static_cast<BtHciCtlrH4_t*>(pDev->pCtx);
	BtHciCtlrDev_t *ctlr = bridge->pCtlr;

	switch (Type)
	{
		case BT_HCI_H4_PKT_CMD:
			if (ctlr->SendCommand == nullptr || ctlr->SendCommand(ctlr, pPkt, Len) != Len)
			{
				bridge->CmdFailCnt++;
			}
			break;

		case BT_HCI_H4_PKT_ACL:
			if (ctlr->Send == nullptr || ctlr->Send(ctlr, pPkt, Len) != Len)
			{
				bridge->AclFailCnt++;
			}
			break;

		default:
			bridge->UnsupportedCnt++;
			break;
	}
}

void BtHciCtlrH4FromCtlr(BtHciCtlrDev_t * const pDev, bool bIsEvent, uint8_t *pPacket)
{
	BtHciCtlrH4_t *bridge = static_cast<BtHciCtlrH4_t*>(pDev->pObj);
	uint32_t len;
	uint8_t type;

	if (bIsEvent)
	{
		type = BT_HCI_H4_PKT_EVT;
		len = HDRLEN_EVT + pPacket[1];
	}
	else
	{
		type = BT_HCI_H4_PKT_ACL;
		len = HDRLEN_ACL + (static_cast<uint32_t>(pPacket[2]) | (static_cast<uint32_t>(pPacket[3]) << 8));
	}

	if (len > BT_HCI_H4_PKT_MAXLEN)
	{
		bridge->BadLenCnt++;

		return;
	}

	if (BtHciH4Send(bridge->pH4, type, pPacket, len) == false)
	{
		bridge->UpFailCnt++;
	}
}
