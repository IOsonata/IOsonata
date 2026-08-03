/**-------------------------------------------------------------------------
@file	bt_gatt.cpp

@brief	Bluetooth GATT  

Generic implementation & definitions of Bluetooth Generic Attribute Profile

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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
#include <memory.h>
#include <stdio.h>

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_smp.h"

static BtGattSrvc_t *s_pBtGattSrvcList = nullptr;

BtGattSrvc_t *BtGattGetSrvcList()
{
	return s_pBtGattSrvcList;
}

void BtGattInsertSrvcList(BtGattSrvc_t * const pSrvc)
{
	pSrvc->pPrev = nullptr;
	pSrvc->pNext = s_pBtGattSrvcList;

	if (s_pBtGattSrvcList)
	{
		s_pBtGattSrvcList->pPrev = pSrvc;
	}
	s_pBtGattSrvcList = pSrvc;
}

//size_t BtGattGetValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pBuff)

__attribute__((weak)) bool BtGattCharSetValue(BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr)
	{
		return false;
	}

	if (Len > 0 && pVal == nullptr)
	{
		return false;
	}

	if (pChar->ValHdl == BT_ATT_HANDLE_INVALID || pChar->pValue == nullptr)
	{
		return false;
	}

	// Clamp on size_t before narrowing. Casting Len to uint16_t first would
	// wrap a >= 64 KiB length down to a small value and silently truncate the
	// stored value while still reporting success.
	size_t l = min(Len, (size_t)pChar->MaxDataLen);

	if (l > 0)
	{
		memcpy(pChar->pValue, pVal, l);
	}
	pChar->ValueLen = (uint16_t)l;

	return true;
}

bool isBtGattCharNotifyEnabled(BtGattChar_t *pChar)
{
	if (pChar == nullptr || pChar->CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	// Legacy aggregate view: true when at least one connected peer enabled
	// notification. New code should use BtGattCharNotifyEnabled(ConnHdl,...).
	return pChar->bNotify;
}


static BtGattCccdState_t *BtGattCccdFind(BtDevice_t *pConn, uint16_t CccdHdl, bool bAlloc)
{
	if (pConn == nullptr || CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return nullptr;
	}

	for (uint8_t i = 0; i < pConn->Conn.NbCccd; i++)
	{
		if (pConn->Conn.Cccd[i].Hdl == CccdHdl)
		{
			return &pConn->Conn.Cccd[i];
		}
	}

	if (bAlloc == false || pConn->Conn.NbCccd >= BT_GATT_CCCD_STATE_MAX)
	{
		return nullptr;
	}

	BtGattCccdState_t *pState = &pConn->Conn.Cccd[pConn->Conn.NbCccd++];
	pState->Hdl = CccdHdl;
	pState->Value = 0;

	return pState;
}

static BtGattChar_t *BtGattFindCharByCccd(uint16_t CccdHdl)
{
	if (CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return nullptr;
	}

	for (BtGattSrvc_t *pSrvc = BtGattGetSrvcList(); pSrvc != nullptr; pSrvc = pSrvc->pNext)
	{
		for (int i = 0; i < pSrvc->NbChar; i++)
		{
			BtGattChar_t *pChar = &pSrvc->pCharArray[i];
			if (pChar->CccdHdl == CccdHdl)
			{
				return pChar;
			}
		}
	}

	return nullptr;
}

static void BtGattCccdUpdateMirror(BtGattChar_t *pChar)
{
	if (pChar == nullptr || pChar->CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return;
	}

	bool bNotify = false;
	bool bIndic = false;

	for (uint16_t i = 0; i < BtPeerCount(); i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer == nullptr || pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}

		for (uint8_t j = 0; j < pPeer->Conn.NbCccd; j++)
		{
			if (pPeer->Conn.Cccd[j].Hdl == pChar->CccdHdl)
			{
				uint16_t v = pPeer->Conn.Cccd[j].Value;
				bNotify = bNotify || ((v & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0);
				bIndic = bIndic || ((v & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0);
			}
		}
	}

	pChar->bNotify = bNotify;
	pChar->bIndic = bIndic;

	uint16_t cccval = (bNotify ? BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION : 0) |
					  (bIndic ? BT_DESC_CLIENT_CHAR_CONFIG_INDICATION : 0);
	BtGattCccdDbSync(pChar->CccdHdl, cccval);
}

// Weak no-op default. On a SoftDevice/ST port there is no native ATT DB, so the
// aggregate does not need to be mirrored into a descriptor entry. The native
// host (bt_att.cpp) overrides this with the strong version that writes CccVal.
__attribute__((weak)) void BtGattCccdDbSync(uint16_t CccdHdl, uint16_t CccVal)
{
	(void)CccdHdl;
	(void)CccVal;
}

uint16_t BtGattCccdGet(uint16_t ConnHdl, uint16_t CccdHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	BtGattCccdState_t *pState = BtGattCccdFind(pConn, CccdHdl, false);

	return pState != nullptr ? pState->Value : 0;
}

// A CCCD holds two defined bits and 14 reserved ones, and each defined bit is
// only meaningful when the characteristic declares the matching property
// (Vol 3 Part G 3.3.3.3). Anything else is a value the server cannot honour,
// answered with Client Characteristic Configuration Descriptor Improperly
// Configured (Core Specification Supplement Part B 1.2). Enabling notification
// on a characteristic without the Notify property is the case that matters in
// practice: accepting it leaves the client waiting for packets that never come.
uint8_t BtGattCccdValueError(BtGattChar_t *pChar, uint16_t Value)
{
	if (pChar == nullptr)
	{
		return BT_ATT_ERROR_INVALID_HANDLE;
	}

	if ((Value & (uint16_t)~BT_DESC_CLIENT_CHAR_CONFIG_MASK) != 0)
	{
		return BT_ATT_ERROR_CCCD_IMPROPER_CFG;
	}

	if ((Value & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0 &&
		(pChar->Property & BT_GATT_CHAR_PROP_NOTIFY) == 0)
	{
		return BT_ATT_ERROR_CCCD_IMPROPER_CFG;
	}

	if ((Value & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0 &&
		(pChar->Property & BT_GATT_CHAR_PROP_INDICATE) == 0)
	{
		return BT_ATT_ERROR_CCCD_IMPROPER_CFG;
	}

	return 0;
}

uint8_t BtGattCccdWriteError(uint16_t CccdHdl, uint16_t Value)
{
	return BtGattCccdValueError(BtGattFindCharByCccd(CccdHdl), Value);
}

// Whether BtGattCccdSet would be able to store this value, asked before the
// write is acknowledged. Clearing a subscription always fits, and so does
// changing one that already has a slot on this link; only a new non-zero
// subscription can run the per-link table out of room. Answering a Write
// Response for a subscription that was then dropped leaves the client believing
// it is subscribed, so the ATT layer asks first and answers Insufficient
// Resources instead.
bool BtGattCccdCanStore(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value)
{
	if (Value == 0)
	{
		return true;
	}

	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr)
	{
		return false;
	}

	if (BtGattCccdFind(pConn, CccdHdl, false) != nullptr)
	{
		return true;
	}

	return pConn->Conn.NbCccd < BT_GATT_CCCD_STATE_MAX;
}

static void BtGattCccdReportChange(uint16_t ConnHdl, uint16_t CccdHdl,
									 uint16_t OldValue, uint16_t Value)
{
	BtGattChar_t *pChar = BtGattFindCharByCccd(CccdHdl);
	if (pChar == nullptr)
	{
		return;
	}

	BtSmpBondCccdSave(ConnHdl, CccdHdl, Value);

	bool oldNotify = (OldValue & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;
	bool newNotify = (Value & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;
	if (oldNotify != newNotify && pChar->SetNotifCB != nullptr)
	{
		pChar->SetNotifCB(pChar, newNotify, ConnHdl);
	}

	bool oldIndic = (OldValue & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;
	bool newIndic = (Value & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;
	if (oldIndic != newIndic && pChar->SetIndCB != nullptr)
	{
		pChar->SetIndCB(pChar, newIndic, ConnHdl);
	}
}

static bool BtGattCccdApply(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value,
								 bool bReport, uint16_t *pOldValue)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	BtGattChar_t *pChar = BtGattFindCharByCccd(CccdHdl);

	if (pConn == nullptr || pChar == nullptr)
	{
		return false;
	}

	if (BtGattCccdValueError(pChar, Value) != 0)
	{
		return false;
	}

	uint16_t oldValue = BtGattCccdGet(ConnHdl, CccdHdl);

	if (Value == 0)
	{
		for (uint8_t i = 0; i < pConn->Conn.NbCccd; i++)
		{
			if (pConn->Conn.Cccd[i].Hdl == CccdHdl)
			{
				pConn->Conn.Cccd[i] = pConn->Conn.Cccd[pConn->Conn.NbCccd - 1];
				pConn->Conn.NbCccd--;
				break;
			}
		}
	}
	else
	{
		BtGattCccdState_t *pState = BtGattCccdFind(pConn, CccdHdl, true);
		if (pState == nullptr)
		{
			return false;
		}
		pState->Value = Value;
	}

	BtGattCccdUpdateMirror(pChar);

	if (pOldValue != nullptr)
	{
		*pOldValue = oldValue;
	}
	if (bReport)
	{
		BtGattCccdReportChange(ConnHdl, CccdHdl, oldValue, Value);
	}

	return true;
}

bool BtGattCccdSet(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value)
{
	return BtGattCccdApply(ConnHdl, CccdHdl, Value, true, nullptr);
}

bool BtGattCccdSetDeferred(uint16_t ConnHdl, uint16_t CccdHdl,
								 uint16_t Value, uint16_t *pOldValue)
{
	return BtGattCccdApply(ConnHdl, CccdHdl, Value, false, pOldValue);
}

void BtGattCccdCommitDeferred(uint16_t ConnHdl, uint16_t CccdHdl,
									uint16_t OldValue, uint16_t Value)
{
	BtGattCccdReportChange(ConnHdl, CccdHdl, OldValue, Value);
}

void BtGattCccdClear(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr)
	{
		return;
	}

	uint8_t n = pConn->Conn.NbCccd;
	uint16_t hdl[BT_GATT_CCCD_STATE_MAX];
	uint16_t val[BT_GATT_CCCD_STATE_MAX];

	for (uint8_t i = 0; i < n; i++)
	{
		hdl[i] = pConn->Conn.Cccd[i].Hdl;
		val[i] = pConn->Conn.Cccd[i].Value;
	}

	pConn->Conn.NbCccd = 0;

	for (uint8_t i = 0; i < n; i++)
	{
		BtGattChar_t *pChar = BtGattFindCharByCccd(hdl[i]);

		BtGattCccdUpdateMirror(pChar);

		if (pChar == nullptr)
		{
			continue;
		}

		if ((val[i] & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0 &&
			pChar->SetNotifCB != nullptr)
		{
			pChar->SetNotifCB(pChar, false, ConnHdl);
		}

		if ((val[i] & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0 &&
			pChar->SetIndCB != nullptr)
		{
			pChar->SetIndCB(pChar, false, ConnHdl);
		}
	}
}

// Restore persisted CCCD subscriptions atomically. Validate the whole restored
// set, including projected table occupancy, before changing the live link.
void BtGattCccdRestoreBonded(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr)
	{
		return;
	}

	uint16_t hdl[BT_GATT_CCCD_STATE_MAX];
	uint16_t val[BT_GATT_CCCD_STATE_MAX];
	uint16_t old[BT_GATT_CCCD_STATE_MAX];
	BtGattCccdState_t snapshot[BT_GATT_CCCD_STATE_MAX];
	BtGattCccdState_t projected[BT_GATT_CCCD_STATE_MAX];

	uint8_t n = BtSmpBondCccdGet(ConnHdl, hdl, val, BT_GATT_CCCD_STATE_MAX);
	uint8_t originalCount = pConn->Conn.NbCccd;
	uint8_t projectedCount = originalCount;
	memcpy(snapshot, pConn->Conn.Cccd, sizeof(snapshot));
	memcpy(projected, pConn->Conn.Cccd, sizeof(projected));

	for (uint8_t i = 0; i < n; i++)
	{
		if (BtGattCccdWriteError(hdl[i], val[i]) != 0)
		{
			return;
		}
		for (uint8_t j = 0; j < i; j++)
		{
			if (hdl[j] == hdl[i])
			{
				return;
			}
		}

		int idx = -1;
		for (uint8_t j = 0; j < projectedCount; j++)
		{
			if (projected[j].Hdl == hdl[i])
			{
				idx = j;
				break;
			}
		}

		if (val[i] == 0)
		{
			if (idx >= 0)
			{
				projected[idx] = projected[projectedCount - 1];
				projectedCount--;
			}
		}
		else if (idx >= 0)
		{
			projected[idx].Value = val[i];
		}
		else
		{
			if (projectedCount >= BT_GATT_CCCD_STATE_MAX)
			{
				return;
			}
			projected[projectedCount].Hdl = hdl[i];
			projected[projectedCount].Value = val[i];
			projectedCount++;
		}
	}

	for (uint8_t i = 0; i < n; i++)
	{
		if (BtGattCccdSetDeferred(ConnHdl, hdl[i], val[i], &old[i]) == false)
		{
			memcpy(pConn->Conn.Cccd, snapshot, sizeof(snapshot));
			pConn->Conn.NbCccd = originalCount;
			for (uint8_t j = 0; j < n; j++)
			{
				BtGattCccdUpdateMirror(BtGattFindCharByCccd(hdl[j]));
			}
			return;
		}
	}

	for (uint8_t i = 0; i < n; i++)
	{
		BtGattCccdCommitDeferred(ConnHdl, hdl[i], old[i], val[i]);
	}
}

bool BtGattCharNotifyEnabled(uint16_t ConnHdl, BtGattChar_t *pChar)
{
	if (pChar == nullptr || pChar->CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	return (BtGattCccdGet(ConnHdl, pChar->CccdHdl) &
			BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;
}

bool BtGattCharIndicateEnabled(uint16_t ConnHdl, BtGattChar_t *pChar)
{
	if (pChar == nullptr || pChar->CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	return (BtGattCccdGet(ConnHdl, pChar->CccdHdl) &
			BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;
}

static uint16_t BtGattTxPktCount(BtDevice_t *pConn, uint16_t AclLen);

static int BtGattSendHandleValue(uint16_t ConnHdl, uint8_t OpCode, uint16_t ValHdl,
								 const void *pVal, size_t Len, BtGattChar_t *pOwner)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr || pConn->pHciDev == nullptr || pConn->pHciDev->SendData == nullptr)
	{
		return -1;
	}

	if (Len > 0 && pVal == nullptr)
	{
		return -1;
	}

	uint16_t mtu = pConn->Conn.MaxMtu >= BT_ATT_MTU_MIN ? pConn->Conn.MaxMtu : BT_ATT_MTU_MIN;
	size_t maxData = (size_t)mtu - 3;
	size_t bufMax = BT_HCI_BUFFER_MAX_SIZE - sizeof(BtHciACLDataPacketHdr_t) - sizeof(BtL2CapHdr_t) - 3;
	if (maxData > bufMax)
	{
		maxData = bufMax;
	}
	if (Len > maxData)
	{
		Len = maxData;
	}

	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	l2pdu->Att.OpCode = OpCode;
	l2pdu->Att.HandleValueNtf.ValHdl = ValHdl;
	if (Len > 0)
	{
		memcpy(l2pdu->Att.HandleValueNtf.Data, pVal, Len);
	}
	l2pdu->Hdr.Len = 1 + 2 + Len;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint16_t nbpkt = BtGattTxPktCount(pConn, acl->Hdr.Len);

	if (BtGattTxPendReserve(ConnHdl, pOwner, nbpkt) == false)
	{
		return -1;
	}

	uint32_t sent = BtHciSendAcl(pConn->pHciDev, acl);
	if (sent == 0)
	{
		BtGattTxPendRelease(ConnHdl);
		return -1;
	}

	return (int)Len;
}

bool BtGattTxPendReserve(uint16_t ConnHdl, BtGattChar_t *pChar, uint16_t NbPkt)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

	if (pConn == nullptr || NbPkt == 0)
	{
		return false;
	}

	if (pConn->TxPendCount >= BT_DEV_TXPEND_MAX)
	{
		return false;
	}

	uint8_t idx = (uint8_t)((pConn->TxPendHead + pConn->TxPendCount) % BT_DEV_TXPEND_MAX);
	pConn->TxPend[idx].pChar = pChar;
	pConn->TxPend[idx].Remain = NbPkt;
	pConn->TxPendCount++;

	return true;
}

void BtGattTxPendRelease(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

	if (pConn == nullptr || pConn->TxPendCount == 0)
	{
		return;
	}

	pConn->TxPendCount--;
}

// Compatibility helper for callers that own no completion callback. It still
// occupies its real position in the ordered ring.
bool BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)
{
	return BtGattTxPendReserve(ConnHdl, nullptr, NbPkt);
}

bool BtGattTxPendingAdd(uint16_t ConnHdl, BtGattChar_t *pChar)
{
	return BtGattTxPendReserve(ConnHdl, pChar, 1);
}

static uint16_t BtGattTxPktCount(BtDevice_t *pConn, uint16_t AclLen)
{
	if (pConn == nullptr || pConn->pHciDev == nullptr ||
		pConn->pHciDev->AclMaxLen == 0 || AclLen == 0)
	{
		return 1;
	}

	uint16_t max = pConn->pHciDev->AclMaxLen;

	return (uint16_t)((AclLen + max - 1) / max);
}

__attribute__((weak)) bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr || pChar->CccdHdl == BT_ATT_HANDLE_INVALID ||
		BtGattCharNotifyEnabled(ConnHdl, pChar) == false)
	{
		return false;
	}

	return BtGattSendHandleValue(ConnHdl, BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF,
								 pChar->ValHdl, pVal, Len, pChar) >= 0;
}

__attribute__((weak)) bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *pChar, void * const pVal, size_t Len)
{
	if (pChar == nullptr || pChar->CccdHdl == BT_ATT_HANDLE_INVALID ||
		BtGattCharIndicateEnabled(ConnHdl, pChar) == false)
	{
		return false;
	}

	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr || pConn->Conn.bIndCfmPending)
	{
		return false;
	}

	if (BtGattSendHandleValue(ConnHdl, BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND,
							  pChar->ValHdl, pVal, Len, pChar) < 0)
	{
		return false;
	}

	pConn->Conn.bIndCfmPending = true;
	pConn->Conn.IndCfmTime = BtGattMsTick();
	return true;
}

static void BtGattTxCompleteChar(BtGattChar_t *pChar)
{
	if (pChar == nullptr || pChar->TxCompleteCB == nullptr)
	{
		return;
	}

	int idx = 0;
	if (pChar->pSrvc != nullptr)
	{
		for (int k = 0; k < pChar->pSrvc->NbChar; k++)
		{
			if (&pChar->pSrvc->pCharArray[k] == pChar)
			{
				idx = k;
				break;
			}
		}
	}
	pChar->TxCompleteCB(pChar, idx);
}

void BtGattHandleValueConfirm(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr)
	{
		return;
	}

	pConn->Conn.bIndCfmPending = false;
	BtGattChar_t *pChar = (BtGattChar_t*)pConn->pIndChar;
	pConn->pIndChar = nullptr;
	BtGattTxCompleteChar(pChar);
}

__attribute__((weak)) void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl,
												uint8_t *pData, uint16_t Len)
{
	(void)ConnHdl;
	(void)ValHdl;
	(void)pData;
	(void)Len;
}

__attribute__((weak)) uint32_t BtGattMsTick(void)
{
	return 0;
}

bool BtGattIndicationTimedOut(uint16_t ConnHdl, uint32_t TimeoutMs)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr || pConn->Conn.bIndCfmPending == false)
	{
		return false;
	}

	return (uint32_t)(BtGattMsTick() - pConn->Conn.IndCfmTime) >= TimeoutMs;
}

__attribute__((weak)) void BtGattIndicationTimeout(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr)
	{
		return;
	}

	pConn->Conn.bIndCfmPending = false;
	pConn->pIndChar = nullptr;

	if (pConn->pHciDev != nullptr && pConn->pHciDev->Command != nullptr)
	{
		uint8_t param[3];
		param[0] = (uint8_t)(ConnHdl & 0xFF);
		param[1] = (uint8_t)((ConnHdl >> 8) & 0xFF);
		param[2] = 0x13;
		pConn->pHciDev->Command(pConn->pHciDev,
								BT_HCI_CMD_LINKCTRL_DISCONNECT,
								param, sizeof(param), nullptr, 0);
	}
}

void BtGattIndicationTimeoutCheck(void)
{
	uint16_t hdl[BT_GATT_TIMEOUT_CHECK_MAX];
	size_t n = BtPeerGetConnectedHandles(hdl, BT_GATT_TIMEOUT_CHECK_MAX);

	for (size_t i = 0; i < n; i++)
	{
		if (BtGattIndicationTimedOut(hdl[i], BT_GATT_INDICATION_TIMEOUT_MS))
		{
			BtGattIndicationTimeout(hdl[i]);
		}
	}
}

static bool BtGattSrvcAddFailed(BtGattSrvc_t *pSrvc, const BtAttDBMark_t *pMark)
{
	BtAttDBUnwind(pMark);

	pSrvc->Hdl = BT_ATT_HANDLE_INVALID;

	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		BtGattChar_t *c = &pSrvc->pCharArray[i];

		c->Hdl     = BT_ATT_HANDLE_INVALID;
		c->ValHdl  = BT_ATT_HANDLE_INVALID;
		c->DescHdl = BT_ATT_HANDLE_INVALID;
		c->CccdHdl = BT_ATT_HANDLE_INVALID;
		c->SccdHdl = BT_ATT_HANDLE_INVALID;
		c->pValue  = nullptr;
		c->ValueLen = 0;
		c->bNotify = false;
		c->bIndic  = false;
		c->pSrvc   = nullptr;
	}

	return false;
}

__attribute__((weak)) bool BtGattSrvcAdd(BtGattSrvc_t *pSrvc)
{
	uint8_t baseidx = 0;

	if (pSrvc == nullptr || pSrvc->pCharArray == nullptr || pSrvc->NbChar <= 0)
	{
		return false;
	}

	for (BtGattSrvc_t *p = BtGattGetSrvcList(); p != nullptr; p = p->pNext)
	{
		if (p == pSrvc)
		{
			return true;
		}
	}

	if (pSrvc->bCustom)
	{
		baseidx = BtUuidAddBase(pSrvc->UuidBase);
	}

	pSrvc->Uuid = { baseidx, BT_UUID_TYPE_16, { pSrvc->UuidSrvc } };
	pSrvc->Hdl  = BT_ATT_HANDLE_INVALID;

	BtUuid16_t typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_PRIMARY_SERVICE };

	int l = sizeof(BtAttSrvcDeclar_t);

	BtAttDBMark_t mark;
	BtAttDBMark(&mark);

	BtAttDBEntry_t *srvcentry = BtAttDBAddEntry(&typeuuid, l);

	if (srvcentry == nullptr)
	{
		return BtGattSrvcAddFailed(pSrvc, &mark);
	}

	BtAttSrvcDeclar_t *srvcdec = (BtAttSrvcDeclar_t*) srvcentry->Data;

	srvcdec->Uuid = pSrvc->Uuid;
	srvcdec->pSrvc = pSrvc;

	pSrvc->Hdl = srvcentry->Hdl;

	BtGattChar_t *c = pSrvc->pCharArray;

	BtAttDBEntry_t *entry = nullptr;

	for (int i = 0; i < pSrvc->NbChar; i++, c++)
	{
		typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_CHARACTERISTIC };
		l = sizeof(BtAttCharDeclar_t);

		entry = BtAttDBAddEntry(&typeuuid, l);
		if (entry == nullptr)
		{
			return BtGattSrvcAddFailed(pSrvc, &mark);
		}

		BtAttCharDeclar_t *chardec = (BtAttCharDeclar_t*)entry->Data;

		chardec->Uuid  = { baseidx, BT_UUID_TYPE_16, { c->Uuid } };
		chardec->pChar = c;

		c->ValHdl      = BT_ATT_HANDLE_INVALID;
		c->DescHdl     = BT_ATT_HANDLE_INVALID;
		c->CccdHdl     = BT_ATT_HANDLE_INVALID;
		c->SccdHdl     = BT_ATT_HANDLE_INVALID;
		c->pSrvc       = pSrvc;
		c->BaseUuidIdx = pSrvc->Uuid.BaseIdx;

		typeuuid = {baseidx, BT_UUID_TYPE_16, c->Uuid };
		entry = BtAttDBAddEntry(&typeuuid, c->MaxDataLen + sizeof(BtAttCharValue_t));
		if (entry == nullptr)
		{
			return BtGattSrvcAddFailed(pSrvc, &mark);
		}
		BtAttCharValue_t *charval = (BtAttCharValue_t*)entry->Data;

		charval->pChar = c;
		c->ValHdl = entry->Hdl;

		void     *pInitVal = c->pValue;
		uint16_t  initLen  = c->ValueLen;

		c->pValue = charval->Data;

		if (initLen > c->MaxDataLen)
		{
			return BtGattSrvcAddFailed(pSrvc, &mark);
		}

		memset(c->pValue, 0, c->MaxDataLen);

		if (initLen > 0 && pInitVal != nullptr)
		{
			memcpy(c->pValue, pInitVal, initLen);
			c->ValueLen = initLen;
		}
		else
		{
			c->ValueLen = c->MaxDataLen;
		}

		c->bNotify = false;
		c->bIndic  = false;
		if (c->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
		{
			typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION };
			l = sizeof(BtDescClientCharConfig_t);
			entry = BtAttDBAddEntry(&typeuuid, l);
			if (entry == nullptr)
			{
				return BtGattSrvcAddFailed(pSrvc, &mark);
			}

			BtDescClientCharConfig_t *ccc = (BtDescClientCharConfig_t*)entry->Data;

			ccc->pChar  = c;
			ccc->CccVal = 0;
			c->CccdHdl = entry->Hdl;
		}

		if (c->pDesc)
		{
			typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION };
			size_t dl = sizeof(BtDescCharUserDesc_t);
			entry = BtAttDBAddEntry(&typeuuid, dl);
			if (entry == nullptr)
			{
				return BtGattSrvcAddFailed(pSrvc, &mark);
			}

			BtDescCharUserDesc_t *dcud = (BtDescCharUserDesc_t*)entry->Data;

			dcud->pChar = c;
			c->DescHdl = entry->Hdl;
		}
	}

	BtGattInsertSrvcList(pSrvc);

	return true;
}

__attribute__((weak)) void BtGattSrvcDisconnected(BtGattSrvc_t *pSrvc)
{
	for (int i = 0; i < pSrvc->NbChar; i++)
	{
		if (pSrvc->pCharArray[i].CccdHdl != BT_ATT_HANDLE_INVALID)
		{
			pSrvc->pCharArray[i].bNotify = false;
			pSrvc->pCharArray[i].bIndic  = false;

			BtAttDBEntry_t *entry = BtAttDBFindHandle(pSrvc->pCharArray[i].CccdHdl);
			if (entry)
			{
				BtDescClientCharConfig_t *p = (BtDescClientCharConfig_t*)entry->Data;
				p->CccVal = 0;
			}
		}
	}
}

__attribute__((weak)) void BtGattEvtHandler(uint32_t Evt, void * const pCtx)
{
	if (s_pBtGattSrvcList)
	{
		BtGattSrvc_t *p = s_pBtGattSrvcList;

		while (p)
		{
			BtGattSrvcEvtHandler(p, Evt, pCtx);

			p = p->pNext;
		}
	}
}

void BtGattSendCompleted(uint16_t ConnHdl, uint16_t NbPktSent)
{
	if (NbPktSent == 0)
	{
		return;
	}

	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr)
	{
		return;
	}

	uint16_t n = NbPktSent;

	while (n > 0 && pConn->TxPendCount > 0)
	{
		uint16_t take = min(n, pConn->TxPend[pConn->TxPendHead].Remain);

		pConn->TxPend[pConn->TxPendHead].Remain =
				(uint16_t)(pConn->TxPend[pConn->TxPendHead].Remain - take);
		n = (uint16_t)(n - take);

		if (pConn->TxPend[pConn->TxPendHead].Remain > 0)
		{
			break;
		}

		BtGattChar_t *c = (BtGattChar_t*)pConn->TxPend[pConn->TxPendHead].pChar;
		pConn->TxPendHead = (uint8_t)((pConn->TxPendHead + 1) % BT_DEV_TXPEND_MAX);
		pConn->TxPendCount--;

		BtGattTxCompleteChar(c);
	}
}

