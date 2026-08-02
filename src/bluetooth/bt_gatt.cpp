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

	memcpy(pChar->pValue, pVal, l);
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

bool BtGattCccdSet(uint16_t ConnHdl, uint16_t CccdHdl, uint16_t Value)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	BtGattChar_t *pChar = BtGattFindCharByCccd(CccdHdl);

	if (pConn == nullptr || pChar == nullptr)
	{
		return false;
	}

	// Also checked here, not only in the ATT write path, so a bond restore or
	// a port that reports subscriptions on its own cannot install a value the
	// characteristic does not support.
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

	// Persist for bonded clients; no-op (volatile) otherwise.
	BtSmpBondCccdSave(ConnHdl, CccdHdl, Value);

	bool oldNotify = (oldValue & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;
	bool newNotify = (Value & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) != 0;
	if (oldNotify != newNotify && pChar->SetNotifCB != nullptr)
	{
		pChar->SetNotifCB(pChar, newNotify, ConnHdl);
	}

	bool oldIndic = (oldValue & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;
	bool newIndic = (Value & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) != 0;
	if (oldIndic != newIndic && pChar->SetIndCB != nullptr)
	{
		pChar->SetIndCB(pChar, newIndic, ConnHdl);
	}

	return true;
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

		// Report the disable edge the same way a write to zero would. An
		// application that starts a producer or takes a per-client resource on
		// the enable edge has no other way to learn the link is gone, and was
		// left holding that state after every disconnect.
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

// Restore persisted CCCD subscriptions for a bonded peer once its link is
// encrypted, replaying each through BtGattCccdSet so the live state, the
// per-characteristic aggregate, and the notify/indicate re-arm callbacks all
// run. A no-op for unbonded links and fresh pairings (empty bond CCCD set).
void BtGattCccdRestoreBonded(uint16_t ConnHdl)
{
	uint16_t hdl[BT_GATT_CCCD_STATE_MAX];
	uint16_t val[BT_GATT_CCCD_STATE_MAX];

	uint8_t n = BtSmpBondCccdGet(ConnHdl, hdl, val, BT_GATT_CCCD_STATE_MAX);

	for (uint8_t i = 0; i < n; i++)
	{
		BtGattCccdSet(ConnHdl, hdl[i], val[i]);
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

// Forward: defined with the TX-pending ring below, used by the send path.
static uint16_t BtGattTxPktCount(BtDevice_t *pConn, uint16_t AclLen);

// Build and send a server-initiated Handle Value PDU - Notification (0x1B) or
// Indication (0x1D) - to the peer on ConnHdl. The two PDUs share the same
// {opcode, value handle, value[]} wire layout. The value is clamped to
// ATT_MTU - 3 and to the ACL transmit buffer. Returns the number of value
// bytes sent, or -1 if the link is unknown or the transport refused the packet.
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

	// A notifiable/indicatable value holds at most ATT_MTU - 3 bytes (opcode
	// + value handle). Fall back to the default MTU if none was negotiated.
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
	// HandleValueNtf and HandleValueInd overlay the same handle+data layout.
	l2pdu->Att.HandleValueNtf.ValHdl = ValHdl;
	if (Len > 0)
	{
		memcpy(l2pdu->Att.HandleValueNtf.Data, pVal, Len);
	}
	l2pdu->Hdr.Len = 1 + 2 + Len;	// opcode + value handle + value
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	// Reserve the packets this PDU becomes before any of it goes out. A ring
	// with no room refuses the send outright rather than transmitting a value
	// whose completion can never be attributed.
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

// Reserve a group of NbPkt HCI packets on this link before they are handed to
// the transport, owned by pChar. Reservation comes first because a full ring
// has to stop the send: the old code sent the notification and then dropped the
// tracking, so the caller was told the value went out and the completion
// callback never came.
//
// A characteristic with no TxCompleteCB still takes a slot. It consumes
// controller completions like any other traffic, and leaving it out was enough
// on its own to walk the ring out of step with the link.
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

// Give back the reservation the transport then refused. Only the most recent
// one can be released, which is all the send paths need: they reserve, send,
// and release on failure with nothing in between.
void BtGattTxPendRelease(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

	if (pConn == nullptr || pConn->TxPendCount == 0)
	{
		return;
	}

	pConn->TxPendCount--;
}

// Account for HCI packets this link sent that carry no GATT completion: ATT
// responses, SMP PDUs, L2CAP signaling. They own no callback but the controller
// still reports them, and without an entry of their own their completions drain
// the ring and fire someone else's callback early.
//
// Consecutive untracked packets coalesce into one entry, so ordinary request
// and response traffic does not consume the ring. If there is no room at all
// the count is added to the last entry instead of being dropped: that delays a
// callback rather than firing it early, which is the safe direction to err.
void BtGattTxPendUntracked(uint16_t ConnHdl, uint16_t NbPkt)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

	if (pConn == nullptr || NbPkt == 0)
	{
		return;
	}

	if (pConn->TxPendCount > 0)
	{
		uint8_t tail = (uint8_t)((pConn->TxPendHead + pConn->TxPendCount - 1) %
								 BT_DEV_TXPEND_MAX);

		if (pConn->TxPend[tail].pChar == nullptr ||
			pConn->TxPendCount >= BT_DEV_TXPEND_MAX)
		{
			pConn->TxPend[tail].Remain = (uint16_t)(pConn->TxPend[tail].Remain + NbPkt);
			return;
		}
	}

	BtGattTxPendReserve(ConnHdl, nullptr, NbPkt);
}

// Number of HCI ACL packets a PDU of AclLen octets is sent as. The controller
// reports one completion per packet, so this is what a send has to reserve.
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

// Send a Handle Value Notification for pChar to the client on ConnHdl. No-op
// (returns false) unless the client subscribed for notifications via the CCCD.
// Weak so a vendor port (e.g. SoftDevice sd_ble_gatts_hvx) can override the
// generic HCI transmit path with a native one.
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

// Send a Handle Value Indication for pChar to the client on ConnHdl. Only one
// indication may be outstanding per link until the client returns a Handle
// Value Confirmation (Core spec Vol 3 Part F, 3.4.7.2); a second indication
// while one is pending returns false. No-op unless the client subscribed for
// indications via the CCCD.
// Weak so a vendor port can override with a native indication path.
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

// Clear the outstanding-indication flag for ConnHdl when the peer's Handle
// Value Confirmation arrives, allowing the next indication to be sent. The ATT
// transaction timeout for a peer that never confirms is handled by
// BtGattIndicationTimeoutCheck (call it periodically from the app).
void BtGattHandleValueConfirm(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn != nullptr)
	{
		pConn->Conn.bIndCfmPending = false;
	}
}

// Client-role receive hook. Weak default does nothing; the application overrides
// it to consume notified/indicated values. Called from the native-host ATT path
// and from the vendor GATT-client event handlers, so one override serves every
// port.
__attribute__((weak)) void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl,
												uint8_t *pData, uint16_t Len)
{
	(void)ConnHdl;
	(void)ValHdl;
	(void)pData;
	(void)Len;
}

// Millisecond tick for the indication transaction timeout. Weak default returns
// 0 so the timeout is inert on ports that do not supply a clock; the elapsed
// time is then always 0 and BtGattIndicationTimedOut never fires. An app with a
// running millisecond counter overrides this.
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

	// Unsigned subtraction handles counter rollover, so a tick rollover during
	// the wait does not produce a false timeout.
	return (uint32_t)(BtGattMsTick() - pConn->Conn.IndCfmTime) >= TimeoutMs;
}

// Action taken when an indication is not confirmed within the ATT transaction
// timeout. Core Vol 3 Part F 3.3.3 requires the bearer be closed on a
// transaction timeout; for the LE-U fixed ATT bearer that means disconnecting
// the link. The weak default clears the outstanding-indication flag and issues
// an HCI Disconnect through the connection's controller command path when one
// is bound. A port may override this whole function to route the disconnect
// through its own stack.
__attribute__((weak)) void BtGattIndicationTimeout(uint16_t ConnHdl)
{
	BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
	if (pConn == nullptr)
	{
		return;
	}

	pConn->Conn.bIndCfmPending = false;

	// Reason 0x13 (Remote User Terminated Connection) is the conventional
	// host-initiated disconnect reason. Guarded so ports with no controller
	// command path fall back to the flag release without dereferencing null.
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

// Undo a partial registration: drop every attribute added since the mark and
// return the characteristics to the state they had before the attempt, so a
// caller that retries or gives up is not left with half a service.
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

	// Already registered? Scan the service list; if this exact service is
	// present, no-op. Guards against duplicate ATT DB entries and a list
	// cycle if BtGattSrvcAdd (e.g. via BtGapInit) runs more than once.
	for (BtGattSrvc_t *p = BtGattGetSrvcList(); p != nullptr; p = p->pNext)
	{
		if (p == pSrvc)
		{
			return true;
		}
	}

	// Add base UUID to internal list for custom 128-bit services.
	if (pSrvc->bCustom)
	{
		baseidx = BtUuidAddBase(pSrvc->UuidBase);
	}

	pSrvc->Uuid = { baseidx, BT_UUID_TYPE_16, { pSrvc->UuidSrvc } };
	pSrvc->Hdl  = BT_ATT_HANDLE_INVALID;

	BtUuid16_t typeuuid = {0, BT_UUID_TYPE_16, BT_UUID_DECLARATIONS_PRIMARY_SERVICE };

	int l = sizeof(BtAttSrvcDeclar_t);

	// Registering a service means adding one attribute per declaration,
	// value and descriptor. If any of them fails the ones already added are
	// dropped back to this position, so a failed call leaves the database and
	// the handle counter as they were.
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

		// Characteristic value
		typeuuid = {baseidx, BT_UUID_TYPE_16, c->Uuid };
		entry = BtAttDBAddEntry(&typeuuid, c->MaxDataLen + sizeof(BtAttCharValue_t));
		if (entry == nullptr)
		{
			return BtGattSrvcAddFailed(pSrvc, &mark);
		}
		BtAttCharValue_t *charval = (BtAttCharValue_t*)entry->Data;

		charval->pChar = c;
		c->ValHdl = entry->Hdl;

		// The caller may have supplied an initial value through pValue and
		// ValueLen. Read both before pValue is repointed at the database buffer,
		// or the value is lost: the old code overwrote the pointer and then
		// tested a ValueLen that no longer described anything reachable.
		void     *pInitVal = c->pValue;
		uint16_t  initLen  = c->ValueLen;

		c->pValue = charval->Data;

		// A declared length the attribute cannot hold is a configuration error.
		// Refusing beats reinterpreting it as a full-size zero value, which is
		// what happened before and hid the mistake.
		if (initLen > c->MaxDataLen)
		{
			return BtGattSrvcAddFailed(pSrvc, &mark);
		}

		// The database buffer is always cleared. BtAttDBUnwind reclaims memory
		// without wiping it, so a failed registration can leave bytes behind
		// that a later one would otherwise serve.
		memset(c->pValue, 0, c->MaxDataLen);

		if (initLen > 0 && pInitVal != nullptr)
		{
			memcpy(c->pValue, pInitVal, initLen);
			c->ValueLen = initLen;
		}
		else
		{
			// No initial value. ValueLen is read back by BtAttReadValue for
			// every Read Request and is not set anywhere else until the first
			// write or an explicit BtGattCharSetValue, so a discovery-time read
			// would otherwise return nothing and the central would show the
			// characteristic as unreadable. Default to the full fixed size over
			// the zeroed buffer. A caller that wants an empty value declares
			// pValue with ValueLen 0 and calls BtGattCharSetValue.
			c->ValueLen = c->MaxDataLen;
		}

		c->bNotify = false;
		c->bIndic  = false;
		if (c->Property & (BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_INDICATE))
		{
			// Characteristic Descriptor CCC
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
			// Characteristic Description
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

// HCI Number-Of-Completed-Packets hook. The controller reports per-connection
// packet completions but not which characteristic produced them, so each
// outgoing notification/indication records its char on a per-peer ring via
// BtGattTxPendPush. Here we dequeue in send order and fire TxCompleteCB on the
// originating char.
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

	// NbPktSent counts HCI ACL packets, which is what the ring is now built
	// from. Take them off the head group in send order; a group finishes only
	// when its last packet is reported, so a fragmented notification fires once
	// and a group with no owner (an ATT response, SMP, L2CAP signaling) absorbs
	// its own completions instead of firing someone else's callback.
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

		if (c == nullptr || c->TxCompleteCB == nullptr)
		{
			continue;
		}

		int idx = 0;
		if (c->pSrvc != nullptr)
		{
			for (int k = 0; k < c->pSrvc->NbChar; k++)
			{
				if (&c->pSrvc->pCharArray[k] == c)
				{
					idx = k;
					break;
				}
			}
		}
		c->TxCompleteCB(c, idx);
	}
}


