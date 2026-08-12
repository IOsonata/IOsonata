/**-------------------------------------------------------------------------
@file	bt_gatt_hci.cpp

@brief	Bluetooth GATT

Generic implementation of Bluetooth Generic Attribute Profile over standard HCI.

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

#include "istddef.h"
#include "crypto/icrypto.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_smp.h"

static BtGattChar_t *BtGattHciEntryChar(BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	if (pEntry->TypeUuid.BaseIdx != 0)
	{
		return ((BtAttCharValue_t *)pEntry->Data)->pChar;
	}

	switch (pEntry->TypeUuid.Uuid)
	{
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
			return ((BtDescClientCharConfig_t *)pEntry->Data)->pChar;

		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
			return ((BtDescCharUserDesc_t *)pEntry->Data)->pChar;

		case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
		case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
		case BT_UUID_DECLARATIONS_INCLUDE:
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
		case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
			return nullptr;

		default:
			return ((BtAttCharValue_t *)pEntry->Data)->pChar;
	}
}

static bool BtGattHciSignedMitm(uint16_t ConnHdl)
{
	BtSmpKeys_t keys;
	bool ok = BtSmpBondKeysLookup(ConnHdl, 0U, 0U, &keys) &&
			  keys.bAuthenticated;
	CryptoSecureWipe(&keys, sizeof(keys));
	return ok;
}

uint8_t BtAttAccessSecurityError(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
													  bool bRead)
{
	BtGattChar_t *pChar = BtGattHciEntryChar(pEntry);
	if (pChar == nullptr)
	{
		return 0;
	}

	uint8_t secType = BtGattSecTypeGet(pChar->pSrvc, pChar,
		g_BtAppData.SecType);
	if (BtGattSecTypeIsOpen(secType))
	{
		return 0;
	}

	if (!bRead &&
		(secType == BT_GAP_SECTYPE_SIGNED_NO_MITM ||
		 secType == BT_GAP_SECTYPE_SIGNED_MITM) &&
		(pChar->Property & BT_GATT_CHAR_PROP_AUTH_SIGNED) != 0 &&
		(pChar->Property & (BT_GATT_CHAR_PROP_WRITE |
							BT_GATT_CHAR_PROP_WRITE_WORESP)) == 0)
	{
		if (secType == BT_GAP_SECTYPE_SIGNED_MITM &&
			!BtGattHciSignedMitm(ConnHdl))
		{
			return BT_ATT_ERROR_INSUF_AUTHEN;
		}
		return 0;
	}

	BtConnSec_t sec = {};
	(void)BtGapConnSecGet(ConnHdl, &sec);

	uint8_t minLevel = BT_GAP_SEC_LEVEL_ENC_UNAUTH;
	bool requireSc = false;

	switch (secType)
	{
		case BT_GAP_SECTYPE_STATICKEY_NO_MITM:
			break;

		case BT_GAP_SECTYPE_STATICKEY_MITM:
			minLevel = BT_GAP_SEC_LEVEL_ENC_AUTH;
			break;

		case BT_GAP_SECTYPE_LESC_MITM:
			minLevel = BT_GAP_SEC_LEVEL_LESC_AUTH;
			requireSc = true;
			break;

		case BT_GAP_SECTYPE_SIGNED_NO_MITM:
			requireSc = true;
			break;

		case BT_GAP_SECTYPE_SIGNED_MITM:
			minLevel = BT_GAP_SEC_LEVEL_LESC_AUTH;
			requireSc = true;
			break;

		default:
			return BT_ATT_ERROR_INSUF_AUTHEN;
	}

	if (sec.Level == BT_GAP_SEC_LEVEL_NONE)
	{
		return BT_ATT_ERROR_INSUF_ENCRYPT;
	}

	if (sec.KeySize != 0 && sec.KeySize < 7)
	{
		return BT_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT;
	}

	if (sec.Level < minLevel ||
		(requireSc && (sec.Flags & BT_GAP_SEC_FLAG_SC) == 0))
	{
		return BT_ATT_ERROR_INSUF_AUTHEN;
	}

	return 0;
}

static bool BtGattHciSendHandleValue(uint16_t ConnHdl, BtGattChar_t *pChar,
										uint8_t OpCode, void * const pData,
										size_t Len, BtGattChar_t *pTxOwner)
{
	if (pChar == nullptr || pChar->ValHdl == BT_ATT_HANDLE_INVALID)
	{
		return false;
	}

	if (Len > 0 && (pData == nullptr || pChar->pValue == nullptr))
	{
		return false;
	}

	if (Len > pChar->MaxDataLen || Len > UINT16_MAX)
	{
		return false;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pPeer->pHciDev == nullptr)
	{
		return false;
	}

	uint16_t mtu = pPeer->Conn.MaxMtu >= BT_ATT_MTU_MIN ?
					pPeer->Conn.MaxMtu : BT_ATT_MTU_MIN;
	size_t maxData = (size_t)mtu - 3;
	size_t bufMax = BT_HCI_BUFFER_MAX_SIZE - sizeof(BtHciACLDataPacketHdr_t) -
					sizeof(BtL2CapHdr_t) - sizeof(BtAttHandleValueNtf_t);
	if (maxData > bufMax)
	{
		maxData = bufMax;
	}
	if (Len > maxData)
	{
		return false;
	}

	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	l2pdu->Att.OpCode = OpCode;
	l2pdu->Att.HandleValueNtf.ValHdl = pChar->ValHdl;
	if (Len > 0)
	{
		memcpy(l2pdu->Att.HandleValueNtf.Data, pData, Len);
	}
	l2pdu->Hdr.Len = sizeof(BtAttHandleValueNtf_t) + Len;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t aclLen = acl->Hdr.Len + sizeof(BtHciACLDataPacketHdr_t);
	uint16_t max = pPeer->pHciDev->AclMaxLen;
	uint16_t nbpkt = max > 0 ?
			(uint16_t)((acl->Hdr.Len + max - 1) / max) : (uint16_t)1;

	if (BtGattTxPendReserve(ConnHdl, pTxOwner, nbpkt) == false)
	{
		return false;
	}

	if (BtHciSendAcl(pPeer->pHciDev, acl) != aclLen)
	{
		BtGattTxPendRelease(ConnHdl);
		return false;
	}

	if (Len > 0)
	{
		memcpy(pChar->pValue, pData, Len);
	}
	pChar->ValueLen = (uint16_t)Len;
	return true;
}

bool BtGattCharNotify(uint16_t ConnHdl, BtGattChar_t *pChar,
						  void * const pData, size_t Len)
{
	if (pChar == nullptr ||
		BtGattCharNotifyEnabled(ConnHdl, pChar) == false)
	{
		return false;
	}

	return BtGattHciSendHandleValue(ConnHdl, pChar,
									BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF,
									pData, Len, pChar);
}

bool BtGattCharIndicate(uint16_t ConnHdl, BtGattChar_t *pChar,
							void * const pData, size_t Len)
{
	if (pChar == nullptr ||
		BtGattCharIndicateEnabled(ConnHdl, pChar) == false)
	{
		return false;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || pPeer->Conn.bIndCfmPending)
	{
		return false;
	}

	if (BtGattHciSendHandleValue(ConnHdl, pChar,
									 BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND,
									 pData, Len, nullptr) == false)
	{
		return false;
	}

	pPeer->Conn.bIndCfmPending = true;
	pPeer->Conn.IndCfmTime = BtGattMsTick();
	pPeer->pIndChar = pChar;
	return true;
}
