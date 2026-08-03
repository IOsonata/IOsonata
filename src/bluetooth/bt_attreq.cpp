/**-------------------------------------------------------------------------
@file	bt_attreq.cpp

@brief	Generic Bluetooth ATT protocol request

Generic definitions for Bluetooth Attribute Protocol implementation
Send request

@author	Hoang Nguyen Hoan
@date	Jan. 16, 2024

@license

MIT License

Copyright (c) 2024, I-SYST, all rights reserved

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

#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"

/******** For DEBUG Trace ************/
//#define DEBUG_ENABLE

#if !defined(NDEBUG) && defined(DEBUG_ENABLE)
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

#ifndef BT_ATT_TRANSACTION_TIMEOUT_MS
#define BT_ATT_TRANSACTION_TIMEOUT_MS		30000U
#endif

static void BtAttTransactionTimeout(BtDevice_t *pPeer, uint32_t Now)
{
	if (pPeer == nullptr || !pPeer->bAttReqPending || pPeer->bAttTimedOut ||
		(uint32_t)(Now - pPeer->AttReqTime) < BT_ATT_TRANSACTION_TIMEOUT_MS)
	{
		return;
	}

	pPeer->bAttReqPending = false;
	pPeer->bAttTimedOut = true;
	pPeer->AttReqOpcode = 0;
	pPeer->AttRspOpcode = 0;
	pPeer->AttReqTime = 0;

	if (pPeer->pHciDev != nullptr && pPeer->pHciDev->Command != nullptr)
	{
		uint8_t param[3];
		param[0] = (uint8_t)(pPeer->Conn.Hdl & 0xFF);
		param[1] = (uint8_t)((pPeer->Conn.Hdl >> 8) & 0xFF);
		param[2] = 0x13;
		pPeer->pHciDev->Command(pPeer->pHciDev,
				BT_HCI_CMD_LINKCTRL_DISCONNECT, param, sizeof(param),
				nullptr, 0);
	}
}

void BtAttTransactionTimeoutCheck(void)
{
	uint32_t now = BtGattMsTick();
	if (now == 0)
	{
		return;
	}

	for (uint16_t i = 0; i < BtPeerCount(); i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer == nullptr || pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}
		BtAttTransactionTimeout(pPeer, now);
	}
}

static uint16_t BtAttRequestMtu(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	return pPeer != nullptr && pPeer->Conn.MaxMtu >= BT_ATT_MTU_MIN ?
		pPeer->Conn.MaxMtu : BT_ATT_MTU_MIN;
}

static uint16_t BtAttAclPacketCount(const BtHciDevice_t *pDev, uint16_t AclLen)
{
	if (pDev == nullptr || pDev->AclMaxLen == 0 || AclLen == 0)
	{
		return 1;
	}
	return (uint16_t)((AclLen + pDev->AclMaxLen - 1U) / pDev->AclMaxLen);
}

static uint8_t BtAttExpectedResponse(uint8_t OpCode)
{
	switch (OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			return BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;
		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:
			return BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP;
		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:
			return BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
			return BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;
		case BT_ATT_OPCODE_ATT_READ_REQ:
			return BT_ATT_OPCODE_ATT_READ_RSP;
		case BT_ATT_OPCODE_ATT_READ_BLOB_REQ:
			return BT_ATT_OPCODE_ATT_READ_BLOB_RSP;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:
			return BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
			return BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;
		case BT_ATT_OPCODE_ATT_WRITE_REQ:
			return BT_ATT_OPCODE_ATT_WRITE_RSP;
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:
			return BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP;
		case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ:
			return BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP;
		default:
			return 0;
	}
}

static bool BtAttSendRequest(BtHciDevice_t *pDev, BtHciACLDataPacket_t *pAcl)
{
	if (pDev == nullptr || pAcl == nullptr || pDev->SendData == nullptr)
	{
		return false;
	}

	uint16_t connHdl = pAcl->Hdr.ConnHdl;
	BtL2CapPdu_t *pL2 = (BtL2CapPdu_t*)pAcl->Data;
	uint8_t request = pL2->Att.OpCode;
	uint8_t response = BtAttExpectedResponse(request);
	BtDevice_t *pPeer = nullptr;

	if (response != 0)
	{
		pPeer = BtPeerFindByHdl(connHdl);
		if (pPeer == nullptr || pPeer->bAttTimedOut)
		{
			return false;
		}

		if (pPeer->bAttReqPending)
		{
			uint32_t now = BtGattMsTick();
			if (now != 0)
			{
				BtAttTransactionTimeout(pPeer, now);
			}
			return false;
		}
	}

	uint16_t packets = BtAttAclPacketCount(pDev, pAcl->Hdr.Len);
	if (!BtGattTxPendReserve(connHdl, nullptr, packets))
	{
		return false;
	}

	if (pPeer != nullptr)
	{
		pPeer->bAttReqPending = true;
		pPeer->AttReqOpcode = request;
		pPeer->AttRspOpcode = response;
		pPeer->AttReqTime = BtGattMsTick();
	}

	if (BtHciSendAcl(pDev, pAcl) == 0)
	{
		BtGattTxPendRelease(connHdl);
		if (pPeer != nullptr)
		{
			pPeer->bAttReqPending = false;
			pPeer->AttReqOpcode = 0;
			pPeer->AttRspOpcode = 0;
			pPeer->AttReqTime = 0;
		}
		return false;
	}

	return true;
}

bool BtAttExchangeMtuRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t Mtu)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ;
	l2pdu->Att.ExchgMtuReqRsp.RxMtu = Mtu;
	l2pdu->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t) + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	bool ok = BtAttSendRequest(pDev, acl);
	DEBUG_PRINTF("BtAttExchangeMtuRequest : %s, %d\r\n", ok ? "SUCCESS" : "FAILED", Mtu);
	return ok;
}

bool BtAttFindInformationRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ;
	l2pdu->Att.FindInfoReq.StartHdl = StartHdl;
	l2pdu->Att.FindInfoReq.EndHdl = EndHdl;
	l2pdu->Hdr.Len = sizeof(BtAttFindInfoReq_t) + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttFindByTypeValueRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t StartHdl, uint16_t EndHdl, uint16_t AttType,
		uint8_t *pAttVal, size_t ValLen)
{
	if ((pAttVal == nullptr && ValLen != 0) || ValLen > BtAttRequestMtu(ConnHdl) - 7U)
	{
		return false;
	}

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;
	const size_t bufferMax = BT_HCI_BUFFER_MAX_SIZE - sizeof(acl->Hdr) - sizeof(BtL2CapHdr_t) - 7U;
	if (ValLen > bufferMax)
	{
		return false;
	}

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ;
	l2pdu->Att.FindByTypeValueReq.StartHdl = StartHdl;
	l2pdu->Att.FindByTypeValueReq.EndHdl = EndHdl;
	l2pdu->Att.FindByTypeValueReq.Type = AttType;
	if (ValLen != 0)
	{
		memcpy(l2pdu->Att.FindByTypeValueReq.Val, pAttVal, ValLen);
	}
	l2pdu->Hdr.Len = (uint16_t)(7U + ValLen);
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttReadByTypeRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t StartHdl, uint16_t EndHdl, BtUuid_t *pUuid)
{
	if (pUuid == nullptr)
	{
		return false;
	}

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ;
	l2pdu->Att.ReadByTypeReq.StartHdl = StartHdl;
	l2pdu->Att.ReadByTypeReq.EndHdl = EndHdl;
	l2pdu->Hdr.Len = 5;
	if (pUuid->BaseIdx > 0 || pUuid->Type == BT_UUID_TYPE_32)
	{
		if (!BtUuidTo128(pUuid, l2pdu->Att.ReadByTypeReq.Uuid.Uuid128))
		{
			return false;
		}
		l2pdu->Hdr.Len += 16;
	}
	else
	{
		l2pdu->Att.ReadByTypeReq.Uuid.Uuid16 = pUuid->Uuid16;
		l2pdu->Hdr.Len += 2;
	}
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttReadRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t Hdl)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	l2pdu->Att.ReadReq.Hdl = Hdl;
	l2pdu->Hdr.Len = 3;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttWriteRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t Hdl, uint8_t *pData, size_t DataLen)
{
	if (pData == nullptr && DataLen != 0)
	{
		return false;
	}

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;
	size_t maxData = BtAttRequestMtu(ConnHdl) - 3U;
	size_t bufferMax = BT_HCI_BUFFER_MAX_SIZE - sizeof(acl->Hdr) - sizeof(BtL2CapHdr_t) - 3U;
	if (maxData > bufferMax)
	{
		maxData = bufferMax;
	}
	if (DataLen > maxData)
	{
		return false;
	}

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_WRITE_REQ;
	l2pdu->Att.WriteReq.Hdl = Hdl;
	if (DataLen != 0)
	{
		memcpy(l2pdu->Att.WriteReq.Data, pData, DataLen);
	}
	l2pdu->Hdr.Len = (uint16_t)(3U + DataLen);
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttPrepareWriteRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t Hdl, uint16_t Offset, const uint8_t *pData, size_t DataLen)
{
	if ((pData == nullptr && DataLen != 0) || BtAttRequestMtu(ConnHdl) < 5U)
	{
		return false;
	}

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;
	const size_t mtuMax = BtAttRequestMtu(ConnHdl) - 5U;
	const size_t bufferMax = BT_HCI_BUFFER_MAX_SIZE - sizeof(acl->Hdr) -
			sizeof(BtL2CapHdr_t) - 5U;
	if (DataLen > mtuMax || DataLen > bufferMax)
	{
		return false;
	}

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ;
	l2pdu->Att.PrepareWriteReq.Hdl = Hdl;
	l2pdu->Att.PrepareWriteReq.Offset = Offset;
	if (DataLen > 0)
	{
		memcpy(l2pdu->Att.PrepareWriteReq.Data, pData, DataLen);
	}
	l2pdu->Hdr.Len = (uint16_t)(5U + DataLen);
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = (uint16_t)(l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t));

	return BtAttSendRequest(pDev, acl);
}

bool BtAttExecuteWriteRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		bool Execute)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ;
	l2pdu->Att.ExecuteWriteReq.Flag = Execute ? 0x01 : 0x00;
	l2pdu->Hdr.Len = 2;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = (uint16_t)(l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t));

	return BtAttSendRequest(pDev, acl);
}

bool BtAttWriteCommand(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t Hdl, uint8_t *pData, size_t DataLen)
{
	if (pData == nullptr && DataLen != 0)
	{
		return false;
	}

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;
	size_t maxData = BtAttRequestMtu(ConnHdl) - 3U;
	size_t bufferMax = BT_HCI_BUFFER_MAX_SIZE - sizeof(acl->Hdr) - sizeof(BtL2CapHdr_t) - 3U;
	if (maxData > bufferMax)
	{
		maxData = bufferMax;
	}
	if (DataLen > maxData)
	{
		return false;
	}

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_CMD;
	l2pdu->Att.WriteCmd.Hdl = Hdl;
	if (DataLen != 0)
	{
		memcpy(l2pdu->Att.WriteCmd.Data, pData, DataLen);
	}
	l2pdu->Hdr.Len = (uint16_t)(3U + DataLen);
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttReadBlobRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t Hdl, uint16_t Offset)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_BLOB_REQ;
	l2pdu->Att.ReadBlobReq.Hdl = Hdl;
	l2pdu->Att.ReadBlobReq.Offset = Offset;
	l2pdu->Hdr.Len = sizeof(BtAttReadBlobReq_t) + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttReadMultipleRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t *pHdl, size_t NbHdl)
{
	if (pHdl == nullptr || NbHdl < 2)
	{
		return false;
	}

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;
	const size_t dataLen = NbHdl * sizeof(uint16_t);
	if (dataLen + 1U > BtAttRequestMtu(ConnHdl) ||
		dataLen + 1U + sizeof(BtL2CapHdr_t) >
			BT_HCI_BUFFER_MAX_SIZE - sizeof(acl->Hdr))
	{
		return false;
	}

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ;
	memcpy(l2pdu->Att.ReadMultipleReq.Hdl, pHdl, dataLen);
	l2pdu->Hdr.Len = (uint16_t)(dataLen + 1U);
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttReadByGroupTypeRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl,
		uint16_t StartHdl, uint16_t EndHdl, BtUuid_t *pUuid)
{
	if (pUuid == nullptr)
	{
		return false;
	}

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_START_NONFLUSHABLE;
	acl->Hdr.BCFlag = 0;
	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ;
	l2pdu->Att.ReadByGroupTypeReq.StartHdl = StartHdl;
	l2pdu->Att.ReadByGroupTypeReq.EndHdl = EndHdl;
	l2pdu->Hdr.Len = 5;
	if (pUuid->BaseIdx > 0 || pUuid->Type == BT_UUID_TYPE_32)
	{
		if (!BtUuidTo128(pUuid, l2pdu->Att.ReadByGroupTypeReq.Uuid.Uuid128))
		{
			return false;
		}
		l2pdu->Hdr.Len += 16;
	}
	else
	{
		l2pdu->Att.ReadByGroupTypeReq.Uuid.Uuid16 = pUuid->Uuid16;
		l2pdu->Hdr.Len += 2;
	}
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;
	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	return BtAttSendRequest(pDev, acl);
}

bool BtAttStartReadByGroupTypeRequest(BtHciDevice_t * const pDev,
		uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl, BtUuid_t *pUuid)
{
	if (pDev == nullptr || pUuid == nullptr)
	{
		return false;
	}

	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr)
	{
		return false;
	}

	pPeer->Discovery.UuidType = *pUuid;
	return BtAttReadByGroupTypeRequest(pDev, ConnHdl, StartHdl, EndHdl,
		&pPeer->Discovery.UuidType);
}
