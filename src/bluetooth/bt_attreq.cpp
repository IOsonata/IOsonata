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

#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"

/******** For DEBUG ************/
//#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

bool BtAttExchangeMtuRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t Mtu)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ;
	l2pdu->Att.ExchgMtuReqRsp.RxMtu = Mtu;//BtAttGetMtu();
	l2pdu->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t) + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	DEBUG_PRINTF("BtAttExchangeMtuRequest : %d, %d\r\n", n, Mtu);

//	BtHciSendAtt(BtAttReqRsp_t req, 2);
	return true;
}

bool BtAttFindInformationRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ;
	l2pdu->Att.FindInfoReq.StartHdl = StartHdl;
	l2pdu->Att.FindInfoReq.EndHdl = EndHdl;
	l2pdu->Hdr.Len = sizeof(BtAttFindInfoReq_t) + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	DEBUG_PRINTF("BtAttFindInformationRequest (%s): %d, %d\r\n", n == 0 ? "FAILED" : "SUCCESS", StartHdl, EndHdl);

//	BtHciSendAtt(BtAttReqRsp_t req, 2);
	return true;
}

bool BtAttFindByTypeValueRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl, uint16_t AttType, uint8_t *pAttVal, size_t ValLen)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ;
	l2pdu->Att.FindByTypeValueReq.StartHdl = StartHdl;
	l2pdu->Att.FindByTypeValueReq.EndHdl = EndHdl;
	l2pdu->Att.FindByTypeValueReq.Type = AttType;
	memcpy(l2pdu->Att.FindByTypeValueReq.Val, pAttVal, ValLen);
	l2pdu->Hdr.Len = sizeof(BtAttFindByTypeValueReq_t) + ValLen;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	DEBUG_PRINTF("BtAttFindByTypeValueRequest : %d, %d %d\r\n", StartHdl, EndHdl, AttType);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	return true;
}

bool BtAttReadByTypeRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl, BtUuid_t *pUuid)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ;//BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ;
	l2pdu->Att.ReadByTypeReq.StartHdl = StartHdl;
	l2pdu->Att.ReadByTypeReq.EndHdl = EndHdl;
	l2pdu->Hdr.Len = 5;
	if (pUuid->BaseIdx > 0)
	{
		BtUuidTo128(pUuid, l2pdu->Att.ReadByTypeReq.Uuid.Uuid128);
		l2pdu->Hdr.Len += 16;
	}
	else
	{
		if (pUuid->Type == BT_UUID_TYPE_32)
		{
			l2pdu->Att.ReadByTypeReq.Uuid.Uuid32 = pUuid->Uuid32;
			l2pdu->Hdr.Len += 4;
		}
		else
		{
			l2pdu->Att.ReadByTypeReq.Uuid.Uuid16 = pUuid->Uuid16;
			l2pdu->Hdr.Len += 2;
		}
	}
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	//DEBUG_PRINTF("BtAttReadByTypeRequest : %d, %d %d\r\n", StartHdl, EndHdl, pUuid->BaseIdx);

	return true;
}

bool BtAttReadRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t Hdl)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_REQ;
	l2pdu->Att.ReadReq.Hdl= Hdl;
	l2pdu->Hdr.Len = 3;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	//DEBUG_PRINTF("BtAttReadRequest Hdl: %d\r\n", Hdl);

	return true;
}

bool BtAttReadBlobRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t Hdl, uint16_t Offset)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_BLOB_REQ;
	l2pdu->Att.ReadBlobReq.Hdl= Hdl;
	l2pdu->Att.ReadBlobReq.Offset = Offset;
	l2pdu->Hdr.Len = sizeof(BtAttReadBlobReq_t) + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	DEBUG_PRINTF("BtAttReadBlobRequest : %d %x\r\n", Hdl, Offset);

	return true;
}

bool BtAttReadMultipleRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t *pHdl, size_t NbHdl)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ;
	memcpy(l2pdu->Att.ReadMultipleReq.Hdl, pHdl, NbHdl << 1);
	l2pdu->Hdr.Len = sizeof(BtAttReadMultipleReq_t) * NbHdl + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	DEBUG_PRINTF("BtAttReadMultipleRequest : %x %d\r\n", pHdl[0], NbHdl);

	return true;
}


/**
 * Send AttReadByGroupTypeRequest command to target device
 * For parsing information of a group attribute
 * @param pDev		: Pointer to the BLE HCI object
 * @param ConnHdl	: Connection handle
 * @param StartHdl	: Start handle of the ATT group type to read
 * @param EndHdl	: End handle of the ATT group type to read
 * @param pUuid		: Contain the UUID type to be read
 * @return
 */
bool BtAttReadByGroupTypeRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl, BtUuid_t *pUuid)
{
	DEBUG_PRINTF("BtAttReadByGroupTypeRequest() called\r\n");

	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*) buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*) acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ;
	l2pdu->Att.ReadByGroupTypeReq.StartHdl = StartHdl;
	l2pdu->Att.ReadByGroupTypeReq.EndHdl = EndHdl;
	l2pdu->Hdr.Len = 5;// 1-byte OpCode + 2-byte StartHdl + 2-byte EndHdl
	if (pUuid->BaseIdx > 0)
	{
		BtUuidTo128(pUuid, l2pdu->Att.ReadByTypeReq.Uuid.Uuid128);
		l2pdu->Hdr.Len += 16;
	}
	else
	{
		if (pUuid->Type == BT_UUID_TYPE_32)
		{
			l2pdu->Att.ReadByTypeReq.Uuid.Uuid32 = pUuid->Uuid32;
			l2pdu->Hdr.Len += 4;
		}
		else
		{
			l2pdu->Att.ReadByTypeReq.Uuid.Uuid16 = pUuid->Uuid16;
			l2pdu->Hdr.Len += 2;
		}
	}
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*) acl,
			acl->Hdr.Len + sizeof(acl->Hdr));

//	DEBUG_PRINTF("BtAttReadByGroupTypeRequest : StartHdl %d, EndHdl %d, BaseIdx %d\r\n", StartHdl,
//			EndHdl, pUuid->BaseIdx);

	return true;
}

// Current UUID type to be search
BtUuid_t g_UuidType = {0, BT_UUID_TYPE_16, (uint16_t) BT_UUID_DECLARATIONS_CHARACTERISTIC };

bool BtAttStartReadByGroupTypeRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl, BtUuid_t *pUuid)
{
	if (pDev == NULL || pUuid == NULL)
		return false;

	memcpy((uint8_t*)&g_UuidType, (uint8_t*)pUuid, sizeof(BtUuid_t));
	return BtAttReadByGroupTypeRequest(pDev, ConnHdl, StartHdl, EndHdl, &g_UuidType);
}


