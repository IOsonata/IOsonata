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
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"

#include "coredev/uart.h"

extern UART g_Uart;

bool BtAttExchangeMtuRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t Mtu)
{
	uint8_t buff[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buff;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ;
	l2pdu->Att.ExchgMtuReqRsp.RxMtu = BtAttGetMtu();
	l2pdu->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t) + 1;
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	g_Uart.printf("BtAttExchangeMtuRequest : %d, %d\r\n", n, Mtu);

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

	g_Uart.printf("BtAttFindInformationRequest : %d, %d\r\n", StartHdl, EndHdl);

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

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	g_Uart.printf("BtAttFindByTypeValueRequest : %d, %d %d\r\n", StartHdl, EndHdl, AttType);

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

	l2pdu->Att.OpCode = BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ;
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
		l2pdu->Att.ReadByTypeReq.Uuid.Uuid16 = pUuid->Uuid16;
		l2pdu->Hdr.Len + 2;
	}
	l2pdu->Hdr.Cid = BT_L2CAP_CID_ATT;

	acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

	uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));

	g_Uart.printf("BtAttReadByTypeRequest : %d, %d %d\r\n", StartHdl, EndHdl, pUuid->BaseIdx);

	return true;
}
