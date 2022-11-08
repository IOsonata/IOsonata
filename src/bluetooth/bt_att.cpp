/**-------------------------------------------------------------------------
@file	bt_att.cpp

@brief	Generic Bluetooth ATT protocol

Generic definitions for Bluetooth Attribute Protocol implementation

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2022

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
#include <inttypes.h>
#include <stddef.h>

#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "coredev/uart.h"

extern UART g_Uart;

void BtAttSendError(BtHciDevice_t *pDev, BtHciACLDataPacket_t *pAcl, uint16_t Hdl, uint8_t OpCode, uint8_t ErrCode)
{
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)pAcl->Data;
	BtAttErrorRsp_t *errsp = (BtAttErrorRsp_t*)&l2pdu->Att;

	errsp->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	errsp->ReqOpCode = OpCode;
	errsp->Hdl = Hdl;
	errsp->Error = ErrCode;

	l2pdu->Hdr.Len = sizeof(BtAttErrorRsp_t);
	pAcl->Hdr.Len = sizeof(BtAttErrorRsp_t) + sizeof(BtL2CapHdr_t);

	g_Uart.printf("Error %x\r\n", ErrCode);

	uint32_t n = pDev->SendData((uint8_t*)pAcl, pAcl->Hdr.Len + sizeof(pAcl->Hdr));
//	g_Uart.printf("n=%d\r\n", n);
}

void BtProcessAttData(BtHciDevice_t *pDev, uint16_t ConnHdl, BtL2CapPdu_t *pRcvPdu)
{
	uint8_t buf[BT_HCI_BUFFER_MAX_SIZE];
	BtHciACLDataPacket_t *acl = (BtHciACLDataPacket_t*)buf;
	BtL2CapPdu_t *l2pdu = (BtL2CapPdu_t*)acl->Data;

	g_Uart.printf("ATT OpCode %x, L2Cap len %d\n", pRcvPdu->Att.OpCode, pRcvPdu->Hdr.Len);

	acl->Hdr.ConnHdl = ConnHdl;
	acl->Hdr.PBFlag = BT_HCI_PBFLAG_COMPLETE_L2CAP_PDU;
	acl->Hdr.BCFlag = 0;

	l2pdu->Hdr = pRcvPdu->Hdr;


	switch (pRcvPdu->Att.OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			{
				BtAttExchgMtuReqRsp_t *req = (BtAttExchgMtuReqRsp_t*)&pRcvPdu->Att;

				g_Uart.printf("ATT_EXCHANGE_MTU_REQ:\r\n");
				g_Uart.printf("RxMtu %d\r\n", req->RxMtu);

				l2pdu->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t);

				BtAttExchgMtuReqRsp_t *att = (BtAttExchgMtuReqRsp_t*)&l2pdu->Att;

				att->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;
				att->RxMtu = 512;	// TODO: get real mtu

				acl->Hdr.Len = sizeof(BtAttExchgMtuReqRsp_t) + sizeof(BtL2CapHdr_t);

				uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
				g_Uart.printf("n=%d\r\n", n);
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:
			{
				g_Uart.printf("ATT_FIND_INFORMATION_REQ:\r\n");
				BtAttFindInfoReq_t *req = (BtAttFindInfoReq_t*)&pRcvPdu->Att;

				g_Uart.printf("sHdl: %x, eHdl: %x\r\n", req->StartHdl, req->EndHdl);

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					BtAttSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttFindInfoRsp_t *rsp = (BtAttFindInfoRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP;

				int l = 0;
#if 0
				BtGattListEntry_t list[50];
				uint16_t lasthdl = 0;

				int c = BtGattGetListHandle(req->StartHdl, req->EndHdl, list, 50, &lasthdl);

				if (c > 0)
				{
					if (list[0].TypeUuid.BaseIdx > 0)
					{
						uint8_t uuid128[16];

						BtUuidGetBase(list[0].TypeUuid.BaseIdx, uuid128);

						rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128;

						for (int i = 0; i < c; i++)
						{
							rsp->HdlUuid128[i].Hdl = list[i].Hdl;
							uuid128[12] = list[i].TypeUuid.Uuid & 0xff;
							uuid128[13] = list[i].TypeUuid.Uuid >> 8;
							memcpy(rsp->HdlUuid128[i].Uuid, uuid128, 16);

							g_Uart.printf("HDL : %d, ", rsp->HdlUuid128[i].Hdl);

							for (int j = 0; j < 16; j++)
							{
								g_Uart.printf("%02x ", rsp->HdlUuid128[i].Uuid[j]);
							}
							g_Uart.printf("\r\n");
						}

						l = c * sizeof(BtAttHdlUuid128_t);
					}
					else
					{
						rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;

						for (int i = 0; i < c; i++)
						{
							rsp->HdlUuid16[i].Hdl = list[i].Hdl;
							rsp->HdlUuid16[i].Uuid = list[i].TypeUuid.Uuid;

							g_Uart.printf("HDL : %d, Uuid16 : 0x%04x\r\n", rsp->HdlUuid16[i].Hdl, rsp->HdlUuid16[i].Uuid);

						}

						l = c * sizeof(BtAttHdlUuid16_t);
					}
//					rsp->Fmt = list[0].Uuid.BaseIdx > 0 ? BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128 : BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;
#else
					BtGattListEntry_t en;

					if (BtGattGetEntryHandle(req->StartHdl, &en))
					{
						if (en.TypeUuid.BaseIdx > 0)
						{
							uint8_t uuid128[16];

							BtUuidGetBase(en.TypeUuid.BaseIdx, uuid128);

							rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128;

						//	for (int i = 0; i < c; i++)
							{
								rsp->HdlUuid128[0].Hdl = en.Hdl;
								uuid128[12] = en.TypeUuid.Uuid & 0xff;
								uuid128[13] = en.TypeUuid.Uuid >> 8;
								memcpy(rsp->HdlUuid128[0].Uuid, uuid128, 16);

								g_Uart.printf("HDL : %d, ", rsp->HdlUuid128[0].Hdl);

								for (int j = 0; j < 16; j++)
								{
									g_Uart.printf("%02x ", rsp->HdlUuid128[0].Uuid[j]);
								}
								g_Uart.printf("\r\n");
							}

							l = sizeof(BtAttHdlUuid128_t);
						}
						else
						{
							rsp->Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;

							//for (int i = 0; i < c; i++)
							{
								rsp->HdlUuid16[0].Hdl = en.Hdl;
								rsp->HdlUuid16[0].Uuid = en.TypeUuid.Uuid;

								g_Uart.printf("HDL : %d, Uuid16 : 0x%04x\r\n", rsp->HdlUuid16[0].Hdl, rsp->HdlUuid16[0].Uuid);

							}

							l = sizeof(BtAttHdlUuid16_t);
						}

#endif

					l2pdu->Hdr.Len = 2 + l;
					acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

					g_Uart.printf("l2pdu len : %d, %d\r\n", l2pdu->Hdr.Len, acl->Hdr.Len);

					uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("n : %d\r\n", n);

					uint8_t*p = (uint8_t*)acl;
					g_Uart.printf("acl: ");
					for (int i = 0; i < acl->Hdr.Len + sizeof(acl->Hdr); i++)
					{
						g_Uart.printf("0x%02x ", p[i]);
					}
					g_Uart.printf("\r\n");

				}
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
			{
				g_Uart.printf("ATT_READ_BY_TYPE_REQ:\r\n");

				// Only the attributes with attribute handles between and including
				// the Starting Handle and the Ending Handle with the attribute type
				// that is the same as the Attribute Type given will be returned. To
				// search through all attributes, the starting handle shall be set to
				// 0x0001 and the ending handle shall be set to 0xFFFF.

				BtAttReadByTypeReq_t *req = (BtAttReadByTypeReq_t*)&pRcvPdu->Att;

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					BtAttSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttReadByTypeRsp_t *rsp = (BtAttReadByTypeRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;

				uint8_t *p = (uint8_t*)rsp->Data;

				g_Uart.printf("sHdl: %x, eHdl: %x, Type: %x\r\n", req->StartHdl, req->EndHdl, req->Uuid.Uuid16);
#if 0
				BtGattListEntry_t list[50];
				uint16_t lasthdl = 0;
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};

				int c = BtGattGetListUuid(&uid16, req->StartHdl, list, 50, &lasthdl);

				if (c > 0)
				{
					int l = 0;

					for (int i = 0; i < c; i++)
					{
						if (list[i].Hdl <= req->EndHdl)// && baseidx == list[i].Uuid.BaseIdx)
						{
							p[0] = list[i].Hdl & 0xFF;
							p[1] = list[i].Hdl >> 8;
							p +=2;

							rsp->Len = BtGattGetValue(&list[i], p) + 2;

							p += rsp->Len;
							l += rsp->Len;
						}
					}

					if (l > 0)
					{
						l2pdu->Hdr.Len = 2 + l;//rsp->Len * c;
						acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

						g_Uart.printf("rsp->Len : %d, hdr.len : %d, %d\r\n", rsp->Len, l2pdu->Hdr.Len, acl->Hdr.Len);

						uint32_t n = s_HciDevice.SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
						g_Uart.printf("n : %d\r\n", n);
						break;
					}
				}
#else
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};
				BtGattListEntry_t entry;

				if (BeGattFindEntryUuid(&uid16, req->StartHdl, req->EndHdl, &entry))
				{
					p[0] = entry.Hdl & 0xFF;
					p[1] = entry.Hdl >> 8;
					p +=2;
					rsp->Len = BtGattGetValue(&entry, p) + 2;
					g_Uart.printf("Att Val: ");

					for (int j = 0; j < rsp->Len; j++)
					{
						g_Uart.printf("%x ", rsp->Data[j]);
					}
					g_Uart.printf("\r\n");

					l2pdu->Hdr.Len = 2 + rsp->Len;//rsp->Len * c;
					acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

					g_Uart.printf("rsp->Len : %d, hdr.len : %d, %d\r\n", rsp->Len, l2pdu->Hdr.Len, acl->Hdr.Len);

					uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("n : %d\r\n", n);
					break;
				}
#endif
				BtAttSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ:
			{
				// The ATT_READ_REQ PDU is used to request the server to read the value
				// of an attribute and return its value in an ATT_READ_RSP PDU.
				BtAttReadReq_t *req = (BtAttReadReq_t*)&pRcvPdu->Att;
				BtAttReadRsp_t *rsp = (BtAttReadRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_RSP;

				g_Uart.printf("ATT_READ_REQ : Hdl = %d\r\n", req->Hdl);

				BtGattListEntry_t eg;

				if (BtGattGetEntryHandle(req->Hdl, &eg) == true)
				{
					int baseidx = eg.Uuid.BaseIdx;
					int l = BtGattGetValue(&eg, rsp->Data);

					g_Uart.printf("Data : ");
					for (int j= 0; j < l; j++)
					{
						g_Uart.printf("%x ", rsp->Data[j]);
					}
					g_Uart.printf("\r\n");
					/*
					BtUuid_t uid;
					uid.Type = BT_UUID_TYPE_16;

					if (baseidx > 0)
					{
						l = 16;
						uid.Type = BT_UUID_TYPE_128;
						BtUuidGetBase(baseidx, rsp->Data);

						rsp->Data[12] = eg.Uuid.Uuid & 0xFF;
						rsp->Data[13] = eg.Uuid.Uuid >> 8;
					}
					else
					{
						l = 2;
						rsp->Data[0] = eg.Uuid.Uuid & 0xFF;
						rsp->Data[1] = eg.Uuid.Uuid >> 8;
					}
*/
					l2pdu->Hdr.Len = sizeof(BtAttReadRsp_t) - 1 + l;
					acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

					uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
					g_Uart.printf("l : %d, %d, %d\r\n", l, l2pdu->Hdr.Len, acl->Hdr.Len);
				}
				else
				{
					BtAttSendError(pDev, acl, req->Hdl, BT_ATT_OPCODE_ATT_READ_REQ, BT_ATT_ERROR_INVALID_HANDLE);
/*
					BtAttErrorRsp_t *errsp = (BtAttErrorRsp_t*)&l2pdu->Att;

					errsp->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
					errsp->ReqOpCode = BT_ATT_OPCODE_ATT_READ_REQ;
					errsp->Hdl = 1;
					errsp->Error = BT_ATT_ERROR_ATT_NOT_FOUND;

					l2pdu->Hdr.Len = sizeof(BtAttErrorRsp_t);
					acl->Hdr.Len = sizeof(BtAttErrorRsp_t) + sizeof(BtL2CapHdr_t);*/
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:
			{
				BtAttBlobReq_t *req = (BtAttBlobReq_t*)&pRcvPdu->Att;

				g_Uart.printf("BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:\r\n");
				g_Uart.printf("Hdl: %x, offset: %x\r\n", req->Hdl, req->Offset);

				BtAttBlobRsp_t *rsp = (BtAttBlobRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_REQ_BLOB_RSP;
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:
			// The ATT_READ_MULTIPLE_REQ PDU is used to request the server to read
			// two or more values of a set of attributes and return their values in
			// an ATT_READ_MULTIPLE_RSP PDU. Only values that have a known fixed size
			// can be read, with the exception of the last value that can have a variable
			// length. The knowledge of whether attributes have a known fixed size is
			// defined in a higher layer specification.
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
			{
				BtAttReadByGroupTypeReq_t *req = (BtAttReadByGroupTypeReq_t*)&pRcvPdu->Att;

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					BtAttSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttReadByGroupTypeRsp_t *rsp = (BtAttReadByGroupTypeRsp_t*)&l2pdu->Att;

				rsp->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;

				g_Uart.printf("ATT_READ_BY_GROUP_TYPE_REQ:\r\n");
				g_Uart.printf("sHdl:%d, eHdl:%d, Uuid: ", req->StartHdl, req->EndHdl, req->Uid.Uuid16);

				if (pRcvPdu->Hdr.Len < 8)
				{
					g_Uart.printf("%x\r\n", req->Uid.Uuid16);

					BtGattListEntry_t list[20];
					uint16_t lasthdl = 0;
					BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uid.Uuid16};

					int c = BtGattGetListUuid(&uid16, req->StartHdl, list, 20, &lasthdl);

					uint8_t *p = (uint8_t*)rsp->Data;

					rsp->Len = 0;

					if (c > 0)
					{
						int baseidx = list[0].Uuid.BaseIdx;
						int l = 0;
						//BtUuid_t uid;
						//uid.Type = BT_UUID_TYPE_16;
//						rsp->Len = BtGattGetValueLen(list, );

						for (int i = 0; i < c && list[i].Hdl <= req->EndHdl && baseidx == list[i].Uuid.BaseIdx; i++)
						{
							BtAttHdlRange_t *hu = (BtAttHdlRange_t*)p;
							hu->StartHdl = list[i].Hdl;
							if ((i + 1) < c)
							{
								hu->EndHdl = list[i+1].Hdl - 1;
							}
							else
							{
								hu->EndHdl = lasthdl;
							}
							g_Uart.printf("sHdl: %d, eHdl: %d\r\n", hu->StartHdl, hu->EndHdl);

							p += sizeof(BtAttHdlRange_t);

							rsp->Len = BtGattGetValue(&list[i], p);

							g_Uart.printf("UUID: ");
							for (int j = 0; j < rsp->Len; j++)
							{
								g_Uart.printf("%x ", p[j]);
							}
							g_Uart.printf("\r\n");

							p += rsp->Len;
							l += 4 + rsp->Len;
						}

						if (l > 0)
						{
							rsp->Len += 4;

							l2pdu->Hdr.Len = sizeof(BtAttReadByGroupTypeRsp_t) - 1 + l;
							acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

							g_Uart.printf("rsp->Len : %d, %d, %d, %d\r\n", rsp->Len, l, l2pdu->Hdr.Len, acl->Hdr.Len);
							uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
							g_Uart.printf("n=%d\r\n", n);
							break;
						}
					}

					g_Uart.printf("List not found\r\n");

					BtAttSendError(pDev, acl, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
				else
				{
					g_Uart.printf("UUID128 : ", req->Uid.Uuid16);
					for (int i = 0; i <16; i++)
					{
						g_Uart.printf("%x \r\n", req->Uid.Uuid128[i]);
					}
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_WRITE_REQ:
			{
				BtAttWriteReq_t *req = (BtAttWriteReq_t*)&pRcvPdu->Att;

				g_Uart.printf("BT_ATT_OPCODE_ATT_WRITE_REQ:\r\n");
				g_Uart.printf("Hdl: %x\r\n", req->Hdl);

				if (req->Hdl < 1)
				{
					BtAttSendError(pDev, acl, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}
				BtAttWriteRsp_t *rsp = (BtAttWriteRsp_t*)&l2pdu->Att;
				size_t l = pRcvPdu->Hdr.Len;

				l = BtGattWriteValue(req->Hdl, req->Data, l);

				rsp->OpCode = BT_ATT_OPCODE_ATT_WRITE_RSP;

				l2pdu->Hdr.Len = 1;
				acl->Hdr.Len = l2pdu->Hdr.Len + sizeof(BtL2CapHdr_t);

				g_Uart.printf("l2pdu->Hdr.Len %d, acl->Hdr.Len %d\r\n", l2pdu->Hdr.Len, acl->Hdr.Len);
				uint32_t n = pDev->SendData((uint8_t*)acl, acl->Hdr.Len + sizeof(acl->Hdr));
				g_Uart.printf("n=%d\r\n", n);
			}
			break;
		case BT_ATT_OPCODE_ATT_CMD:
			{
				BtAttSignedWriteCmd_t *req = (BtAttSignedWriteCmd_t*)&pRcvPdu->Att;

				g_Uart.printf("ATT_CMD:\r\n");
				g_Uart.printf("Hdl: %x, data len: %d\r\n", req->Hdl, pRcvPdu->Hdr.Len -  3);

				size_t l = pRcvPdu->Hdr.Len;

				l = BtGattWriteValue(req->Hdl, req->Data, l - 3);
			}
			break;
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:
			break;
		case BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD:
			break;
	}
}
