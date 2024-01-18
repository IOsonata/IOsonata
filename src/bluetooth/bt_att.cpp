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
#include <memory.h>

#include "istddef.h"
#include "bluetooth/bt_hci.h"
#include "bluetooth/bt_l2cap.h"
#include "bluetooth/bt_att.h"
//#include "bluetooth/bt_gatt.h"
//#include "coredev/uart.h"

//extern UART g_Uart;

/// Default Attribute database mem size
/// User App can re-define a bigger or smaller value to fit use case
#ifndef BT_ATT_DB_MEMSIZE
#define BT_ATT_DB_MEMSIZE			2048	//!< Mem size in byte
#endif

static uint16_t s_AttMtu = BT_ATT_MTU_MIN;

alignas(4) __attribute__((weak)) uint8_t s_BtAttDBMem[BT_ATT_DB_MEMSIZE];
static size_t s_BtAttDBMemSize = sizeof(s_BtAttDBMem);
static uint32_t s_BtAttDBMemEnd = (uint32_t)s_BtAttDBMem + s_BtAttDBMemSize;
static BtAttDBEntry_t * const s_pBtAttDbEntryFirst = (BtAttDBEntry_t *)s_BtAttDBMem;
static BtAttDBEntry_t *s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)s_BtAttDBMem;
static uint16_t s_LastHdl = 0;

uint16_t BtAttSetMtu(uint16_t Mtu)
{
	if (Mtu >= BT_ATT_MTU_MIN && Mtu <= BT_ATT_MTU_MAX)
	{
		s_AttMtu = Mtu;
	}

	return s_AttMtu;
}

uint16_t BtAttGetMtu()
{
	return s_AttMtu;
}
/*
void BtAttSetHandler(AttReadValFct_t ReadFct, AttWriteValFct_t WriteFct)
{
	s_AttReadValue = ReadFct;
	s_AttWriteValue = WriteFct;
}
*/
void BtAttDBInit(size_t MemSize)
{
	s_BtAttDBMemSize = MemSize;
	memset(s_BtAttDBMem, 0, s_BtAttDBMemSize);
	s_BtAttDBMemEnd = (uint32_t)s_BtAttDBMem + s_BtAttDBMemSize;
}

BtAttDBEntry_t * const BtAttDBAddEntry(BtUuid16_t *pUuid, int MaxDataLen)//, void *pData, int DataLen)
{
	BtAttDBEntry_t *entry = s_pBtAttDbEntryEnd;

	uint32_t l = sizeof(BtAttDBEntry_t) + MaxDataLen;

	l = (l + 3) & 0xFFFFFFFC;
	if ((uint32_t)entry + l > s_BtAttDBMemEnd)
	{
		//g_Uart.printf("Out mem. Required %d, Reserved : %d\r\n", ((uint32_t)entry + l) - (uint32_t)s_BtAttDBMem, s_BtAttDBMemEnd - (uint32_t)s_BtAttDBMem);
		return nullptr;
	}

	entry->TypeUuid = *pUuid;
	entry->Hdl = ++s_LastHdl;
	entry->DataLen = MaxDataLen;

	s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)((uint8_t*)entry + l);
	s_pBtAttDbEntryEnd->pNext = nullptr;
	s_pBtAttDbEntryEnd->pPrev = entry;
	entry->pNext = s_pBtAttDbEntryEnd;

//	g_Uart.printf("Entry %p, %x\r\n", entry, s_BtAttDBMemEnd);
	return entry;
}

BtAttDBEntry_t * const BtAttDBFindHandle(uint16_t Hdl)
{
	if (Hdl >= s_LastHdl)
	{
		return nullptr;
	}

	BtAttDBEntry_t *p = (BtAttDBEntry_t *)s_pBtAttDbEntryFirst;

	do {
		if (p->Hdl == Hdl)
		{
			return p;
		}

		p = p->pNext;
	} while (p != s_pBtAttDbEntryEnd);

	return nullptr;
}

BtAttDBEntry_t * const BtAttDBFindUuid(BtAttDBEntry_t *pStart, BtUuid16_t *pUuid)
{
	BtAttDBEntry_t *p = pStart;
	if (p == nullptr)
	{
		p = (BtAttDBEntry_t*)s_BtAttDBMem;
	}
	do {
		if (memcmp(&p->TypeUuid, pUuid, sizeof(BtUuid16_t)) == 0)
		{
			return p;
		}

		p = p->pNext;
	} while (p != s_pBtAttDbEntryEnd);

	return nullptr;
}

BtAttDBEntry_t * const BtAttDBFindUuidRange(BtUuid16_t *pUuid, uint16_t HdlStart, uint16_t HdlEnd)
{
	BtAttDBEntry_t *p = s_pBtAttDbEntryFirst;

	do {
		if (memcmp(&p->TypeUuid, pUuid, sizeof(BtUuid16_t)) == 0)
		{
			if (p->Hdl >= HdlStart && p->Hdl <= HdlEnd)
			{
				return p;
			}
		}

		p = p->pNext;
	} while (p != s_pBtAttDbEntryEnd);

	return nullptr;
}

BtAttDBEntry_t * const BtAttDBFindHdlRange(BtUuid16_t *pUuid, uint16_t *pHdlStart, uint16_t *pHdlEnd)
{
	BtAttDBEntry_t *first = BtAttDBFindUuidRange(pUuid, *pHdlStart, *pHdlEnd);

	if (first)
	{
		*pHdlStart = first->Hdl;
		BtAttDBEntry_t *p = BtAttDBFindUuid(first->pNext, pUuid);
		if (p)
		{
			*pHdlEnd = p->pPrev->Hdl;
		}
		else
		{
			*pHdlEnd = s_pBtAttDbEntryEnd->pPrev->Hdl;//0xFFFF;
		}
	}

	return first;
}

size_t BtAttReadValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pBuff, uint16_t Len)
{
	if (pBuff == nullptr)
	{
		return 0;
	}

	uint16_t len = 0;

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		//g_Uart.printf("BtAttReadValue : %x\r\n", pEntry->TypeUuid.Uuid);

		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
				{
					BtAttSrvcDeclar_t *p = (BtAttSrvcDeclar_t*)pEntry->Data;

					if (p->Uuid.BaseIdx > 0)
					{
						BtUuidGetBase(p->Uuid.BaseIdx, pBuff);

						pBuff[12] = p->Uuid.Uuid16 & 0xFF;
						pBuff[13] = p->Uuid.Uuid16 >> 8;

						len = 16;
					}
					else
					{
						pBuff[0] = p->Uuid.Uuid16 & 0xFF;
						pBuff[1] = p->Uuid.Uuid16 >> 8;

						len = 2;
					}
				}
				break;
			case BT_UUID_DECLARATIONS_INCLUDE:
				{
					BtAttSrvcInclude_t *p = (BtAttSrvcInclude_t*)pEntry->Data;
					len = p->SrvcUuid.BaseIdx > 0 ? 20 : 6;
				}

				break;
			case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				{
					BtAttCharDeclar_t *p = (BtAttCharDeclar_t*)pEntry->Data;
					//len = pEntry->DataLen;
					//memcpy(pBuff, pEntry->Data, len);
					pBuff[0] = p->pChar->Property;
					pBuff[1] = p->pChar->ValHdl & 0xFF;
					pBuff[2] = (p->pChar->ValHdl >> 8)& 0xFF;
					BtUuidVal_t *u = (BtUuidVal_t*)&pBuff[3];
					if (p->Uuid.BaseIdx > 0)
					{
						BtUuidGetBase(p->Uuid.BaseIdx, u->Uuid128);

						u->Uuid128[12] = p->Uuid.Uuid16 & 0xFF;
						u->Uuid128[13] = p->Uuid.Uuid16 >> 8;

						len = 19;
					}
					else
					{
						u->Uuid16 = p->Uuid.Uuid16;
						len = 5;
					}
				}
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
//				if (pEntry->pVal)
				{
					//g_Uart.printf("BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION\r\n");
					BtDescCharUserDesc_t *p = (BtDescCharUserDesc_t*)pEntry->Data;
					strncpy((char*)pBuff, p->pChar->pDesc, Len);
					len = strlen((char*)pBuff);
					//g_Uart.printf("%d : %s\r\n", len, pBuff);
					len = min(len, Len);
				}
				break;
			case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				{
					BtDescClientCharConfig_t *d = (BtDescClientCharConfig_t*)pEntry->Data;

					*(uint16_t*)pBuff = d->CccVal;
					len = 2;
				}
				break;
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				{
					BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;
					//g_Uart.printf("UUID unknown : %x, %d\r\n", pEntry->TypeUuid.Uuid, p->pChar->ValueLen);

					size_t l = p->pChar->ValueLen;//min(p->pChar->ValueLen - Offset, BtAttGetMtu());
					memcpy(pBuff, p->Data + Offset, l);
					len = l;
				}

		}
	}
	else
	{
		BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;
		//g_Uart.printf("Read Req UUID custom: %d: %x, %d\r\n", pEntry->TypeUuid.BaseIdx, pEntry->TypeUuid.Uuid, p->pChar->ValueLen);

		size_t l = p->pChar->ValueLen;//min(p->pChar->ValueLen - Offset, BtAttGetMtu());
		memcpy(pBuff, p->Data + Offset, l);
		len = l;
	}

	return len;
}

//size_t BtGattWriteValue(uint16_t Hdl, uint8_t *pBuff, size_t Len)
size_t BtAttWriteValue(BtAttDBEntry_t *pEntry, uint16_t Offset, uint8_t *pData, uint16_t Len)
{
	size_t len = 0;
	//BtAttDBEntry_t *entry = BtAttDBFindHandle(Hdl);//&s_BtGattEntryTbl[Hdl - 1];
//g_Uart.printf("BtAttWriteValue : uuid %x, %x\r\n", pEntry->TypeUuid.Uuid, pEntry->Hdl);

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
				break;
			case BT_UUID_DECLARATIONS_INCLUDE:
				break;
			case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				break;
			case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				{
					//g_Uart.printf("BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION\r\n");

					BtDescClientCharConfig_t *p = (BtDescClientCharConfig_t*)pEntry->Data;

					p->CccVal = *(uint16_t*)pData;
					p->pChar->bNotify = p->CccVal & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION ? true: false;
					if (p->pChar->SetNotifCB)
					{
						p->pChar->SetNotifCB(p->pChar, p->pChar->bNotify);
					}
//					g_Uart.printf("CccdHdl : %x\r\n", p->pChar->CccdHdl);
				}
				break;
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				{
					BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;

					p->pChar->ValueLen = min(Len, p->pChar->MaxDataLen);// - sizeof(BtGattCharValue_t));
					memcpy(p->Data, pData, p->pChar->ValueLen);

					if (p->pChar->WrCB)
					{
	//					len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
						p->pChar->WrCB(p->pChar, pData, 0, Len);
					}
				}

		}
	}
	else
	{
		BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;

		p->pChar->ValueLen = min(Len,  p->pChar->MaxDataLen);//- sizeof(BtGattCharValue_t));
		memcpy(p->Data, pData, p->pChar->ValueLen);

		if (p->pChar->WrCB)
		{
//					len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
			p->pChar->WrCB(p->pChar, pData, 0, Len);
		}
	}

	return len;
}

uint32_t BtAttError(BtAttReqRsp_t * const pRspAtt, uint16_t Hdl, uint8_t OpCode, uint8_t ErrCode)
{
	pRspAtt->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	pRspAtt->ErrorRsp.ReqOpCode = OpCode;
	pRspAtt->ErrorRsp.Hdl = Hdl;
	pRspAtt->ErrorRsp.Error = ErrCode;

	return sizeof(BtAttErrorRsp_t) + 1;
}

void BtAttProcessRsp(uint16_t ConnHdl, BtAttReqRsp_t * const pRspAtt, int RspLen)
{
	//g_Uart.printf("BtAttProcessRsp : %x\r\n", pRspAtt->OpCode);

	switch(pRspAtt->OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP:
			{
				uint16_t mtu = min(BtAttGetMtu(), pRspAtt->ExchgMtuReqRsp.RxMtu);
	//			g_Uart.printf("BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP : %d %d\r\n", pRspAtt->ExchgMtuReqRsp.RxMtu, mtu);
				BtAttSetMtu(mtu);
			}
			break;
	}
}

uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReqAtt, int ReqLen, BtAttReqRsp_t * const pRspAtt)
{
	uint32_t retval = 0;

	//g_Uart.printf("ATT OpCode %x, L2Cap len %d\n", pReqAtt->OpCode, ReqLen);

	switch (pReqAtt->OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			{
				BtAttExchgMtuReqRsp_t *req = (BtAttExchgMtuReqRsp_t*)&pReqAtt->ExchgMtuReqRsp;

				//g_Uart.printf("ATT_EXCHANGE_MTU_REQ:\r\n");
				//g_Uart.printf("RxMtu %d %d\r\n", pReqAtt->ExchgMtuReqRsp.RxMtu, s_AttMtu);

				if (pReqAtt->ExchgMtuReqRsp.RxMtu < BT_ATT_MTU_MIN)
				{
					retval = BtAttError(pRspAtt, ConnHdl, BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ, BT_ATT_ERROR_INVALID_ATT_VALUE);
					break;
				}
				retval = sizeof(BtAttExchgMtuReqRsp_t) + 1;
				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;
				pRspAtt->ExchgMtuReqRsp.RxMtu = 247;//BtAttSetMtu(pReqAtt->ExchgMtuReqRsp.RxMtu);

				//g_Uart.printf("MTU : %d\r\n", s_AttMtu);
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:
			{
				BtAttFindInfoReq_t *req = (BtAttFindInfoReq_t*)&pReqAtt->FindInfoReq;

				if (/*req->StartHdl < 1 || req->EndHdl < 1 ||*/ req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP;

				int l = 0;
				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->StartHdl);

				if (entry)
				{
					if (entry->TypeUuid.BaseIdx > 0)
					{
						uint8_t uuid128[16];

						BtUuidGetBase(entry->TypeUuid.BaseIdx, uuid128);

						pRspAtt->FindInfoRsp.Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID128;

					//	for (int i = 0; i < c; i++)
						{
							pRspAtt->FindInfoRsp.HdlUuid128[0].Hdl = entry->Hdl;
							uuid128[12] = entry->TypeUuid.Uuid & 0xff;
							uuid128[13] = entry->TypeUuid.Uuid >> 8;
							memcpy(pRspAtt->FindInfoRsp.HdlUuid128[0].Uuid, uuid128, 16);

//								g_Uart.printf("HDL : %d, ", rsp->HdlUuid128[0].Hdl);
//
//								for (int j = 0; j < 16; j++)
//								{
//									g_Uart.printf("%02x ", rsp->HdlUuid128[0].Uuid[j]);
//								}
//								g_Uart.printf("\r\n");
						}

						l = sizeof(BtAttHdlUuid128_t);
					}
					else
					{
						pRspAtt->FindInfoRsp.Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;

						//for (int i = 0; i < c; i++)
						{
							pRspAtt->FindInfoRsp.HdlUuid16[0].Hdl = entry->Hdl;
							pRspAtt->FindInfoRsp.HdlUuid16[0].Uuid = entry->TypeUuid.Uuid;

//								g_Uart.printf("HDL : %d, Uuid16 : 0x%04x\r\n", rsp->HdlUuid16[0].Hdl, rsp->HdlUuid16[0].Uuid);

						}

						l = sizeof(BtAttHdlUuid16_t);
					}

					retval = 2 + l;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:
			{
				//g_Uart.printf("BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:\r\n");
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
			{
				//g_Uart.printf("ATT_READ_BY_TYPE_REQ: %x - %x\r\n", pReqAtt->ReadByTypeReq.StartHdl, pReqAtt->ReadByTypeReq.EndHdl);

				// Only the attributes with attribute handles between and including
				// the Starting Handle and the Ending Handle with the attribute type
				// that is the same as the Attribute Type given will be returned. To
				// search through all attributes, the starting handle shall be set to
				// 0x0001 and the ending handle shall be set to 0xFFFF.

				BtAttReadByTypeReq_t *req = (BtAttReadByTypeReq_t*)&pReqAtt->ReadByTypeReq;

				if (req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

//				BtAttReadByTypeRsp_t *rsp = (BtAttReadByTypeRsp_t*)&l2pdu->Att;
//				BtAtt_t *rsp = (BtAtt_t*)&l2pdu->Att;

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;

				uint8_t *p = (uint8_t*)pRspAtt->ReadByTypeRsp.Data;

//				g_Uart.printf("sHdl: %x, eHdl: %x, Type: %x\r\n", req->StartHdl, req->EndHdl, req->Uuid.Uuid16);
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};

				BtAttDBEntry_t *entry = BtAttDBFindUuidRange(&uid16, req->StartHdl, req->EndHdl);
#if 0
				if (entry)
				{
					p[0] = entry->Hdl & 0xFF;
					p[1] = entry->Hdl >> 8;
					p +=2;
					pRspAtt->ReadByTypeRsp.Len = BtAttReadValue(entry, 0, p, 20) + 2;

					retval = 2 + pRspAtt->ReadByTypeRsp.Len;//rsp->Len * c;
					break;
				}
				retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);

#else
				int l = 0;
				pRspAtt->ReadByTypeRsp.Len = 0;

				while (entry && l < s_AttMtu)
				{
					if (entry->Hdl < req->StartHdl || entry->Hdl > req->EndHdl)
					{
						break;
					}

					p[0] = entry->Hdl & 0xFF;
					p[1] = entry->Hdl >> 8;
					p += 2;

					int cnt = BtAttReadValue(entry, 0, p, s_AttMtu) + 2;

					if (pRspAtt->ReadByTypeRsp.Len == 0)
					{
						pRspAtt->ReadByTypeRsp.Len = cnt;
					}
					else if (cnt != pRspAtt->ReadByTypeRsp.Len)
					{
						break;
					}

					l += pRspAtt->ReadByTypeRsp.Len;
					p += pRspAtt->ReadByTypeRsp.Len - 2;
					req->StartHdl = entry->pNext->Hdl;

					entry = BtAttDBFindUuidRange(&uid16, req->StartHdl, req->EndHdl);
				}
				if (l > 0)
				{
					retval = l + 2;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
#endif
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ:
			{
				// The ATT_READ_REQ PDU is used to request the server to read the value
				// of an attribute and return its value in an ATT_READ_RSP PDU.
				BtAttReadReq_t *req = (BtAttReadReq_t*)&pReqAtt->ReadReq;

				//g_Uart.printf("BT_ATT_OPCODE_ATT_READ_REQ: %d\r\n", req->Hdl);

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_RSP;

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
					retval = BtAttReadValue(entry, 0, pRspAtt->ReadRsp.Data, 247)  + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_READ_REQ, BT_ATT_ERROR_INVALID_HANDLE);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:
			{
				//BtAttReadBlobReq_t *req = (BtAttReadBlobReq_t*)&pReqAtt->BlobReq;

				//g_Uart.printf("BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ:\r\n");
//				g_Uart.printf("Hdl: %x, offset: %x\r\n", req->Hdl, req->Offset);

				//BtAtt_t *rsp = (BtAtt_t*)&l2pdu->Att;

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_REQ_BLOB_RSP;
				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadBlobReq.Hdl);

				if (entry)
				{
					retval = BtAttReadValue(entry, pReqAtt->ReadBlobReq.Offset, pRspAtt->ReadBlobRsp.Data, 247) + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, pReqAtt->ReadBlobReq.Hdl, BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ, BT_ATT_ERROR_INVALID_HANDLE);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:
			{
				// The ATT_READ_MULTIPLE_REQ PDU is used to request the server to read
				// two or more values of a set of attributes and return their values in
				// an ATT_READ_MULTIPLE_RSP PDU. Only values that have a known fixed size
				// can be read, with the exception of the last value that can have a variable
				// length. The knowledge of whether attributes have a known fixed size is
				// defined in a higher layer specification.
				int nhdl = (ReqLen - 1) >> 1;
				uint8_t *p = pRspAtt->ReadMultipleRsp.Data;
				retval = 1;
				for (int i = 0; i < nhdl && retval < 20; i++)
				{
					BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadMultipleReq.Hdl[i]);
					if (entry == nullptr)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleReq.Hdl[i], BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
						break;
					}
					int l = BtAttReadValue(entry, 0, p, 247);
					p += l;
					retval += l;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
			{
				BtAttReadByGroupTypeReq_t *req = (BtAttReadByGroupTypeReq_t*)&pReqAtt->ReadByGroupTypeReq;

				if (req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;

//				g_Uart.printf("ATT_READ_BY_GROUP_TYPE_REQ:\r\n");
//				g_Uart.printf("sHdl:%x, eHdl:%x, Uuid: %x\r\n", req->StartHdl, req->EndHdl, req->Uuid.Uuid16);

				uint8_t *p = (uint8_t*)pRspAtt->ReadByGroupTypeRsp.Data;
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};
				BtAttHdlRange_t *hu = (BtAttHdlRange_t*)p;
				int l = 0;

				pRspAtt->ReadByGroupTypeRsp.Len = 0;

//					g_Uart.printf("%x\r\n", req->Uid.Uuid16);

				hu->StartHdl = req->StartHdl;
				hu->EndHdl = req->EndHdl;
				BtAttDBEntry_t *entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);

				int baseidx = entry->TypeUuid.BaseIdx;

				if (entry)
				{
					baseidx = entry->TypeUuid.BaseIdx;
				}

				pRspAtt->ReadByGroupTypeRsp.Len = 0;

				while (entry && l < s_AttMtu)
				{
					if (entry->Hdl >= req->StartHdl && entry->Hdl <= req->EndHdl && baseidx == entry->TypeUuid.BaseIdx)
					{
//						g_Uart.printf("shdl : %x, ehdl : %x\r\n", hu->StartHdl, hu->EndHdl);
						p += sizeof(BtAttHdlRange_t);
						int cnt = BtAttReadValue(entry, 0, p, 247);
						if (pRspAtt->ReadByGroupTypeRsp.Len == 0)
						{
							pRspAtt->ReadByGroupTypeRsp.Len = cnt;
						}
						else if (cnt != pRspAtt->ReadByGroupTypeRsp.Len)
						{
							break;
						}
/*
						g_Uart.printf("Val Len : %d\r\n", pRspAtt->ReadByGroupTypeRsp.Len);
						for (int i=0; i< pRspAtt->ReadByGroupTypeRsp.Len; i++)
						{
							g_Uart.printf("%02x ", p[i]);
						}
						g_Uart.printf("\r\n");
*/
						p += pRspAtt->ReadByGroupTypeRsp.Len;
						l += 4 + pRspAtt->ReadByGroupTypeRsp.Len;
						((BtAttHdlRange_t*)p)->StartHdl = hu->EndHdl;
						hu = (BtAttHdlRange_t*)p;
						hu->EndHdl = req->EndHdl;
//						g_Uart.printf("Len : %d\r\n", l);
					}
					else
					{
						break;
					}
					entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);
				}

				if (l > 0)
				{
					pRspAtt->ReadByGroupTypeRsp.Len += 4;

					retval = l + 2;
//					g_Uart.printf("retval %d\r\n", retval);
					break;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_WRITE_REQ:	// Write With Response
			{
				BtAttWriteReq_t *req = (BtAttWriteReq_t*)&pReqAtt->WriteReq;

				//g_Uart.printf("BT_ATT_OPCODE_ATT_WRITE_REQ:\r\n");
				//g_Uart.printf("Hdl: %x\r\n", req->Hdl);

				if (req->Hdl < 1)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
				//BtAttWriteRsp_t *rsp = (BtAttWriteRsp_t*)&l2pdu->Att;
					size_t l = ReqLen;

					l = BtAttWriteValue(entry, 0, req->Data, l);

					pRspAtt->OpCode = BT_ATT_OPCODE_ATT_WRITE_RSP;

					retval = 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);

				}
			}
			break;
		case BT_ATT_OPCODE_ATT_CMD:		// Write without response
			{
				BtAttSignedWriteCmd_t *req = (BtAttSignedWriteCmd_t*)&pReqAtt->SignedWriteCmd;

//				g_Uart.printf("ATT_CMD:\r\n");
//				g_Uart.printf("Hdl: %x, data len: %d\r\n", req->Hdl, pRcvPdu->Hdr.Len -  3);

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
					size_t l = ReqLen;

					l = BtAttWriteValue(entry, 0, req->Data, l - 3);

					retval = 0;
				}
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
		default:
			//g_Uart.printf("OpCode : %x\r\n", pReqAtt->OpCode);
			BtAttProcessRsp(ConnHdl, pReqAtt, ReqLen);
	}

	return retval;
}

