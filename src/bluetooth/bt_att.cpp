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

#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_uuid.h"
#include "bluetooth/bt_gatt.h"

/******** For DEBUG ************/
#define UART_DEBUG_ENABLE

#ifdef UART_DEBUG_ENABLE
#include "coredev/uart.h"
extern UART g_Uart;
#define DEBUG_PRINTF(...)		g_Uart.printf(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

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

extern BtDev_t g_BtDevSdc;
#pragma pack(push,1)
typedef struct {
	uint8_t	SrvIdx;		// Current Service Idx
	uint8_t Hdl;		// Current handle
	uint8_t CharIdx;	// Current Characteristic Idx
} CurParseInf_t;
#pragma pack(pop)

CurParseInf_t CurIdx = {
	.SrvIdx = 0,
	.Hdl = 0,
	.CharIdx = 0,
};

// Current UUID type to be search
BtUuid_t s_UuidType = {0, BT_UUID_TYPE_16, (uint16_t) BT_UUID_DECLARATIONS_CHARACTERISTIC };

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
		//DEBUG_PRINTF("Out mem. Required %d, Reserved : %d\r\n", ((uint32_t)entry + l) - (uint32_t)s_BtAttDBMem, s_BtAttDBMemEnd - (uint32_t)s_BtAttDBMem);
		return nullptr;
	}

	entry->TypeUuid = *pUuid;
	entry->Hdl = ++s_LastHdl;
	entry->DataLen = MaxDataLen;

	s_pBtAttDbEntryEnd = (BtAttDBEntry_t*)((uint8_t*)entry + l);
	s_pBtAttDbEntryEnd->pNext = nullptr;
	s_pBtAttDbEntryEnd->pPrev = entry;
	entry->pNext = s_pBtAttDbEntryEnd;

//	DEBUG_PRINTF("Entry %p, %x\r\n", entry, s_BtAttDBMemEnd);
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
		//DEBUG_PRINTF("BtAttReadValue : %x, %d\r\n", pEntry->TypeUuid.Uuid, Len);

		switch (pEntry->TypeUuid.Uuid)
		{
		case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
		case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
		{
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_PRIMARY_SERVICE (0x2800)/ SECONDARY_SERVICE (0x2801)\r\n");
			BtAttSrvcDeclar_t *p = (BtAttSrvcDeclar_t*) pEntry->Data;

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
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_INCLUDE (0x2802)\r\n");
			BtAttSrvcInclude_t *p = (BtAttSrvcInclude_t*) pEntry->Data;
			len = p->SrvcUuid.BaseIdx > 0 ? 20 : 6;
		}

			break;
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		{
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_CHARACTERISTIC (0x2803)\r\n");
			BtAttCharDeclar_t *p = (BtAttCharDeclar_t*) pEntry->Data;
			//len = pEntry->DataLen;
			//memcpy(pBuff, pEntry->Data, len);
			pBuff[0] = p->pChar->Property;
			pBuff[1] = p->pChar->ValHdl & 0xFF;
			pBuff[2] = (p->pChar->ValHdl >> 8) & 0xFF;
			BtUuidVal_t *u = (BtUuidVal_t*) &pBuff[3];
			if (p->Uuid.BaseIdx > 0)
			{
				BtUuidGetBase(p->Uuid.BaseIdx, u->Uuid128);

				u->Uuid128[12] = p->Uuid.Uuid16 & 0xFF;
				u->Uuid128[13] = p->Uuid.Uuid16 >> 8;
				len = 19;
				DEBUG_PRINTF("UUID16 0x%X ,", p->Uuid.Uuid16);
				DEBUG_PRINTF("Base UUID128 (hex) = ");
				for (int i = 0; i < 16; i++)
					DEBUG_PRINTF("%X ", u->Uuid128[i]);
				DEBUG_PRINTF("\r\n");
			}
			else
			{
				u->Uuid16 = p->Uuid.Uuid16;
				len = 5;
				DEBUG_PRINTF("UUID16 = 0x%X \r\n", u->Uuid16);
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
			DEBUG_PRINTF("BT_UUID_DECLARATIONS_CHARACTERISTIC (0x2900)\r\n");
			break;
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
//				if (pEntry->pVal)
		{
			DEBUG_PRINTF("BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION (0x2901)\r\n");
			BtDescCharUserDesc_t *p = (BtDescCharUserDesc_t*) pEntry->Data;
			strncpy((char*) pBuff, p->pChar->pDesc, Len);
			len = strlen((char*) pBuff);
			//DEBUG_PRINTF("%d : %s\r\n", len, pBuff);
			len = min(len, Len);
		}
			break;
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		{
			DEBUG_PRINTF("BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION (0x2902)\r\n");
			BtDescClientCharConfig_t *d =
					(BtDescClientCharConfig_t*) pEntry->Data;

			*(uint16_t*) pBuff = d->CccVal;
			DEBUG_PRINTF("Hdl = %d, CCC val = %d\r\n", pEntry->Hdl, d->CccVal);
			len = 2;
		}
			break;
		case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
			break;
		default:
		{
			BtAttCharValue_t *p = (BtAttCharValue_t*) pEntry->Data;
			DEBUG_PRINTF("UUID unknown : Type 0x%x, Len %d\r\n", pEntry->TypeUuid.Uuid, p->pChar->ValueLen);

			size_t l = p->pChar->ValueLen;//min(p->pChar->ValueLen - Offset, BtAttGetMtu());
			memcpy(pBuff, p->Data + Offset, l);
			len = l;
		}

		}
	}
	else
	{
		BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;
		DEBUG_PRINTF("Read Req UUID custom: %d: %x, %d\r\n", pEntry->TypeUuid.BaseIdx, pEntry->TypeUuid.Uuid, p->pChar->ValueLen);

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
//DEBUG_PRINTF("BtAttWriteValue : uuid %x, %x\r\n", pEntry->TypeUuid.Uuid, pEntry->Hdl);

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
					//DEBUG_PRINTF("BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION\r\n");

					BtDescClientCharConfig_t *p = (BtDescClientCharConfig_t*)pEntry->Data;

					p->CccVal = *(uint16_t*)pData;
					p->pChar->bNotify = p->CccVal & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION ? true: false;
					if (p->pChar->SetNotifCB)
					{
						p->pChar->SetNotifCB(p->pChar, p->pChar->bNotify);
					}
//					DEBUG_PRINTF("CccdHdl : %x\r\n", p->pChar->CccdHdl);
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

bool BtAttStartReadByGroupTypeRequest(BtHciDevice_t * const pDev, uint16_t ConnHdl, uint16_t StartHdl, uint16_t EndHdl, BtUuid_t *pUuid)
{
	if (pDev == NULL || pUuid == NULL)
		return false;

	memcpy((uint8_t*)&s_UuidType, (uint8_t*)pUuid, sizeof(BtUuid_t));
	return BtAttReadByGroupTypeRequest(pDev, ConnHdl, StartHdl, EndHdl, &s_UuidType);
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
//	DEBUG_PRINTF("BtAttProcessRsp: Opcode 0x%x, RspLen = %d \r\n",
//			pRspAtt->OpCode, RspLen);

	switch (pRspAtt->OpCode)
	{
	case BT_ATT_OPCODE_ATT_ERROR_RSP:
	{
		DEBUG_PRINTF(
				"BT_ATT_OPCODE_ATT_ERROR_RSP (0x01): OpCode 0x%x, ErrCode 0x%x \r\n",
				pRspAtt->ErrorRsp.ReqOpCode, pRspAtt->ErrorRsp.Error);

		// Error code processing
		BtAttProcessError(ConnHdl, pRspAtt, RspLen);
	}
		break;
	case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP:
	{
		uint16_t mtu = min(BtAttGetMtu(), pRspAtt->ExchgMtuReqRsp.RxMtu);
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP (0x03): %d %d\r\n", pRspAtt->ExchgMtuReqRsp.RxMtu, mtu);
		BtAttSetMtu(mtu);
	}
		break;
	case BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_INFORMATION_RSP (0x05) \r\n");
		BtAttFindInfoRsp_t *p = (BtAttFindInfoRsp_t*) &pRspAtt->FindInfoRsp;
		DEBUG_PRINTF("Format %d\r\n", p->Fmt);

		if (RspLen <= 3)
		{
			DEBUG_PRINTF("Hdl = %d\r\n", p->HdlUuid16->Hdl);
			DEBUG_PRINTF("Uuid16 = (0x)%02x \r\n", p->HdlUuid16->Uuid);
		}
		else
		{
			DEBUG_PRINTF("Hdl = %d\r\n", p->HdlUuid128->Hdl);
			DEBUG_PRINTF("Uuid128 = (0x) ");
			for (int i = 0; i < RspLen - 2; i++)
			{
				DEBUG_PRINTF("%02x ", p->HdlUuid128->Uuid[i]);
			}
			DEBUG_PRINTF("\r\n");
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP (0x07)\r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP (0x09)\r\n");
		BtAttReadByTypeRsp_t *p = (BtAttReadByTypeRsp_t *) &pRspAtt->ReadByTypeRsp;
		DEBUG_PRINTF("Len %d, Raw Data (hex): ", p->Len);
		for (int i = 0; i < p->Len; i++)
			DEBUG_PRINTF("%X ", p->Data[i]);
		DEBUG_PRINTF("\r\n");

		uint16_t Hdl = p->Data[0] | (p->Data[1] << 8);
		uint8_t CharProp = p->Data[2];
		uint16_t CharHdlVal = p->Data[3] | (p->Data[4] << 8);

		switch (s_UuidType.Uuid16)
		{
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		{
			DEBUG_PRINTF("Response BT_UUID_DECLARATIONS_CHARACTERISTIC 0x2803\r\n");
			BtGattDBChar_t *pChar =
					(BtGattDBChar_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx].charateristics[CurIdx.CharIdx];
			memcpy((uint8_t*) &pChar->characteristic.char_props, &CharProp,
					sizeof(BtGattCharProps_t));
			pChar->characteristic.handle_value = CharHdlVal;

			if (p->Len <= 7) //16-bit UUID
			{
				pChar->characteristic.uuid.Uuid = p->Data[5]
						| (p->Data[6] << 8);
				pChar->characteristic.uuid.BaseIdx = 0;
				pChar->characteristic.uuid.Type = BT_UUID_TYPE_16;
			}
			else if (p->Len > 7 && p->Len <= 9) // 32-bit UUID?
			{
				uint32_t CharUuid32 = p->Data[5] | (p->Data[6] << 8)
						| (p->Data[7] << 16) | (p->Data[8] << 24);
			}
			else // 128-bit UUID
			{
				uint8_t *pUuid128 = (uint8_t*) &p->Data[5];
				uint16_t CharUuid16 = pUuid128[12] | (pUuid128[13] << 8);
				DEBUG_PRINTF("128-bit UUID with UUID16 0x%X \r\n", CharUuid16);
				pUuid128[12] = 0;
				pUuid128[13] = 0;
				int idx = BtUuidFindBase(pUuid128);
				DEBUG_PRINTF("BaseUUID128 (index %d) (hex): ", idx);
				for (int i = 0; i < 16; i++)
					DEBUG_PRINTF("%X ", pUuid128[i]);
				DEBUG_PRINTF("\r\n");

				pChar->characteristic.uuid.BaseIdx = idx;
				pChar->characteristic.uuid.Uuid = CharUuid16;
				pChar->characteristic.uuid.Type = BT_UUID_TYPE_128;
			}

			DEBUG_PRINTF("SrvcIdx %d, CharIdx %d: ", CurIdx.SrvIdx,
					CurIdx.CharIdx);
			DEBUG_PRINTF(
					"Hdl %d, CharProp 0x%X, CharHdlVal %d, CharUuid16 0x%X\r\n",
					Hdl, pChar->characteristic.char_props,
					pChar->characteristic.handle_value,
					pChar->characteristic.uuid.Uuid);

			CurIdx.CharIdx++;
			g_BtDevSdc.Services[CurIdx.SrvIdx].char_count++;

			DEBUG_PRINTF("New char_count = %d\r\n",
					g_BtDevSdc.Services[CurIdx.SrvIdx].char_count);

			// Search for the next BLE characteristic
			uint8_t eHdl =
					g_BtDevSdc.Services[CurIdx.SrvIdx].handle_range.EndHdl;
			if (Hdl + 1 <= eHdl)
			{
				CurIdx.Hdl = Hdl + 1;
				DEBUG_PRINTF(
						"Search for the next characteristic with sHdl = %d\r\n",
						CurIdx.Hdl);
				s_UuidType.BaseIdx = 0;
				s_UuidType.Type = BT_UUID_TYPE_16;
				//s_UuidType.Uuid16 = BT_UUID_DECLARATIONS_CHARACTERISTIC;
				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
						g_BtDevSdc.ConnHdl, CurIdx.Hdl, eHdl, &s_UuidType);
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		{
			DEBUG_PRINTF("Respone BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902\r\n");
			DEBUG_PRINTF("SrvcIdx %d CharIdx %d CurHdl %d\r\n", CurIdx.SrvIdx, CurIdx.CharIdx, CurIdx.Hdl);
			BtGattDBChar_t *pChar =
					(BtGattDBChar_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx].charateristics[CurIdx.CharIdx];

//			if (pChar->characteristic.char_props.notify)
//				DEBUG_PRINTF("Notification Characteristic \r\n");
//			else if (pChar->characteristic.char_props.indicate)
//				DEBUG_PRINTF("Indication Characteristic\r\n");
//			else
//				DEBUG_PRINTF("Neither notification nor indication char\r\n");

			if (pChar->characteristic.char_props.notify
					|| pChar->characteristic.char_props.indicate)
			{
				pChar->cccd_handle = Hdl;
				DEBUG_PRINTF(
						"SrvcIdx %d CharIdx %d with Hdl %d has CCC_handle %d\r\n",
						CurIdx.SrvIdx, CurIdx.CharIdx,
						pChar->characteristic.handle_value, pChar->cccd_handle);
			}

			bool bNextSrvc = false;
			CurIdx.CharIdx++;
			if (CurIdx.CharIdx < g_BtDevSdc.Services[CurIdx.SrvIdx].char_count)
			{
				DEBUG_PRINTF("Scan Next CharIdx %d\r\n", CurIdx.CharIdx);
				pChar = (BtGattDBChar_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx].charateristics[CurIdx.CharIdx];
				uint8_t eHdl = g_BtDevSdc.Services[CurIdx.SrvIdx].handle_range.EndHdl;
				if (Hdl + 1 <= eHdl)
				{
					CurIdx.Hdl = Hdl + 1;
					BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
							g_BtDevSdc.ConnHdl, CurIdx.Hdl, eHdl, &s_UuidType);
				}
				else
				{
					bNextSrvc = true;
				}
			}
			else
			{
				bNextSrvc = true;
			}

			// Look into the next BLE service
			if (bNextSrvc)
			{
				CurIdx.SrvIdx++;
				if (CurIdx.SrvIdx < g_BtDevSdc.NbSrvc)
				{
					DEBUG_PRINTF("Scan Next SrvcIdx %d\r\n", CurIdx.SrvIdx);
					CurIdx.Hdl =
							g_BtDevSdc.Services[CurIdx.SrvIdx].handle_range.StartHdl;
					uint8_t eHdl =
							g_BtDevSdc.Services[CurIdx.SrvIdx].handle_range.EndHdl;
					BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
							g_BtDevSdc.ConnHdl, CurIdx.Hdl, eHdl, &s_UuidType);
				}
				else
				{
					DEBUG_PRINTF("Out of number of service. Start parsing other UUID type\r\n");
				}
			}
		}
			break;
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
		{
			DEBUG_PRINTF("Response BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION 0x2901\r\n");
			uint16_t hdl = p->Data[0] | (p->Data[1] << 8);
			DEBUG_PRINTF("Hdl = %d\r\n", hdl);
			DEBUG_PRINTF("Data (hex): ");
			for (int i = 2; i < p->Len; i++)
				DEBUG_PRINTF("%X ", p->Data[i]);
			DEBUG_PRINTF("\r\n");
		}
			break;
		default:
			DEBUG_PRINTF("Unprocess Uuid16 Type (code 0x%X)\r\n", s_UuidType.Uuid16);
		}
	}
	break;
	case BT_ATT_OPCODE_ATT_READ_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_RSP (0x0B), RspLen = %d \r\n", RspLen);
		BtAttReadRsp_t *p = (BtAttReadRsp_t *) &pRspAtt->ReadRsp;

		DEBUG_PRINTF("Raw Data (hex): ");
		for (int i = 0; i < RspLen; i++)
		{
			DEBUG_PRINTF("%x ", p->Data[i]);
		}
		DEBUG_PRINTF("\r\n");

		BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx];

		// Parse the next handle
		CurIdx.Hdl++;
		if (CurIdx.Hdl > pSrvc->handle_range.EndHdl)
		{
			// Jump to the next BLE service handle range
			if (CurIdx.SrvIdx < (g_BtDevSdc.NbSrvc - 1))
			{
				CurIdx.SrvIdx++;
				DEBUG_PRINTF("Next service idx = %d, UUID16 = 0x%X\r\n",
						CurIdx.SrvIdx, pSrvc->srv_uuid.Uuid);
				pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx];
				CurIdx.Hdl = pSrvc->handle_range.StartHdl + 1; // ignore the first handle, which corresponds to service handle

				BtAttReadRequest((BtHciDevice_t *)g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, CurIdx.Hdl);
			}
			else
			{
				DEBUG_PRINTF("All handles were scanned\r\n");
			}
		}
		else
		{
			DEBUG_PRINTF("Next CurIdx.Hdl = %d \r\n", CurIdx.Hdl);
			BtAttReadRequest((BtHciDevice_t *)g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, CurIdx.Hdl);
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BLOB_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BLOB_RSP (0x0D) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_RSP (0x0F) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP (0x11) \r\n");
		BtAttReadByGroupTypeRsp_t *p =
				(BtAttReadByGroupTypeRsp_t*) &pRspAtt->ReadByGroupTypeRsp;
		DEBUG_PRINTF("Len = %d, Raw data (hex): ", p->Len);
		for (int i = 0; i < p->Len; i++)
			DEBUG_PRINTF("%X ", p->Data[i]);
		DEBUG_PRINTF("\r\n");

		bool bScanNext = false;
		uint16_t NextStartHdl;

		g_BtDevSdc.NbSrvc++;
		uint8_t SrvcIdx = g_BtDevSdc.NbSrvc - 1;

		// Process the received data
		if (p->Len <= 6) // Process UUID 16
		{
			BtAttReadByGroupTypeRspUuid16_t *g =
					(BtAttReadByGroupTypeRspUuid16_t*) p->Data;
			DEBUG_PRINTF("StartHdl = %d, EndHdl = %d, Service UUID16 0x%X \r\n", g->HdlStart, g->HdlEnd, g->Uuid);

			BtGattDBSrvc_t *pSrvc =
					(BtGattDBSrvc_t*) &g_BtDevSdc.Services[SrvcIdx];
			pSrvc->handle_range.StartHdl = g->HdlStart;
			pSrvc->handle_range.EndHdl = g->HdlEnd;
			pSrvc->srv_uuid.Uuid = g->Uuid;
			pSrvc->srv_uuid.BaseIdx = 0; // Standard Bluetooth service
			pSrvc->srv_uuid.Type = BT_UUID_TYPE_16;

			bScanNext = (g->HdlEnd != 0xFFFF) ? true : false;
			NextStartHdl = (g->HdlEnd != 0xFFFF) ? (g->HdlEnd + 1) : 0xFFFF;
		}
		else // Process UUID 128
		{
			BtAttReadByGroupTypeRspUuid128_t *g =
					(BtAttReadByGroupTypeRspUuid128_t*) p->Data;
			DEBUG_PRINTF("StartHdl = %d, EndHdl = %d \r\n", g->HdlStart, g->HdlEnd);
			DEBUG_PRINTF("Custom service UUID128 (hex): ");
			for (int i = 0; i < 16; i++)
				DEBUG_PRINTF("%X ", g->Uuid[i]);
			DEBUG_PRINTF("\r\n");

			BtGattDBSrvc_t *pSrvc =
					(BtGattDBSrvc_t*) &g_BtDevSdc.Services[SrvcIdx];
			pSrvc->handle_range.StartHdl = g->HdlStart;
			pSrvc->handle_range.EndHdl = g->HdlEnd;
			int idx = BtUuid128To16(&pSrvc->srv_uuid, g->Uuid);
			g->Uuid[12] = 0;
			g->Uuid[13] = 0;
			DEBUG_PRINTF("BLE service UUID16 0x%X \r\n", pSrvc->srv_uuid.Uuid);
			DEBUG_PRINTF("BaseUUID128 (hex) = ");
			for (int i = 0; i < 16; i++)
				DEBUG_PRINTF("%X ", g->Uuid[i]);
			DEBUG_PRINTF("was added at the internal table with index %d\r\n", idx);

			bScanNext = (g->HdlEnd != 0xFFFF) ? true : false;
			NextStartHdl = (g->HdlEnd != 0xFFFF) ? (g->HdlEnd + 1) : 0xFFFF;
		}

		// Read the next group type
		if (bScanNext)
		{
			// Continue to read the next group
			DEBUG_PRINTF("Read the next group type with StartHdl = %d\r\n", NextStartHdl);
			BtUuid_t Uuid = {
					.BaseIdx = 0, // Standard bluetooth
					.Type = BT_UUID_TYPE_16,
					.Uuid16 = BT_UUID_DECLARATIONS_PRIMARY_SERVICE,
			};
			BtAttReadByGroupTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev,
					g_BtDevSdc.ConnHdl, NextStartHdl, 0xFFFF, &Uuid);
		}

	}
		break;
	case BT_ATT_OPCODE_ATT_WRITE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_WRITE_RSP (0x13) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP (0x19) \r\n");
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP:
	{
		DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP (0x21) \r\n");
	}
		break;
	}
}

uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReqAtt, int ReqLen, BtAttReqRsp_t * const pRspAtt)
{
	uint32_t retval = 0;

	DEBUG_PRINTF("ATT OpCode %x, L2Cap len %d\n", pReqAtt->OpCode, ReqLen);

	switch (pReqAtt->OpCode)
	{
		case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:
			{
				BtAttExchgMtuReqRsp_t *req = (BtAttExchgMtuReqRsp_t*)&pReqAtt->ExchgMtuReqRsp;

				DEBUG_PRINTF("ATT_EXCHANGE_MTU_REQ (0x02) \r\n");
				DEBUG_PRINTF("RxMtu %d %d\r\n", pReqAtt->ExchgMtuReqRsp.RxMtu, s_AttMtu);

				if (pReqAtt->ExchgMtuReqRsp.RxMtu < BT_ATT_MTU_MIN)
				{
					retval = BtAttError(pRspAtt, ConnHdl, BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ, BT_ATT_ERROR_INVALID_ATT_VALUE);
					break;
				}
				retval = sizeof(BtAttExchgMtuReqRsp_t) + 1;
				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_EXCHANGE_MTU_RSP;
				pRspAtt->ExchgMtuReqRsp.RxMtu = BtAttSetMtu(pReqAtt->ExchgMtuReqRsp.RxMtu);

				//DEBUG_PRINTF("MTU : %d\r\n", s_AttMtu);
			}
			break;
		case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ (0x04) \r\n");
				BtAttFindInfoReq_t *req = (BtAttFindInfoReq_t*)&pReqAtt->FindInfoReq;

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
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

						pRspAtt->FindInfoRsp.HdlUuid128[0].Hdl = entry->Hdl;
						uuid128[12] = entry->TypeUuid.Uuid & 0xff;
						uuid128[13] = entry->TypeUuid.Uuid >> 8;
						memcpy(pRspAtt->FindInfoRsp.HdlUuid128[0].Uuid, uuid128, 16);
						l = sizeof(BtAttHdlUuid128_t);
					}
					else
					{
						pRspAtt->FindInfoRsp.Fmt = BT_ATT_FIND_INFORMATION_RSP_FMT_UUID16;

						pRspAtt->FindInfoRsp.HdlUuid16[0].Hdl = entry->Hdl;
						pRspAtt->FindInfoRsp.HdlUuid16[0].Uuid = entry->TypeUuid.Uuid;
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
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ (0x06) \r\n");
				DEBUG_PRINTF("TODO:");
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ: // Parse UUID Type of characteristic inside a BLE service
			{
				// Only the attributes with attribute handles between and including
				// the Starting Handle and the Ending Handle with the attribute type
				// that is the same as the Attribute Type given will be returned. To
				// search through all attributes, the starting handle shall be set to
				// 0x0001 and the ending handle shall be set to 0xFFFF.
				BtAttReadByTypeReq_t *req = (BtAttReadByTypeReq_t*)&pReqAtt->ReadByTypeReq;

				DEBUG_PRINTF(
						"BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ (0x08): StartHdl %d, EndHdl %d, Uuid16Type 0x%X \r\n",
						pReqAtt->ReadByTypeReq.StartHdl, pReqAtt->ReadByTypeReq.EndHdl,
						req->Uuid.Uuid16);

				if (req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_TYPE_RSP;

				uint8_t *p = (uint8_t*)pRspAtt->ReadByTypeRsp.Data;

				//DEBUG_PRINTF("sHdl: %x, eHdl: %x, Type: %x\r\n", req->StartHdl, req->EndHdl, req->Uuid.Uuid16);
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};

				BtAttDBEntry_t *entry = BtAttDBFindUuidRange(&uid16, req->StartHdl, req->EndHdl);
				if (entry)
				{
					DEBUG_PRINTF("Entry found\r\n");
				}
				else
				{
					DEBUG_PRINTF("Entry not found\r\n");
				}
#if 0
				if (entry)
				{
					p[0] = entry->Hdl & 0xFF;
					p[1] = entry->Hdl >> 8;
					p +=2;
					pRspAtt->ReadByTypeRsp.Len = BtAttReadValue(entry, 0, p, s_AttMtu - 2) + 2;

					retval = 2 + pRspAtt->ReadByTypeRsp.Len;//rsp->Len * c;
					break;
				}
				retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);

#else
				int l = 0;
				pRspAtt->ReadByTypeRsp.Len = 0;

				while (entry && (s_AttMtu - l) >= BT_ATT_MTU_MIN)
				{
					if (entry->Hdl < req->StartHdl || entry->Hdl > req->EndHdl)
					{
						break;
					}

					p[0] = entry->Hdl & 0xFF;
					p[1] = entry->Hdl >> 8;
					p +=2;

					int cnt = BtAttReadValue(entry, 0, p, s_AttMtu - l - 2) + 2;
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
				//DEBUG_PRINTF("retval : %d\r\n", retval);
#endif
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_REQ:
			{
				// The ATT_READ_REQ PDU is used to request the server to read the value
				// of an attribute and return its value in an ATT_READ_RSP PDU.
				BtAttReadReq_t *req = (BtAttReadReq_t*)&pReqAtt->ReadReq;
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_REQ (0x0A): ServIdx = %d, CharIdx = %d, Hdl = %d\r\n", CurIdx.SrvIdx, CurIdx.CharIdx, req->Hdl);

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_RSP;

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
					DEBUG_PRINTF("Entry with Hdl = %d found\r\n", req->Hdl);
					retval = BtAttReadValue(entry, 0, pRspAtt->ReadRsp.Data, s_AttMtu - 1)  + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_READ_REQ, BT_ATT_ERROR_INVALID_HANDLE);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BLOB_REQ:
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_REQ_BLOB_REQ (0x0C):\r\n");

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BLOB_RSP;
				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadBlobReq.Hdl);

				if (entry)
				{
					retval = BtAttReadValue(entry, pReqAtt->ReadBlobReq.Offset, pRspAtt->ReadBlobRsp.Data, s_AttMtu - 1) + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, pReqAtt->ReadBlobReq.Hdl, BT_ATT_OPCODE_ATT_READ_BLOB_REQ, BT_ATT_ERROR_INVALID_HANDLE);
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

				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ (0x0E)\r\n");
				int nhdl = (ReqLen - 1) >> 1;
				uint8_t *p = pRspAtt->ReadMultipleRsp.Data;
				retval = 1;
				for (int i = 0; i < nhdl && (s_AttMtu - retval) < BT_ATT_MTU_MIN; i++)
				{
					BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadMultipleReq.Hdl[i]);
					if (entry == nullptr)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleReq.Hdl[i], BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
						break;
					}
					int l = BtAttReadValue(entry, 0, p, s_AttMtu);
					p += l;
					retval += l;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ: // Parse the info of primary/secondary service
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ (0x10) \r\n");
				BtAttReadByGroupTypeReq_t *req = (BtAttReadByGroupTypeReq_t*)&pReqAtt->ReadByGroupTypeReq;
				DEBUG_PRINTF("Requested StartHdl = %d, EndHdl = %d\r\n", req->StartHdl, req->EndHdl);
				DEBUG_PRINTF("uuid16 = 0x%02x \r\n", req->Uuid.Uuid16);
				DEBUG_PRINTF("uuid32 = 0x%02x \r\n", req->Uuid.Uuid32);
				DEBUG_PRINTF("uuid128 = (0x) ");
				for (int i = 0; i < 16; i++)
					DEBUG_PRINTF("%x ", req->Uuid.Uuid128[i]);
				DEBUG_PRINTF("\r\n");

				if (req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl, BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_RSP;

				uint8_t *p = (uint8_t*)pRspAtt->ReadByGroupTypeRsp.Data;
				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Uuid.Uuid16};
				BtAttHdlRange_t *hu = (BtAttHdlRange_t*)p;
				int l = 0;

				pRspAtt->ReadByGroupTypeRsp.Len = 0;

				hu->StartHdl = req->StartHdl;
				hu->EndHdl = req->EndHdl;

				BtAttDBEntry_t *entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);

				int baseidx = 0;// = entry->TypeUuid.BaseIdx;

				if (entry)
				{
					baseidx = entry->TypeUuid.BaseIdx;
				}

				uint8_t prevlen = 0;
				pRspAtt->ReadByGroupTypeRsp.Len = 0;
				int temp_cnt = 0;

				while (entry && (s_AttMtu - l) >= BT_ATT_MTU_MIN)
				{
					DEBUG_PRINTF("#%d BaseIdx = %d, entry->Hdl = %d, entry->pNext->Hdl = %d, UuidType = %d, uuid16 = 0x%x\r\n",
							temp_cnt, entry->TypeUuid.BaseIdx, entry->Hdl, entry->pNext->Hdl, entry->TypeUuid.Type, entry->TypeUuid.Uuid);

					if (entry->Hdl >= req->StartHdl && entry->Hdl <= req->EndHdl && baseidx == entry->TypeUuid.BaseIdx)
					{
						p += sizeof(BtAttHdlRange_t);

						int cnt = BtAttReadValue(entry, 0, p, s_AttMtu - l - sizeof(BtAttHdlRange_t));

						BtAttSrvcDeclar_t *x = (BtAttSrvcDeclar_t *) p;
						DEBUG_PRINTF("Ble Service UUID16 = 0x%X \r\n", x->Uuid.Uuid16);

						if (pRspAtt->ReadByGroupTypeRsp.Len == 0)
						{
							pRspAtt->ReadByGroupTypeRsp.Len = cnt;
						}
						else if (cnt != pRspAtt->ReadByGroupTypeRsp.Len)
						{
							break;
						}

						p += pRspAtt->ReadByGroupTypeRsp.Len;
						l += 4 + pRspAtt->ReadByGroupTypeRsp.Len;
						((BtAttHdlRange_t*)p)->StartHdl = hu->EndHdl;
						hu = (BtAttHdlRange_t*)p;// jump to the next entry
						hu->EndHdl = req->EndHdl;
					}
					else
					{
						break;
					}
					DEBUG_PRINTF("Next Find Range StartHdl = %d, EndHdl = %d, Uuid16Type = 0x%X\r\n",
							hu->StartHdl, hu->EndHdl, uid16.Uuid);
					entry = BtAttDBFindHdlRange(&uid16, &hu->StartHdl, &hu->EndHdl);
					temp_cnt++;
				}

				if (l > 0)
				{
					pRspAtt->ReadByGroupTypeRsp.Len += 4;
					retval = l + 2;
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

				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_WRITE_REQ (0x12):\r\n");

				if (req->Hdl < 1)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
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
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_CMD (0x52):\r\n");

				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->SignedWriteCmd.Hdl);

				if (entry)
				{
					size_t l = ReqLen;

					l = BtAttWriteValue(entry, 0, pReqAtt->SignedWriteCmd.Data, l - 3);

					retval = 0;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:
			break;
		case BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ:
			{
				DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ (0x20) \r\n");
				uint16_t *hdl = pReqAtt->ReadMultipleVarReq.Hdl;
				uint8_t *p = pRspAtt->ReadMultipleVarRsp.Data;
				int l = 0;

				while (ReqLen > 0 && (s_AttMtu - l) >= BT_ATT_MTU_MIN)
				{
					BtAttDBEntry_t *entry = BtAttDBFindHandle(*hdl);

					if (entry == nullptr)
					{
						break;
					}

					pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP;
					uint16_t n = BtAttReadValue(entry, 0, p + 2, s_AttMtu - l - 2);
					p[0] = n & 0xFF;
					p[1] = n >> 8;
					p += n + 2;
					l += n + 2;
					hdl++;
					ReqLen -= 2;
				}
				if (l > 0)
				{
					retval = l + 1;
				}
				else
				{
					retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleVarReq.Hdl[0], BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:
			{
				//DEBUG_PRINTF("BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:\r\n");
			}
			break;
		case BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD:
			break;
		default:
			//DEBUG_PRINTF("OpCode : %x\r\n", pReqAtt->OpCode);
			BtAttProcessRsp(ConnHdl, pReqAtt, ReqLen);
	}

	return retval;
}

uint32_t BtAttProcessError(uint16_t ConnHdl, BtAttReqRsp_t * const pRspAtt, int RspLen)
{
	uint32_t retval = 0;

	switch (pRspAtt->ErrorRsp.ReqOpCode)
	{
	case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_ATT_NOT_FOUND)
		{
			DEBUG_PRINTF("ATT not found at StartHdl = %d\r\n", pRspAtt->ErrorRsp.Hdl);
			DEBUG_PRINTF("Total found BLE services = %d \r\n", g_BtDevSdc.NbSrvc);

			for (int i=0; i < g_BtDevSdc.NbSrvc; i++)
			{
				DEBUG_PRINTF("Service #%d: ", i);
				BtGattDBSrvc_t *p = (BtGattDBSrvc_t *) &g_BtDevSdc.Services[i];
				DEBUG_PRINTF("StartHdl = %d, EndHdl = %d\r\n", p->handle_range.StartHdl, p->handle_range.EndHdl);
			}

			// Look into the first BLE Srvc
			CurIdx.SrvIdx = 0;
			BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx];
			CurIdx.CharIdx = 0;
			CurIdx.Hdl = pSrvc->handle_range.StartHdl;
			if (CurIdx.Hdl > pSrvc->handle_range.EndHdl)
			{
				DEBUG_PRINTF("Parsing Hdl (%d) larger than EndHdl (%d)\r\n", CurIdx.Hdl, pSrvc->handle_range.EndHdl);
			}
			else
			{
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;

				//s_UuidType.BaseIdx = 0;
				//s_UuidType.Type = BT_UUID_TYPE_16;
				s_UuidType.Uuid16 = BT_UUID_DECLARATIONS_CHARACTERISTIC;

				DEBUG_PRINTF(
						"Parse the characteristic of the first service ConnHdl %d, sHdl %d, eHdl %d, uuid 0x%X, baseIdx %d\r\n",
						g_BtDevSdc.ConnHdl, sHdl, eHdl, s_UuidType.Uuid16, s_UuidType.BaseIdx);

				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, sHdl, eHdl, &s_UuidType);
			}
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_ATT_NOT_FOUND)
		{
			DEBUG_PRINTF("ATT with sHdl %d of SrvcIdx %d not found \r\n", CurIdx.Hdl, CurIdx.SrvIdx);

			if (CurIdx.SrvIdx >= g_BtDevSdc.NbSrvc - 1)
			{
				DEBUG_PRINTF("Out of number of services. Restart to scan for different Uuid16 Type\r\n");
				CurIdx.SrvIdx = 0;
				CurIdx.CharIdx = 0;
				BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx];
				CurIdx.Hdl = pSrvc->handle_range.StartHdl;
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;

				//s_UuidType.BaseIdx = 0;
				//s_UuidType.Type = BT_UUID_TYPE_16;

				switch (s_UuidType.Uuid16)
				{
				case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				{
					DEBUG_PRINTF("Characteristic scan done. Summary:\r\n");
					for (int i = 0; i < g_BtDevSdc.NbSrvc; i++)
						DEBUG_PRINTF("    SrvcIdx %d has %d characteristics\r\n", i, g_BtDevSdc.Services[i].char_count);

					DEBUG_PRINTF("Start Scanning for BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION 0x2902\r\n");
					s_UuidType.Uuid16 = BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION;
				}
					break;
				case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				{
					DEBUG_PRINTF("Start Scanning for BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION 0x2901\r\n");
					s_UuidType.Uuid16 = BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION;
				}
					break;
				default:
					DEBUG_PRINTF("Unprocess Uuid16 Type (code 0x%X)\r\n", s_UuidType.Uuid16);
				}

				DEBUG_PRINTF("SrvcIdx %d, CharIdx %d, sHdl %d, eHdl %d \r\n", CurIdx.SrvIdx, CurIdx.CharIdx, sHdl, eHdl);
				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, sHdl, eHdl, &s_UuidType);
			}
			else
			{
				DEBUG_PRINTF("Parse the next BLE SrvcIdx %d\r\n", CurIdx.CharIdx + 1);
				CurIdx.SrvIdx++;
				CurIdx.CharIdx = 0;
				BtGattDBSrvc_t *pSrvc = (BtGattDBSrvc_t*) &g_BtDevSdc.Services[CurIdx.SrvIdx];
				CurIdx.Hdl = pSrvc->handle_range.StartHdl;
				uint16_t sHdl = pSrvc->handle_range.StartHdl;
				uint16_t eHdl = pSrvc->handle_range.EndHdl;
				DEBUG_PRINTF("sHdl %d, eHdl %d \r\n", sHdl, eHdl);

				//s_UuidType.BaseIdx = 0;
				//s_UuidType.Type = BT_UUID_TYPE_16;
				BtAttReadByTypeRequest((BtHciDevice_t*) g_BtDevSdc.pHciDev, g_BtDevSdc.ConnHdl, sHdl, eHdl, &s_UuidType);
			}
		}
	}
		break;
	case BT_ATT_OPCODE_ATT_READ_REQ:
	{
		if (pRspAtt->ErrorRsp.Error == BT_ATT_ERROR_INVALID_HANDLE)
		{
			DEBUG_PRINTF("BT_ATT_OPCODE_ATT_READ_REQ (0x0A) error: invalid handle\r\n");
		}
	}
		break;
	default:
		break;
	}

	return retval;
}

