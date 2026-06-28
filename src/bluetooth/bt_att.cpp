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
#include "bluetooth/bt_att.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_peer.h"

/******** For DEBUG ************/
//#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
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

// Set the negotiated ATT MTU from a peer's offered Rx MTU. The negotiated
// value is min(peer_offered, BT_ATT_MTU_MAX). Offers below BT_ATT_MTU_MIN
// are rejected (BT Core spec mandates 23 as the absolute floor).
//
// NOTE: s_AttMtu is currently a single global. Multi-link operation needs
// per-connection MTU; that refactor is tracked separately.
uint16_t BtAttSetMtu(uint16_t Mtu)
{
	if (Mtu < BT_ATT_MTU_MIN)
	{
		return s_AttMtu;
	}
	if (Mtu > BT_ATT_MTU_MAX)
	{
		Mtu = BT_ATT_MTU_MAX;
	}
	s_AttMtu = Mtu;
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

	// We will write pNext/pPrev into the slot at (entry + l) below to seed
	// the next tail entry. That seed write touches the first few bytes of
	// a BtAttDBEntry_t at (entry + l), so the bound check must reserve
	// sizeof(BtAttDBEntry_t) past the end of this entry, not just l bytes.
	// Without the pad, the seed writes spill past s_BtAttDBMemEnd when the
	// DB is exactly full.
	if ((uint32_t)entry + l + sizeof(BtAttDBEntry_t) > s_BtAttDBMemEnd)
	{
		//DEBUG_PRINTF("Out mem. Required %d, Reserved : %d\r\n", ((uint32_t)entry + l + sizeof(BtAttDBEntry_t)) - (uint32_t)s_BtAttDBMem, s_BtAttDBMemEnd - (uint32_t)s_BtAttDBMem);
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
	if (Hdl > s_LastHdl)
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

// Strong override of the weak default in bt_gatt.cpp. On the native host the
// ATT DB exists, so mirror the aggregate CCCD value into the descriptor entry
// for a local CCCD read.
void BtGattCccdDbSync(uint16_t CccdHdl, uint16_t CccVal)
{
	BtAttDBEntry_t *entry = BtAttDBFindHandle(CccdHdl);
	if (entry != nullptr)
	{
		BtDescClientCharConfig_t *pCccd = (BtDescClientCharConfig_t*)entry->Data;
		pCccd->CccVal = CccVal;
	}
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

			// Clamp to the bytes available from Offset and to the caller's
			// buffer/MTU cap (Len). Offset is attacker-controlled on a blob
			// read; without the bound an oversized Offset/ValueLen reads past
			// the value and overflows the response buffer.
			uint16_t vlen = p->pChar->ValueLen;
			if (Offset <= vlen)
			{
				size_t l = min((uint16_t)(vlen - Offset), Len);
				memcpy(pBuff, (uint8_t*)p->pChar->pValue + Offset, l);
				len = l;
			}
		}

		}
	}
	else
	{
		BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;
		DEBUG_PRINTF("Read Req UUID custom: %d: %x, %d\r\n", pEntry->TypeUuid.BaseIdx, pEntry->TypeUuid.Uuid, p->pChar->ValueLen);

		// Clamp to bytes available from Offset and to the caller's buffer/MTU
		// cap (Len) to avoid an out-of-bounds read / response overflow.
		uint16_t vlen = p->pChar->ValueLen;
		if (Offset <= vlen)
		{
			size_t l = min((uint16_t)(vlen - Offset), Len);
			memcpy(pBuff, (uint8_t*)p->pChar->pValue + Offset, l);
			len = l;
		}
	}

	return len;
}


static size_t BtAttReadValueForConn(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
									uint16_t Offset, uint8_t *pBuff, uint16_t Len)
{
	if (pEntry == nullptr || pBuff == nullptr)
	{
		return 0;
	}

	if (pEntry->TypeUuid.BaseIdx == 0 &&
		pEntry->TypeUuid.Uuid == BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION)
	{
		if (Offset >= 2 || Len == 0)
		{
			return 0;
		}

		uint16_t cccd = BtGattCccdGet(ConnHdl, pEntry->Hdl);
		uint8_t tmp[2] = { (uint8_t)(cccd & 0xFF), (uint8_t)(cccd >> 8) };
		uint16_t n = min((uint16_t)(2 - Offset), Len);

		memcpy(pBuff, &tmp[Offset], n);
		return n;
	}

	return BtAttReadValue(pEntry, Offset, pBuff, Len);
}

static bool BtAttEntryIsCharValue(BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return false;
	}

	if (pEntry->TypeUuid.BaseIdx != 0)
	{
		return true;
	}

	switch (pEntry->TypeUuid.Uuid)
	{
		case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
		case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
		case BT_UUID_DECLARATIONS_INCLUDE:
		case BT_UUID_DECLARATIONS_CHARACTERISTIC:
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
		case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
		case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
		case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
			return false;
		default:
			return true;
	}
}

static BtGattChar_t *BtAttEntryChar(BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return nullptr;
	}

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_DECLARATIONS_CHARACTERISTIC:
				return ((BtAttCharDeclar_t*)pEntry->Data)->pChar;

			case BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				return ((BtDescClientCharConfig_t*)pEntry->Data)->pChar;

			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				return ((BtDescCharUserDesc_t*)pEntry->Data)->pChar;

			case BT_UUID_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_DECLARATIONS_SECONDARY_SERVICE:
			case BT_UUID_DECLARATIONS_INCLUDE:
			case BT_UUID_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				return nullptr;

			default:
				break;
		}
	}

	if (BtAttEntryIsCharValue(pEntry))
	{
		return ((BtAttCharValue_t*)pEntry->Data)->pChar;
	}

	return nullptr;
}

// Port/application hook for security state that is not represented in the ATT
// database yet. Return 0 to allow access, or an ATT error code such as
// BT_ATT_ERROR_INSUF_ENCRYPT / BT_ATT_ERROR_INSUF_AUTHEN /
// BT_ATT_ERROR_ENCRYPT_KEY_TOO_SHORT to reject it. The default keeps today's
// open-service behaviour while property permission checks below become active.
__attribute__((weak)) uint8_t BtAttAccessSecurityError(uint16_t ConnHdl,
													  BtAttDBEntry_t *pEntry,
													  bool bRead)
{
	(void)ConnHdl;
	(void)pEntry;
	(void)bRead;
	return 0;
}

static uint8_t BtAttReadPermError(uint16_t ConnHdl, BtAttDBEntry_t *pEntry)
{
	if (pEntry == nullptr)
	{
		return BT_ATT_ERROR_INVALID_HANDLE;
	}

	if (BtAttEntryIsCharValue(pEntry))
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		if (pChar == nullptr || (pChar->Property & BT_GATT_CHAR_PROP_READ) == 0)
		{
			return BT_ATT_ERROR_READ_NOT_PERMITTED;
		}
	}

	return BtAttAccessSecurityError(ConnHdl, pEntry, true);
}

static uint8_t BtAttWritePermError(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
								   uint8_t OpCode, const uint8_t *pData,
								   uint16_t Len)
{
	if (pEntry == nullptr)
	{
		return BT_ATT_ERROR_INVALID_HANDLE;
	}

	if (BtAttEntryIsCharValue(pEntry))
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		if (pChar == nullptr)
		{
			return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
		}

		uint32_t required = BT_GATT_CHAR_PROP_WRITE;
		if (OpCode == BT_ATT_OPCODE_ATT_CMD)
		{
			required = BT_GATT_CHAR_PROP_WRITE_WORESP;
		}
		else if (OpCode == BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD)
		{
			required = BT_GATT_CHAR_PROP_AUTH_SIGNED;
		}

		if ((pChar->Property & required) == 0)
		{
			return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
		}

		return BtAttAccessSecurityError(ConnHdl, pEntry, false);
	}

	if (pEntry->TypeUuid.BaseIdx == 0 &&
		pEntry->TypeUuid.Uuid == BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION)
	{
		BtGattChar_t *pChar = BtAttEntryChar(pEntry);
		if (pChar == nullptr)
		{
			return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
		}
		if (Len >= 2 && pData != nullptr)
		{
			uint16_t cccd = (uint16_t)(pData[0] | (pData[1] << 8));
			if ((cccd & ~(BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION |
						  BT_DESC_CLIENT_CHAR_CONFIG_INDICATION)) != 0)
			{
				return BT_ATT_ERROR_VALUE_NOT_ALLOWED;
			}
			if ((cccd & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION) &&
				(pChar->Property & BT_GATT_CHAR_PROP_NOTIFY) == 0)
			{
				return BT_ATT_ERROR_VALUE_NOT_ALLOWED;
			}
			if ((cccd & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION) &&
				(pChar->Property & BT_GATT_CHAR_PROP_INDICATE) == 0)
			{
				return BT_ATT_ERROR_VALUE_NOT_ALLOWED;
			}
		}
		return BtAttAccessSecurityError(ConnHdl, pEntry, false);
	}

	return BT_ATT_ERROR_WRITE_NOT_PERMITTED;
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

					// The CCCD value is a 2-byte field. A write shorter than 2
					// bytes would read past the received PDU; reject it instead
					// of dereferencing out of bounds.
					if (Len < 2)
					{
						break;
					}

					p->CccVal = *(uint16_t*)pData;
					p->pChar->bNotify = p->CccVal & BT_DESC_CLIENT_CHAR_CONFIG_NOTIFICATION ? true: false;
					if (p->pChar->SetNotifCB)
					{
						p->pChar->SetNotifCB(p->pChar, p->pChar->bNotify, BT_CONN_HDL_INVALID);
					}
					p->pChar->bIndic = p->CccVal & BT_DESC_CLIENT_CHAR_CONFIG_INDICATION ? true: false;
					if (p->pChar->SetIndCB)
					{
						p->pChar->SetIndCB(p->pChar, p->pChar->bIndic, BT_CONN_HDL_INVALID);
					}
					len = 2;
//					DEBUG_PRINTF("CccdHdl : %x\r\n", p->pChar->CccdHdl);
				}
				break;
			case BT_UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				{
					BtAttCharValue_t *p = (BtAttCharValue_t*)pEntry->Data;

					p->pChar->ValueLen = min(Len, p->pChar->MaxDataLen);// - sizeof(BtGattCharValue_t));
					memcpy(p->pChar->pValue, pData, p->pChar->ValueLen);
					len = p->pChar->ValueLen;

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
		memcpy(p->pChar->pValue, pData, p->pChar->ValueLen);
		len = p->pChar->ValueLen;

		if (p->pChar->WrCB)
		{
//					len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
			p->pChar->WrCB(p->pChar, pData, 0, Len);
		}
	}

	return len;
}


static size_t BtAttWriteValueForConn(uint16_t ConnHdl, BtAttDBEntry_t *pEntry,
									 uint16_t Offset, uint8_t *pData, uint16_t Len)
{
	if (pEntry == nullptr)
	{
		return 0;
	}

	if (pEntry->TypeUuid.BaseIdx == 0 &&
		pEntry->TypeUuid.Uuid == BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION)
	{
		if (Offset != 0 || Len != 2 || pData == nullptr)
		{
			return 0;
		}

		uint16_t cccd = (uint16_t)(pData[0] | (pData[1] << 8));
		return BtGattCccdSet(ConnHdl, pEntry->Hdl, cccd) ? 2 : 0;
	}

	return BtAttWriteValue(pEntry, Offset, pData, Len);
}

uint32_t BtAttError(BtAttReqRsp_t * const pRspAtt, uint16_t Hdl, uint8_t OpCode, uint8_t ErrCode)
{
	pRspAtt->OpCode = BT_ATT_OPCODE_ATT_ERROR_RSP;
	pRspAtt->ErrorRsp.ReqOpCode = OpCode;
	pRspAtt->ErrorRsp.Hdl = Hdl;
	pRspAtt->ErrorRsp.Error = ErrCode;

	return sizeof(BtAttErrorRsp_t) + 1;
}

// Execute a queued long write for one connection: walk the prepare queue,
// compact consecutive chunks that target the same handle into one contiguous
// value, then apply it through BtAttWriteValue (which fires the char WrCB once
// with the full value, same as a normal write). Queue records are laid out as
// { uint16 Hdl, uint16 Offset, uint16 Len, uint8 Data[Len] } in arrival order;
// clients send chunks in ascending offset, so concatenation rebuilds the value.
static void BtAttExecLongWrite(BtDevice_t *pConn)
{
	if (pConn == nullptr || pConn->Conn.pLongWrBuff == nullptr)
	{
		return;
	}

	uint8_t  *buf   = pConn->Conn.pLongWrBuff;
	uint16_t  total = pConn->Conn.LongWrLen;
	uint16_t  pos   = 0;

	while (pos + 6 <= total)
	{
		uint16_t hdl, off, len;
		memcpy(&hdl, buf + pos, 2);
		memcpy(&off, buf + pos + 2, 2);
		memcpy(&len, buf + pos + 4, 2);

		// Pull any following same-handle chunks up against this chunk's data,
		// dropping their 6-byte record headers, to form one contiguous value.
		uint8_t  *dst    = buf + pos + 6 + len;
		uint16_t  totLen = len;
		uint16_t  next   = pos + 6 + len;

		while (next + 6 <= total)
		{
			uint16_t nhdl, nlen;
			memcpy(&nhdl, buf + next, 2);
			if (nhdl != hdl)
			{
				break;
			}
			memcpy(&nlen, buf + next + 4, 2);
			memmove(dst, buf + next + 6, nlen);
			dst    += nlen;
			totLen += nlen;
			next   += 6 + nlen;
		}

		BtAttDBEntry_t *entry = BtAttDBFindHandle(hdl);
		if (entry != nullptr)
		{
			BtAttWriteValueForConn(pConn->Conn.Hdl, entry, off, buf + pos + 6, totLen);
		}

		pos = next;
	}

	pConn->Conn.LongWrLen = 0;
}

uint32_t BtAttProcessReq(uint16_t ConnHdl, BtAttReqRsp_t * const pReqAtt, int ReqLen, BtAttReqRsp_t * const pRspAtt)
{
	uint32_t retval = 0;

	DEBUG_PRINTF("ATT OpCode %x, L2Cap len %d\n", pReqAtt->OpCode, ReqLen);

	// Reject truncated request PDUs before any field is parsed. ReqLen counts
	// the opcode byte plus all parameters; a PDU shorter than the fixed size
	// required by its opcode would read handles/lengths past the end of the
	// received L2CAP buffer. PDUs arrive over the air from an untrusted peer.
	{
		int minLen = 1;
		switch (pReqAtt->OpCode)
		{
			case BT_ATT_OPCODE_ATT_EXCHANGE_MTU_REQ:		minLen = 3; break;	// op + RxMtu(2)
			case BT_ATT_OPCODE_ATT_FIND_INFORMATION_REQ:	minLen = 5; break;	// op + start(2) + end(2)
			case BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ:	minLen = 7; break;	// op + start(2) + end(2) + type(2)
			case BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ:		minLen = 7; break;	// op + start(2) + end(2) + uuid16(2)
			case BT_ATT_OPCODE_ATT_READ_REQ:				minLen = 3; break;	// op + hdl(2)
			case BT_ATT_OPCODE_ATT_READ_BLOB_REQ:			minLen = 5; break;	// op + hdl(2) + offset(2)
			case BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ:		minLen = 5; break;	// op + at least 2 handles
			case BT_ATT_OPCODE_ATT_READ_BY_GROUP_TYPE_REQ:	minLen = 7; break;	// op + start(2) + end(2) + uuid16(2)
			case BT_ATT_OPCODE_ATT_WRITE_REQ:				minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_CMD:						minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:		minLen = 5; break;	// op + hdl(2) + offset(2)
			case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ:		minLen = 2; break;	// op + flags(1)
			case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:		minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:		minLen = 3; break;	// op + hdl(2) + value(>=0)
			case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:		minLen = 1; break;	// op only
			default:										minLen = 1; break;
		}

		if (ReqLen < minLen)
		{
			return BtAttError(pRspAtt, 0, pReqAtt->OpCode, BT_ATT_ERROR_INVALID_PDU);
		}
	}

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

				uint16_t negMtu = BtAttSetMtu(pReqAtt->ExchgMtuReqRsp.RxMtu);
				pRspAtt->ExchgMtuReqRsp.RxMtu = negMtu;

				// Record the negotiated MTU on the link so server-initiated PDUs
				// (notifications/indications) are sized to it, mirroring the
				// client-side path in bt_attrsp.cpp. s_AttMtu still drives this
				// handler's response sizing until the per-connection MTU refactor
				// noted at BtAttSetMtu lands.
				BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);
				if (pConn != nullptr)
				{
					pConn->Conn.MaxMtu = negMtu;
				}

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
				BtAttFindByTypeValueReq_t *req =
						(BtAttFindByTypeValueReq_t*)&pReqAtt->FindByTypeValueReq;
				int valLen = ReqLen - 7; // opcode + start + end + type

				if (req->StartHdl < 1 || req->EndHdl < 1 || req->StartHdl > req->EndHdl)
				{
					retval = BtAttError(pRspAtt, req->StartHdl,
										BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ,
										BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				BtUuid16_t uid16 = { 0, BT_UUID_TYPE_16, req->Type };
				BtAttHdlRange_t *pOut = pRspAtt->FindByTypeValueRsp.Hdl;
				uint16_t start = req->StartHdl;
				int l = 0;

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_RSP;

				while (start <= req->EndHdl &&
					   (size_t)(1 + l) + sizeof(BtAttHdlRange_t) <= s_AttMtu)
				{
					uint16_t hdlStart = start;
					uint16_t hdlEnd   = req->EndHdl;
					BtAttDBEntry_t *entry = BtAttDBFindHdlRange(&uid16, &hdlStart, &hdlEnd);

					if (entry == nullptr)
						break;

					if (hdlEnd > req->EndHdl)
						hdlEnd = req->EndHdl;

					uint8_t val[BT_ATT_MTU_MAX];
					size_t rlen = BtAttReadValueForConn(ConnHdl, entry, 0, val, sizeof(val));

					if (rlen == (size_t)valLen &&
						(valLen == 0 || memcmp(val, req->Val, (size_t)valLen) == 0))
					{
						pOut->StartHdl = entry->Hdl;
						pOut->EndHdl   = hdlEnd;
						pOut++;
						l += sizeof(BtAttHdlRange_t);
					}

					if (hdlEnd >= req->EndHdl || hdlEnd == 0xFFFF)
						break;

					start = hdlEnd + 1;
				}

				if (l > 0)
					retval = 1 + l;
				else
					retval = BtAttError(pRspAtt, req->StartHdl,
										BT_ATT_OPCODE_ATT_FIND_BY_TYPE_VALUE_REQ,
										BT_ATT_ERROR_ATT_NOT_FOUND);
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
					pRspAtt->ReadByTypeRsp.Len = BtAttReadValueForConn(ConnHdl, entry, 0, p, s_AttMtu - 2) + 2;

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

					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, entry->Hdl,
											BT_ATT_OPCODE_ATT_READ_BY_TYPE_REQ,
											err);
						break;
					}

					int cnt = BtAttReadValueForConn(ConnHdl, entry, 0, p, s_AttMtu - l - 2) + 2;
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

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_RSP;

				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);

				if (entry)
				{
					DEBUG_PRINTF("Entry with Hdl = %d found\r\n", req->Hdl);
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, req->Hdl,
											BT_ATT_OPCODE_ATT_READ_REQ, err);
					}
					else
					{
						retval = BtAttReadValueForConn(ConnHdl, entry, 0, pRspAtt->ReadRsp.Data,
												s_AttMtu - 1)  + 1;
					}
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
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadBlobReq.Hdl,
											BT_ATT_OPCODE_ATT_READ_BLOB_REQ, err);
					}
					else
					{
						retval = BtAttReadValueForConn(ConnHdl, entry, pReqAtt->ReadBlobReq.Offset,
												pRspAtt->ReadBlobRsp.Data, s_AttMtu - 1) + 1;
					}
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
				for (int i = 0; i < nhdl && (s_AttMtu - retval) >= BT_ATT_MTU_MIN; i++)
				{
					BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->ReadMultipleReq.Hdl[i]);
					if (entry == nullptr)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleReq.Hdl[i], BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
						break;
					}
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleReq.Hdl[i],
											BT_ATT_OPCODE_ATT_READ_MULTIPLE_REQ, err);
						break;
					}

					// Cap each value at the space remaining in the response
					// buffer (MTU - bytes already written), not the full MTU,
					// so cumulative values cannot overrun the response.
					int l = BtAttReadValueForConn(ConnHdl, entry, 0, p, s_AttMtu - retval);
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

						int cnt = BtAttReadValueForConn(ConnHdl, entry, 0, p, s_AttMtu - l - sizeof(BtAttHdlRange_t));

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
					// ReqLen counts opcode(1) + handle(2) + data, so the data
					// length is ReqLen - 3. Passing ReqLen directly overruns the
					// value buffer by 3 bytes (handle + opcode worth of trailing
					// memory), corrupting the stored value.
					int dlen = ReqLen - 3;
					if (dlen < 0)
					{
						dlen = 0;
					}

					// A Client Characteristic Configuration descriptor is a fixed
					// 2-octet field. A write of any other length is rejected with
					// Invalid Attribute Value Length (Core spec Vol 3 Part G,
					// 3.3.3.3), not silently truncated or accepted.
					if (entry->TypeUuid.BaseIdx == 0 &&
						entry->TypeUuid.Uuid == BT_UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION &&
						dlen != 2)
					{
						retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, BT_ATT_ERROR_INVALID_ATT_VALUE);
						break;
					}

					uint8_t err = BtAttWritePermError(ConnHdl, entry,
													  BT_ATT_OPCODE_ATT_WRITE_REQ,
													  req->Data, (uint16_t)dlen);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_WRITE_REQ, err);
						break;
					}

					BtAttWriteValueForConn(ConnHdl, entry, 0, req->Data, (uint16_t)dlen);

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

				// Write Command (opcode 0x52) uses BtAttWriteCmd_t (handle + data).
				// SignedWriteCmd is opcode 0xD2 (handle + data + 12B signature).
				// Both union members overlay so the data offset is identical,
				// but the field name should match the opcode being handled.
				BtAttDBEntry_t *entry = BtAttDBFindHandle(pReqAtt->WriteCmd.Hdl);

				if (entry)
				{
					// ReqLen counts opcode + handle + data, so the data length
					// is ReqLen - 1 (opcode) - 2 (handle) = ReqLen - 3.
					int dlen = ReqLen - 3;
					if (dlen < 0)
					{
						dlen = 0;
					}
					if (BtAttWritePermError(ConnHdl, entry, BT_ATT_OPCODE_ATT_CMD,
											pReqAtt->WriteCmd.Data, (uint16_t)dlen) == 0)
					{
						BtAttWriteValueForConn(ConnHdl, entry, 0, pReqAtt->WriteCmd.Data, (uint16_t)dlen);
					}

					retval = 0;
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ:
			{
				BtAttPrepareWriteReq_t *req = &pReqAtt->PrepareWriteReq;
				BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

				// Value bytes = total PDU - opcode(1) - handle(2) - offset(2).
				int vlen = ReqLen - 5;
				if (vlen < 0)
				{
					vlen = 0;
				}

				if (pConn == nullptr || pConn->Conn.pLongWrBuff == nullptr)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, BT_ATT_ERROR_PREPARE_QUE_FULL);
					break;
				}
				BtAttDBEntry_t *entry = BtAttDBFindHandle(req->Hdl);
				if (entry == nullptr)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, BT_ATT_ERROR_INVALID_HANDLE);
					break;
				}

				uint8_t err = BtAttWritePermError(ConnHdl, entry,
												  BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ,
												  req->Data, (uint16_t)vlen);
				if (err != 0)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, err);
					break;
				}

				// Append a { Hdl, Offset, Len, Data } record to the per-link
				// prepare queue. The 6-byte header keeps it self-describing so
				// the execute step can walk and reassemble it.
				uint16_t vl   = (uint16_t)vlen;
				uint32_t need = 6 + (uint32_t)vl;
				if ((uint32_t)pConn->Conn.LongWrLen + need > pConn->Conn.LongWrBuffSize)
				{
					retval = BtAttError(pRspAtt, req->Hdl, BT_ATT_OPCODE_ATT_PREPARE_WRITE_REQ, BT_ATT_ERROR_PREPARE_QUE_FULL);
					break;
				}

				uint8_t *q = pConn->Conn.pLongWrBuff + pConn->Conn.LongWrLen;
				memcpy(q,     &req->Hdl,    2);
				memcpy(q + 2, &req->Offset, 2);
				memcpy(q + 4, &vl,          2);
				memcpy(q + 6, req->Data,    vl);
				pConn->Conn.LongWrLen += (uint16_t)need;

				// Echo the request back as the Prepare Write Response.
				pRspAtt->OpCode               = BT_ATT_OPCODE_ATT_PREPARE_WRITE_RSP;
				pRspAtt->PrepareWriteRsp.Hdl    = req->Hdl;
				pRspAtt->PrepareWriteRsp.Offset = req->Offset;
				memcpy(pRspAtt->PrepareWriteRsp.Data, req->Data, vl);
				retval = 1 + 2 + 2 + vl;	// opcode + handle + offset + value
			}
			break;
		case BT_ATT_OPCODE_ATT_EXECUTE_WRITE_REQ:
			{
				BtDevice_t *pConn = BtPeerFindByHdl(ConnHdl);

				if (pReqAtt->ExecuteWriteReq.Flag != 0)
				{
					// Execute: apply the queued writes, firing each char WrCB.
					BtAttExecLongWrite(pConn);
				}
				else if (pConn != nullptr)
				{
					// Cancel: discard the queue without applying it.
					pConn->Conn.LongWrLen = 0;
				}

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_EXECUTE_WRITE_RSP;
				retval = 1;	// opcode only
			}
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
					uint8_t err = BtAttReadPermError(ConnHdl, entry);
					if (err != 0)
					{
						retval = BtAttError(pRspAtt, *hdl,
											BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ, err);
						break;
					}

					pRspAtt->OpCode = BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_RSP;
					uint16_t n = BtAttReadValueForConn(ConnHdl, entry, 0, p + 2, s_AttMtu - l - 2);
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
					retval = BtAttError(pRspAtt, pReqAtt->ReadMultipleVarReq.Hdl[0], BT_ATT_OPCODE_ATT_READ_MULTIPLE_VARIABLE_REQ, BT_ATT_ERROR_ATT_NOT_FOUND);
				}
			}
			break;
		case BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF:
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF:
			{
				// Server notified us (client role). Deliver the value to the
				// application. PDU is opcode(1) + value handle(2) + value.
				int dlen = ReqLen - 3;
				if (dlen < 0)
				{
					dlen = 0;
				}
				BtGattClientNotified(ConnHdl, pReqAtt->HandleValueNtf.ValHdl,
									 pReqAtt->HandleValueNtf.Data, (uint16_t)dlen);
			}
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND:
			{
				// We are the client and the server indicated us. ATT requires a
				// Handle Value Confirmation in reply before the server may send
				// another indication (Core spec Vol 3 Part F, 3.4.7.2). The
				// confirmation is the opcode alone, no parameters. Deliver the
				// value first, then confirm.
				int dlen = ReqLen - 3;
				if (dlen < 0)
				{
					dlen = 0;
				}
				BtGattClientNotified(ConnHdl, pReqAtt->HandleValueInd.Hdl,
									 pReqAtt->HandleValueInd.Data, (uint16_t)dlen);

				pRspAtt->OpCode = BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM;
				retval = 1;
			}
			break;
		case BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:
			{
				//DEBUG_PRINTF("BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM:\r\n");
				// Peer confirmed our indication; release the per-link slot so
				// the next indication on this connection can be sent.
				BtGattHandleValueConfirm(ConnHdl);
			}
			break;
		case BT_ATT_OPCODE_ATT_SIGNED_WRITE_CMD:
			break;
		default:
			//DEBUG_PRINTF("OpCode : %x\r\n", pReqAtt->OpCode);
			// ATT spec: a server that receives a Request it does not support
			// MUST answer with an Error Response (Request Not Supported).
			// Staying silent leaves the client waiting and the link idles out.
			// Commands (bit6=0x40), notifications/indications/confirmations and
			// responses do not get a reply. A request is anything else with an
			// even-ish method that expects a response; treat unknown opcodes
			// that are not command/notify/confirm/response as requests.
			{
				uint8_t op = pReqAtt->OpCode;
				bool isCommand = (op & 0x40) != 0;	// Write Command, Signed Write
				bool noRsp = isCommand ||
							 op == BT_ATT_OPCODE_ATT_HANDLE_VALUE_NTF ||
							 op == BT_ATT_OPCODE_ATT_HANDLE_VALUE_IND ||
							 op == BT_ATT_OPCODE_ATT_HANDLE_VALUE_CFM ||
							 op == BT_ATT_OPCODE_ATT_MULTIPLE_HANDLE_VALUE_NTF ||
							 op == BT_ATT_OPCODE_ATT_ERROR_RSP ||
							 (op & 0x01);	// odd opcodes in 1..0x1B are responses
				if (!noRsp)
				{
					retval = BtAttError(pRspAtt, 0x0000, op,
										 BT_ATT_ERROR_REQUEST_NOT_SUPP);
				}
			}
			break;
	}

	return retval;
}

