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

#include "istddef.h"
#include "bluetooth/bt_gatt.h"

#ifndef BT_GATT_ENTRY_MAX_COUNT
#define BT_GATT_ENTRY_MAX_COUNT		100
#endif

alignas(4) static BtGattListEntry_t s_BtGatEntryTbl[BT_GATT_ENTRY_MAX_COUNT] = {0,};
static int s_NbGattListEntry = 0;

uint16_t BtGattRegister(BtUuid16_t *pTypeUuid, void *pAttVal)
{
	if (pAttVal == nullptr || s_NbGattListEntry >= BT_GATT_ENTRY_MAX_COUNT)
	{
		return -1;
	}

	BtGattListEntry_t *p = (BtGattListEntry_t*)&s_BtGatEntryTbl[s_NbGattListEntry];

	p->TypeUuid = *pTypeUuid;

	if (p->TypeUuid.BaseIdx == 0)
	{
		// Standard Bluetooth 128 bits based UUID
		switch (p->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				s_BtGatEntryTbl[s_NbGattListEntry].SrvcDeclar = *(BtGattSrvcDeclar_t *)pAttVal;
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				memcpy(&s_BtGatEntryTbl[s_NbGattListEntry].SrvcInc, pAttVal, sizeof(BtGattSrvcInclude_t));
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				memcpy(&s_BtGatEntryTbl[s_NbGattListEntry].CharDeclar, pAttVal, sizeof(BtGattCharDeclar_t));
				s_BtGatEntryTbl[s_NbGattListEntry].CharDeclar.ValHdl = s_NbGattListEntry + 2;
				s_BtGatEntryTbl[s_NbGattListEntry].Hdl = s_NbGattListEntry + 1;
				//s_NbGattListEntry++;
				//s_BtGatEntryTbl[s_NbGattListEntry].TypeUuid = ((BtGattCharDeclar_t*)pAttVal)->Uuid;
				//((BtGattCharDeclar_t*)pAttVal)->ValHdl = s_NbGattListEntry + 1;
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				s_BtGatEntryTbl[s_NbGattListEntry].Val32 = *(uint8_t*)pAttVal;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;

			default:
				memcpy(&s_BtGatEntryTbl[s_NbGattListEntry].CharVal, pAttVal, sizeof(BtGattCharValue_t));
		}
	}
	else
	{
		memcpy(&s_BtGatEntryTbl[s_NbGattListEntry].CharVal, pAttVal, sizeof(BtGattCharValue_t));
/*		BtGattCharValHandler_t *p = (BtGattCharValHandler_t*)pAttVal;

		s_BtGatEntryTbl[s_NbGattListEntry].ValHandler.RdHandler = p->RdHandler;
		s_BtGatEntryTbl[s_NbGattListEntry].ValHandler.WrHandler = p->WrHandler;
		s_BtGatEntryTbl[s_NbGattListEntry].ValHandler.pCtx = p->pCtx;*/

//		s_BtGatEntryTbl[s_NbGattListEntry].pCharVal = (BtGattCharValue_t*)pAttVal;
	}
	s_BtGatEntryTbl[s_NbGattListEntry].Hdl = s_NbGattListEntry + 1;

	s_NbGattListEntry++;

	return s_NbGattListEntry;
}

bool BtGattUpdate(uint16_t Hdl, void *pAttVal, size_t Len)
{
	if (Hdl <= 0 || Hdl > s_NbGattListEntry)
	{
		return false;
	}

	BtGattListEntry_t *p = (BtGattListEntry_t*)&s_BtGatEntryTbl[Hdl - 1];

	if (p->TypeUuid.BaseIdx == 0)
	{
		switch (p->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				memcpy(&p->SrvcDeclar, pAttVal, sizeof(BtGattSrvcDeclar_t));
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				memcpy(&p->SrvcInc, pAttVal, sizeof(BtGattSrvcInclude_t));
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				memcpy(&p->CharDeclar, pAttVal, sizeof(BtGattCharDeclar_t));
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				p->Val32 = *(uint8_t*)pAttVal;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				memcpy(&p->CharVal, pAttVal, sizeof(BtGattCharValue_t));
		}
	}
	else
	{
		memcpy(&p->CharVal, pAttVal, sizeof(BtGattCharValue_t));
	}

	return true;
}

int BtGattGetListHandle(uint16_t StartHdl, uint16_t EndHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl)
{
	int idx = 0;

	for (int i = StartHdl; i < min(EndHdl + 1, s_NbGattListEntry) && idx < MaxEntry; i++)
	{
		memcpy(&pArr[idx], &s_BtGatEntryTbl[i - 1], sizeof(BtGattListEntry_t));
		idx++;
	}

	if (pLastHdl && s_NbGattListEntry > 0)
	{
		*pLastHdl = s_BtGatEntryTbl[s_NbGattListEntry - 1].Hdl;
	}

	return idx;
}

int BtGattGetListUuid(BtUuid16_t *pTypeUuid, uint16_t StartHdl, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl)
{
	int idx = 0;

	for (int i = StartHdl - 1; i < s_NbGattListEntry && idx < MaxEntry; i++)
	{
		if (memcmp(&s_BtGatEntryTbl[i].TypeUuid, pTypeUuid, sizeof(BtUuid16_t)) == 0)
		{
			pArr[idx] = s_BtGatEntryTbl[i];
			idx++;
		}
	}

	if (pLastHdl && s_NbGattListEntry > 0)
	{
		*pLastHdl = s_BtGatEntryTbl[s_NbGattListEntry - 1].Hdl;
	}

	return idx;
}

bool BtGattGetEntryHandle(uint16_t Hdl, BtGattListEntry_t *pEntry)
{
	if (Hdl <= 0 || Hdl > s_NbGattListEntry)
	{
		return false;
	}

	*pEntry = s_BtGatEntryTbl[Hdl - 1];

	return true;
}

size_t BtGattGetValue(BtGattListEntry_t *pEntry, uint8_t *pBuff)
{
	if (pBuff == nullptr)
	{
		return 0;
	}

	size_t len = 0;
	//uint8_t uuid128[16];

	if (pEntry->TypeUuid.BaseIdx == 0)
	{
		switch (pEntry->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				if (pEntry->SrvcDeclar.Uuid.BaseIdx > 0)
				{
					BtUuidGetBase(pEntry->SrvcDeclar.Uuid.BaseIdx, pBuff);

					pBuff[12] = pEntry->SrvcDeclar.Uuid.Uuid & 0xFF;
					pBuff[13] = pEntry->SrvcDeclar.Uuid.Uuid >> 8;

					len = 16;
				}
				else
				{
					pBuff[0] = pEntry->SrvcDeclar.Uuid.Uuid & 0xFF;
					pBuff[1] = pEntry->SrvcDeclar.Uuid.Uuid >> 8;

					len = 2;
				}
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				len = pEntry->SrvcInc.SrvcUuid.BaseIdx > 0 ? 20 : 6;
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				pBuff[0] = pEntry->Hdl & 0xFF;
				pBuff[1] = pEntry->Hdl >> 8;
				pBuff += 2;
				*pBuff++ = pEntry->CharDeclar.Prop;
				pBuff[0] = pEntry->CharDeclar.ValHdl & 0xFF;
				pBuff[1] = pEntry->CharDeclar.ValHdl >> 8;
				pBuff += 2;
				if (pEntry->CharDeclar.Uuid.BaseIdx > 0)
				{
					BtUuidGetBase(pEntry->CharDeclar.Uuid.BaseIdx, pBuff);

					pBuff[12] = pEntry->CharDeclar.Uuid.Uuid & 0xFF;
					pBuff[13] = pEntry->CharDeclar.Uuid.Uuid >> 8;

					len = 21;
				}
				else
				{
					pBuff[0] = pEntry->CharDeclar.Uuid.Uuid & 0xFF;
					pBuff[1] = pEntry->CharDeclar.Uuid.Uuid >> 8;

					len = 7;
				}

				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				pBuff[0] = pEntry->Val32 & 0xFF;
				len = 1;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				if (pEntry->CharVal.pData)
				{
					memcpy(pBuff, pEntry->CharVal.pData, pEntry->CharVal.Len);
					len = pEntry->CharVal.Len;
				}

		}
	}
	else
	{
		if (pEntry->CharVal.pData)
		{
			memcpy(pBuff, pEntry->CharVal.pData, pEntry->CharVal.Len);
			len = pEntry->CharVal.Len;
		}
		/*
		if (pEntry->ValHandler.RdHandler)
		{
			len = pEntry->ValHandler.RdHandler(pEntry->Hdl, pBuff, 10, pEntry->ValHandler.pCtx);
		}
*/
//		len = pEntry->pCharVal->Len;
//		memcpy(pBuff, pEntry->pCharVal->pData, len);
	}

	return len;
}

size_t BtGattWriteValue(uint16_t Hdl, uint8_t *pBuff, size_t Len)
{
	BtGattListEntry_t *p = &s_BtGatEntryTbl[Hdl - 1];

	size_t len = 0;

	if (p->TypeUuid.BaseIdx == 0)
	{
		switch (p->TypeUuid.Uuid)
		{
			case BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE:
			case BT_UUID_GATT_DECLARATIONS_SECONDARY_SERVICE:
				break;
			case BT_UUID_GATT_DECLARATIONS_INCLUDE:
				break;
			case BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_EXTENDED_PROPERTIES:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CHARACTERISTIC_USER_DESCRIPTION:
				break;
			case BT_UUID_GATT_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION:
				p->Val32 = *pBuff;
				len = 1;
				break;
			case BT_UUID_GATT_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION:
				break;
			default:
				if (p->CharVal.WrHandler)
				{
					len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
				}

		}
	}
	else
	{
		if (p->CharVal.WrHandler)
		{
			len = p->CharVal.WrHandler(p->Hdl, pBuff, Len, p->CharVal.pCtx);
		}
	}

	return len;
}

BtGattListEntry_t *GetEntryTable(size_t *count)
{
	*count = s_NbGattListEntry;
	return s_BtGatEntryTbl;
}

#if 0
typedef struct __Bt_Att_xx {
	uint16_t StartHdl;
	uint16_t EndHdl;
} BtAttxx_t;

typedef struct __Bt_Att_Uuid16 {
	BtAttxx_t Hdl;
	uint16_t Uuid;
} BtAttUuid16_t;

typedef struct __Bt_Att_Uuid128 {
	BtAttxx_t Hdl;
	uint8_t Uuid[16];
} BtAttUuid128_t;

uint32_t BtGattGetEntry(uint16_t GattUuid, uint8_t *pBuff)
{
	uint16_t *p = (uint16_t *)pBuff;
	int x = 0;
	int hidx = 0;
	bool start = true;

	for (int i = 0; i < s_NbGattListEntry; i++)
	{
		if (s_BtGatEntryTbl[s_NbGattListEntry].GattUuid == GattUuid)
		{
			if (start == true)
			{
				start = false;

				if (s_BtGatEntryTbl[i].Uuid.BaseIdx != 0)
				{
					BtAttUuid128_t *x = (BtAttUuid128_t*)p;

				}
			}
			p[x + hidx] = s_BtGatEntryTbl[i].Hdl;
			p[x+2] = s_BtGatEntryTbl[i].Uuid.Uuid;
		}
	}

	return 0;
}
#endif

