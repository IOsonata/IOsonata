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

#include "bluetooth/bt_gatt.h"

#ifndef BT_GATT_ENTRY_MAX_COUNT
#define BT_GATT_ENTRY_MAX_COUNT		100
#endif

alignas(4) static BtGattListEntry_t s_BtGatEntryTbl[BT_GATT_ENTRY_MAX_COUNT] = {0,};
static uint32_t s_NbGattListEntry = 0;

#if 0
uint32_t BtGattDeclarPrimSrvc(BtUuid16_t *pUuid)
{
	if (pUuid == nullptr || s_NbGattListEntry >= BT_GATT_ENTRY_MAX_COUNT)
	{
		return -1;
	}

	s_BtGatEntryTbl[s_NbGattListEntry].GattUuid = BT_UUID_GATT_DECLARATIONS_PRIMARY_SERVICE;
	memcpy(&s_BtGatEntryTbl[s_NbGattListEntry].Uuid, pUuid, sizeof(BtUuid16_t));
	s_BtGatEntryTbl[s_NbGattListEntry++].Hdl = s_NbGattListEntry + 1;

	s_NbGattListEntry++;

	return s_NbGattListEntry;
}

uint32_t BtGattDeclarChar(BtUuid16_t *pUuid)
{
	if (pUuid == nullptr || s_NbGattListEntry >= BT_GATT_ENTRY_MAX_COUNT)
	{
		return -1;
	}

	s_BtGatEntryTbl[s_NbGattListEntry].GattUuid = BT_UUID_GATT_DECLARATIONS_CHARACTERISTIC;
	memcpy(&s_BtGatEntryTbl[s_NbGattListEntry].Uuid, pUuid, sizeof(BtUuid16_t));
	s_BtGatEntryTbl[s_NbGattListEntry++].Hdl = s_NbGattListEntry + 1;

	s_NbGattListEntry++;

	return s_NbGattListEntry;
}
#endif

uint16_t BtGattRegister(uint16_t GatUuid, BtUuid16_t *pUuid)
{
	if (pUuid == nullptr || s_NbGattListEntry >= BT_GATT_ENTRY_MAX_COUNT)
	{
		return -1;
	}

	s_BtGatEntryTbl[s_NbGattListEntry].GattUuid = GatUuid;
	memcpy(&s_BtGatEntryTbl[s_NbGattListEntry].Uuid, pUuid, sizeof(BtUuid16_t));
	s_BtGatEntryTbl[s_NbGattListEntry].Hdl = s_NbGattListEntry + 1;

	s_NbGattListEntry++;

	return s_NbGattListEntry;
}


int BtGattGetList(uint16_t GattUuid, BtGattListEntry_t *pArr, int MaxEntry, uint16_t *pLastHdl)
{
	int idx = 0;

	for (int i = 0; i < s_NbGattListEntry && idx < MaxEntry; i++)
	{
		if (s_BtGatEntryTbl[i].GattUuid == GattUuid)
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

bool BtGattGetEntryHandle(uint16_t Hdl, BtGattListEntry_t *pArr)
{
	if (Hdl <= 0 || Hdl > s_NbGattListEntry)
	{
		return false;
	}

	*pArr = s_BtGatEntryTbl[Hdl - 1];

	return true;
}

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
