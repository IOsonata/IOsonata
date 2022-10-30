/**-------------------------------------------------------------------------
@file	bt_uuid.cpp

@brief	Bluetooth UUID implementation

Generic implementation & definitions of Bluetooth UUID

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

#include "bluetooth/bt_uuid.h"

// Bluetooth SIG base uuid
// 0000FF00-0000-1000-8000-00805F9B34FB
//
#define BLUETOOTH_SIG_BASE_UUID		{ 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, \
									  0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }


typedef struct __Bt_Base_Uuid_Tbl_Entry {
	uint8_t Uuid[16];
	bool bValid;
} BtBaseUuidTblEntry_t;

#ifndef BT_BASE_UUID_ENTRY_MAX_COUNT
#define BT_BASE_UUID_ENTRY_MAX_COUNT		4
#endif

alignas(4) static BtBaseUuidTblEntry_t s_BtBaseUuidTbl[BT_BASE_UUID_ENTRY_MAX_COUNT] = {{BLUETOOTH_SIG_BASE_UUID, true}, {0, false},};
static uint32_t s_NbUuidEntry = 1;

int BtUuidFindBase(uint8_t const Uuid[16])
{
	for (int i = 0; i < BT_BASE_UUID_ENTRY_MAX_COUNT; i++)
	{
		if (s_BtBaseUuidTbl[i].bValid == true)
		{
			if (memcmp(s_BtBaseUuidTbl[i].Uuid, Uuid, 16) == 0)
			{
				return i;
			}
		}
	}

	return -1;
}

int BtUuidAddBase(uint8_t const Uuid[16])
{
	int8_t idx = BtUuidFindBase(Uuid);

	if (idx >= 0)
	{
		return idx;
	}

	for (int i = 1; i < BT_BASE_UUID_ENTRY_MAX_COUNT; i++)
	{
		if (s_BtBaseUuidTbl[i].bValid == false)
		{
			s_BtBaseUuidTbl[i].bValid = true;
			memcpy(s_BtBaseUuidTbl[i].Uuid, Uuid, 16);

			return i;
		}
	}

	return -1;
}

bool BtUuidGetBase(int Idx, uint8_t Uuid[16])
{
	if (Idx < 0 || Idx >= BT_BASE_UUID_ENTRY_MAX_COUNT)
	{
		return false;
	}

	if (s_BtBaseUuidTbl[Idx].bValid)
	{
		memcpy(Uuid, s_BtBaseUuidTbl[Idx].Uuid, 16);

		return true;
	}

	return false;
}

bool BtUuidTo128(BtUuid_t * const pUuid, uint8_t Uuid128[16])
{
	BtBaseUuidTblEntry_t *p = &s_BtBaseUuidTbl[pUuid->BaseIdx];

	if (pUuid->Type == BT_UUID_TYPE_128)
	{
		memcpy(Uuid128, pUuid->Val.Uuid128, 16);
		return true;
	}

	if (p->bValid == true)
	{
		memcpy(Uuid128, p->Uuid, 16);
		Uuid128[12] = pUuid->Val.Uuid16 & 0xFF;
		Uuid128[13] = pUuid->Val.Uuid16 >> 8;

		if (pUuid->Type == BT_UUID_TYPE_32)
		{
			Uuid128[14] = (pUuid->Val.Uuid32 >> 16) & 0xFF;
			Uuid128[15] = (pUuid->Val.Uuid32 >> 24) & 0xFF;
		}
	}

	return p->bValid;
}

bool BtUuid16To128(BtUuid16_t * const pUuid, uint8_t Uuid128[16])
{
	BtBaseUuidTblEntry_t *p = &s_BtBaseUuidTbl[pUuid->BaseIdx];

	if (p->bValid == true)
	{
		memcpy(Uuid128, p->Uuid, 16);
		Uuid128[12] = pUuid->Uuid & 0xFF;
		Uuid128[13] = pUuid->Uuid >> 8;
	}

	return p->bValid;
}

bool BtUuid32To128(BtUuid16_t * const pUuid, uint8_t Uuid128[16])
{
	BtBaseUuidTblEntry_t *p = &s_BtBaseUuidTbl[pUuid->BaseIdx];

	if (p->bValid == true)
	{
		memcpy(Uuid128, p->Uuid, 16);
		Uuid128[12] = pUuid->Uuid & 0xFF;
		Uuid128[13] = pUuid->Uuid >> 8;

		if (pUuid->Type == BT_UUID_TYPE_32)
		{
			Uuid128[14] = (pUuid->Uuid >> 16) & 0xFF;
			Uuid128[15] = (pUuid->Uuid >> 24) & 0xFF;
		}
	}

	return p->bValid;
}
