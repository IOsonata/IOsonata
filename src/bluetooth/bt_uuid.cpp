/**-------------------------------------------------------------------------
@file	bt_uuid.cpp

@brief	Bluetooth UUID implementation

Generic implementation & definitions of Bluetooth UUID. Contains most of the
standard Bluetooth UUID values and utility management functions.
Maintain a table of stored base 128 bits UUID. The table size is defined by
BT_BASE_UUID_ENTRY_MAX_COUNT at compile time. Add this define at the compiler
preprocessor option to overwrite the default value when compiling the IOsonata library.
This has no effect if added to application firmware project setting, only for compiling
the IOsonata library.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2022

@license

MIT License

Copyright (c) 2022, I-SYST, all rights reserved

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

#include "bluetooth/bt_uuid.h"


typedef struct __Bt_Base_Uuid_Tbl_Entry {
	uint8_t Uuid[16];
	bool bValid;
} BtBaseUuidTblEntry_t;

#ifndef BT_BASE_UUID_ENTRY_MAX_COUNT
#define BT_BASE_UUID_ENTRY_MAX_COUNT		4
#endif

// BLUETOOTH_SIG_BASE_UUID includes its own braces for the Uuid[16] member.
// The remaining table entries are value-initialized (all zero, bValid false).
alignas(4) static BtBaseUuidTblEntry_t s_BtBaseUuidTbl[BT_BASE_UUID_ENTRY_MAX_COUNT] = {{BLUETOOTH_SIG_BASE_UUID, true},};

int BtUuidFindBase(uint8_t const Uuid[16])
{
	if (Uuid == nullptr)
	{
		return -1;
	}

	for (int i = 0; i < BT_BASE_UUID_ENTRY_MAX_COUNT; i++)
	{
		if (s_BtBaseUuidTbl[i].bValid &&
			memcmp(s_BtBaseUuidTbl[i].Uuid, Uuid, 16) == 0)
		{
			return i;
		}
	}

	return -1;
}

int BtUuidAddBase(uint8_t const Uuid[16])
{
	if (Uuid == nullptr)
	{
		return -1;
	}

	int idx = BtUuidFindBase(Uuid);
	if (idx >= 0)
	{
		return idx;
	}

	for (int i = 1; i < BT_BASE_UUID_ENTRY_MAX_COUNT; i++)
	{
		if (!s_BtBaseUuidTbl[i].bValid)
		{
			memcpy(s_BtBaseUuidTbl[i].Uuid, Uuid, 16);
			s_BtBaseUuidTbl[i].bValid = true;
			return i;
		}
	}

	return -1;
}

bool BtUuidGetBase(int Idx, uint8_t Uuid[16])
{
	if (Uuid == nullptr || Idx < 0 || Idx >= BT_BASE_UUID_ENTRY_MAX_COUNT ||
		!s_BtBaseUuidTbl[Idx].bValid)
	{
		return false;
	}

	memcpy(Uuid, s_BtBaseUuidTbl[Idx].Uuid, 16);
	return true;
}

bool BtUuidTo128(BtUuid_t * const pUuid, uint8_t Uuid128[16])
{
	if (pUuid == nullptr || Uuid128 == nullptr ||
		pUuid->BaseIdx >= BT_BASE_UUID_ENTRY_MAX_COUNT)
	{
		return false;
	}

	BtBaseUuidTblEntry_t *p = &s_BtBaseUuidTbl[pUuid->BaseIdx];
	if (!p->bValid)
	{
		return false;
	}

	memcpy(Uuid128, p->Uuid, 16);
	Uuid128[12] = pUuid->Uuid16 & 0xFF;
	Uuid128[13] = (pUuid->Uuid16 >> 8) & 0xFF;

	if (pUuid->Type == BT_UUID_TYPE_32)
	{
		Uuid128[14] = (pUuid->Uuid32 >> 16) & 0xFF;
		Uuid128[15] = (pUuid->Uuid32 >> 24) & 0xFF;
	}

	return true;
}

bool BtUuid16To128(BtUuid16_t * const pUuid, uint8_t Uuid128[16])
{
	if (pUuid == nullptr || Uuid128 == nullptr ||
		pUuid->BaseIdx >= BT_BASE_UUID_ENTRY_MAX_COUNT)
	{
		return false;
	}

	BtBaseUuidTblEntry_t *p = &s_BtBaseUuidTbl[pUuid->BaseIdx];
	if (!p->bValid)
	{
		return false;
	}

	memcpy(Uuid128, p->Uuid, 16);
	Uuid128[12] = pUuid->Uuid & 0xFF;
	Uuid128[13] = pUuid->Uuid >> 8;
	return true;
}

bool BtUuid32To128(BtUuid32_t * const pUuid, uint8_t Uuid128[16])
{
	if (pUuid == nullptr || Uuid128 == nullptr ||
		pUuid->BaseIdx >= BT_BASE_UUID_ENTRY_MAX_COUNT)
	{
		return false;
	}

	BtBaseUuidTblEntry_t *p = &s_BtBaseUuidTbl[pUuid->BaseIdx];
	if (!p->bValid)
	{
		return false;
	}

	memcpy(Uuid128, p->Uuid, 16);
	Uuid128[12] = pUuid->Uuid & 0xFF;
	Uuid128[13] = pUuid->Uuid >> 8;

	if (pUuid->Type == BT_UUID_TYPE_32)
	{
		Uuid128[14] = (pUuid->Uuid >> 16) & 0xFF;
		Uuid128[15] = (pUuid->Uuid >> 24) & 0xFF;
	}

	return true;
}

int BtUuid128To16(BtUuid16_t *pUuid16, uint8_t Uuid128[16])
{
	if (pUuid16 == nullptr || Uuid128 == nullptr)
	{
		return -1;
	}

	uint8_t base[16];
	memcpy(base, Uuid128, 16);

	uint16_t shortUuid = (uint16_t)base[12] | ((uint16_t)base[13] << 8);
	base[12] = 0;
	base[13] = 0;

	int idx = BtUuidFindBase(base);
	if (idx < 0)
	{
		idx = BtUuidAddBase(base);
	}

	pUuid16->Type = BT_UUID_TYPE_16;
	// Do not alias a vendor UUID onto the Bluetooth SIG base when the custom
	// table is full. The out-of-range index makes all conversion helpers reject
	// it until the application provides more table capacity.
	pUuid16->BaseIdx = (idx >= 0) ? (uint8_t)idx :
		(uint8_t)BT_BASE_UUID_ENTRY_MAX_COUNT;
	pUuid16->Uuid = shortUuid;

	return idx;
}
