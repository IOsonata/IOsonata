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
alignas(4) static int s_NbUuidEntry = 1;

/**
 * @brief	Find index of stored base UUID 128 bits
 *
 * This function lookup the internal table for a 128 bits UUID.
 *
 * @param 	Uuid : The 128 bits UUID to find
 *
 * @return	The index in the internal UUID table
 * 			-1 : Not found
\ */
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

/**
 * @brief	Add new 128 bit base UUID into the internal table
 *
 * If UUID already exits, returns its index. Otherwise add to available slot.
 *
 * @param	Uuid :	128 bits UUID to add
 *
 * @return	The index in the internal UUID table
 * 			-1 : Table full, cannot add
 */
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

			if (s_NbUuidEntry < i)
			{
				s_NbUuidEntry = i;
			}
			return i;
		}
	}

	return -1;
}

/**
 * @brief	Get the base 128 bits UUID
 *
 * Find and return to base 128 UUID located at index location in the internal table
 *
 * @param	Idx		: Index into the base UUID table.
 * @param 	Uuid 	: Buffer to store the UUID found
 *
 * @return	true - UUID found
 * 			false - UUID not found
 */
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

/**
 * @brief	Convert the BtUuid_t type to 128 bits UUID
 *
 * @param 	pUuid	: Reference to the BtUuid_t to convert
 * @param 	Uuid128	: Buffer to store the 128 bits UUID result
 *
 * @return	true - success
 */
bool BtUuidTo128(BtUuid_t * const pUuid, uint8_t Uuid128[16])
{
	BtBaseUuidTblEntry_t *p = &s_BtBaseUuidTbl[pUuid->BaseIdx];

	if (p->bValid == true)
	{
		memcpy(Uuid128, p->Uuid, 16);
		Uuid128[12] = pUuid->Uuid16 & 0xFF;
		Uuid128[13] = (pUuid->Uuid16 >> 8) & 0xFF;

		if (pUuid->Type == BT_UUID_TYPE_32)
		{
			Uuid128[14] = (pUuid->Uuid32 >> 16) & 0xFF;
			Uuid128[15] = (pUuid->Uuid32 >> 24) & 0xFF;
		}
	}

	return p->bValid;
}

/**
 * @brief	Convert the BtUuid16_t 16 bits type to 128 bits UUID
 *
 * @param 	pUuid	: Reference to the BtUuid16_t to convert
 * @param 	Uuid128	: Buffer to store the 128 bits UUID result
 *
 * @return	true - success
 */
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

/**
 * @brief	Convert the BtUuid32_t 32 bits type to 128 bits UUID
 *
 * @param 	pUuid	: Reference to the BtUuid32_t to convert
 * @param 	Uuid128	: Buffer to store the 128 bits UUID result
 *
 * @return	true - success
 */
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
