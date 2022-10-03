/**-------------------------------------------------------------------------
@file	ble_adv.cpp

@brief	Bluetooth advertisement generic implementation


@author	Hoang Nguyen Hoan
@date	Oct. 2, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#include <string.h>

#include "istddef.h"
#include "bluetooth/ble_adv.h"


static int BleAdvFindAdvTag(uint8_t Tag, uint8_t *pData, int Len)
{
	int retval = -1;
	BleAdvDataHdr_t *hdr = (BleAdvDataHdr_t*)pData;
	int idx = 0;

	while (Len > 0)
	{
		if (hdr->Type == Tag)
		{
			retval = idx;
			break;
		}
		idx += hdr->Len;
		Len -= hdr->Len + 1;
		hdr = (BleAdvDataHdr_t*)&pData[idx];
	}

	return retval;
}

/**
 * @brief	Add advertisement data into the adv packet
 *
 * @param 	BleAdvPacket_t 	: Pointer to Adv packet to add data into
 * @param 	Type 				: GAP data type of the data
 * @param	pData				: Pointer to data to add
 * @param	Len				: Length in bytes of the data
 *
 * @return	true - success
 */
bool BleAdvAddData(BleAdvPacket_t *pAdvPkt, uint8_t Type, uint8_t *pData, int Len)
{
	int idx = BleAdvFindAdvTag(Type, pAdvPkt->pData, pAdvPkt->Len);

	if (idx >= 0)
	{
		// Tag already exists, remove it first
		BleAdvData_t *p = (BleAdvData_t*)&pAdvPkt->pData[idx];
		int l = pAdvPkt->Len - p->Hdr.Len - 1;

		if (Len > (pAdvPkt->MaxLen - l))
		{
			return false;
		}

		memmove(&pAdvPkt->pData[idx], &pAdvPkt->pData[idx + p->Hdr.Len + 1], l - idx);
		pAdvPkt->Len = l;
	}

	if (Len > (pAdvPkt->MaxLen - pAdvPkt->Len))
	{
		return false;
	}

	int l = min(pAdvPkt->MaxLen - pAdvPkt->Len, Len + 2);
	BleAdvData_t *p = (BleAdvData_t*)&pAdvPkt->pData[pAdvPkt->Len];
	p->Hdr.Len = Len + 1;
	p->Hdr.Type = Type;
	memcpy(p->Data, pData, l);
	pAdvPkt->Len += l;

	return true;
}

/**
 * @brief	Remove advertisement data from the adv packet
 *
 * @param 	pAdvPkt	: Pointer to Adv packet to add data into
 * @param 	Type 	: GAP data type of the data
 *
 * @return	none
 */
void BleAdvRemoveData(BleAdvPacket_t *pAdvPkt, uint8_t Type)
{
	if (pAdvPkt->Len <= 0)
		return;

	int idx = BleAdvFindAdvTag(Type, pAdvPkt->pData, pAdvPkt->Len);

	if (idx >= 0)
	{
		BleAdvData_t *p = (BleAdvData_t*)&pAdvPkt->pData[idx];
		int l = p->Hdr.Len + 1;

		memmove(&pAdvPkt->pData[idx], &pAdvPkt->pData[idx + l], pAdvPkt->Len - l);
	}
}
