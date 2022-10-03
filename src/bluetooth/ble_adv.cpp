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

typedef struct __Adv_Data {
	int Len;
	uint8_t *pData;
} AdvData_t;

alignas(4) static uint8_t s_BleAdvBuff[32];
alignas(4) static uint8_t s_BleScanRespBuff[32];
alignas(4) static uint8_t s_BleExtAdvBuff[256];

static AdvData_t s_BleAdvData[] = {
	{ 0, s_BleAdvBuff},
	{ 0, s_BleScanRespBuff},
	{ 0, s_BleExtAdvBuff},
};

static int FindAdvTag(uint8_t Tag, uint8_t *pData, int Len)
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
 * @brief
 *
 * @param Type
 * @param pData
 * @param Len
 * @return
 */
int BleAdvSetAdvData(uint8_t Type, uint8_t *pData, int Len)
{
	int retval = 0;

	int idx = FindAdvTag(Type, s_BleAdvData[0].pData, s_BleAdvData[0].Len);

	if (idx >= 0)
	{
		BleAdvData_t *p = (BleAdvData_t*)&s_BleAdvData[0].pData[idx];
		int l = Len;

		if (p->Hdr.Len > (Len + 1))
		{
			memmove(&s_BleAdvData[0].pData[idx + 2 + Len], &s_BleAdvData[0].pData[idx + 1 + p->Hdr.Len], 30 - p->Hdr.Len);
		}
		else if (p->Hdr.Len < (Len + 1))
		{
			l = min(31 - idx - 2 - Len, Len + 2);
			memmove(&s_BleAdvData[0].pData[idx + 2 + l], &s_BleAdvData[0].pData[idx + 1 + p->Hdr.Len], 30 - p->Hdr.Len);

		}
		memcpy(p->Data, pData, l);
	}
	else
	{
		int l = min(31 - s_BleAdvData[0].Len, Len + 2);
		BleAdvData_t *p = (BleAdvData_t*)&s_BleAdvData[0].pData[s_BleAdvData[0].Len];
		p->Hdr.Len = Len + 1;
		p->Hdr.Type = Type;
		memcpy(p->Data, pData, l);
		s_BleAdvData[0].Len += l;
	}

	return retval;
}

int BleAdvGetAdvData(uint8_t *pBuff, int Len)
{
	int l = min(Len, s_BleAdvData[0].Len);

	if (l > 0)
	{
		memcpy(pBuff, s_BleAdvData[0].pData, l);
	}

	return l;
}
