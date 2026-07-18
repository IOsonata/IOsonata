/**-------------------------------------------------------------------------
@file	bt_oob_rftag.cpp

@brief	Bluetooth LE Secure Connections OOB over an NFC tag

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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

#include "bluetooth/bt_oob_rftag.h"

#define AD_TYPE_LOCAL_NAME			0x09
#define AD_TYPE_LE_ADDR				0x1B
#define AD_TYPE_LE_ROLE				0x1C
#define AD_TYPE_LESC_CONFIRM		0x22
#define AD_TYPE_LESC_RANDOM			0x23

static const char s_LeOobMime[] = "application/vnd.bluetooth.le.oob";

// Append one AD structure: length, type, data. Length covers type plus data.
static int AdAppend(uint8_t *pBuf, size_t Size, size_t Off, uint8_t Type,
                    const uint8_t *pData, size_t Len)
{
	if (Off + 2 + Len > Size || Len > 254)
	{
		return -1;
	}

	pBuf[Off] = (uint8_t)(Len + 1);
	pBuf[Off + 1] = Type;
	memcpy(&pBuf[Off + 2], pData, Len);

	return (int)(Off + 2 + Len);
}

int BtOobLePayload(const BtOobLe_t * const pOob, uint8_t *pBuf, size_t Size)
{
	if (pOob == nullptr || pBuf == nullptr || pOob->Role > BT_OOB_LEROLE_BOTH_CENTRAL)
	{
		return 0;
	}

	int off = 0;

	// LE Bluetooth Device Address: 6 byte address then 1 byte address type
	uint8_t ad[7];

	memcpy(ad, pOob->Addr, 6);
	ad[6] = pOob->AddrType & 1;

	off = AdAppend(pBuf, Size, off, AD_TYPE_LE_ADDR, ad, 7);
	if (off < 0)
	{
		return 0;
	}

	off = AdAppend(pBuf, Size, off, AD_TYPE_LE_ROLE, &pOob->Role, 1);
	if (off < 0)
	{
		return 0;
	}

	off = AdAppend(pBuf, Size, off, AD_TYPE_LESC_CONFIRM, pOob->Confirm, 16);
	if (off < 0)
	{
		return 0;
	}

	off = AdAppend(pBuf, Size, off, AD_TYPE_LESC_RANDOM, pOob->Rand, 16);
	if (off < 0)
	{
		return 0;
	}

	if (pOob->pName)
	{
		size_t nl = strlen(pOob->pName);

		if (nl > 64)
		{
			return 0;
		}

		if (nl > 0)
		{
			off = AdAppend(pBuf, Size, off, AD_TYPE_LOCAL_NAME,
						   (const uint8_t *)pOob->pName, nl);
			if (off < 0)
			{
				return 0;
			}
		}
	}

	return off;
}

bool BtOobLeNdefAdd(RFNdefMsg_t * const pMsg, const BtOobLe_t * const pOob)
{
	uint8_t payload[BT_OOB_LE_MIN_PAYLOAD + 2 + 64];

	int l = BtOobLePayload(pOob, payload, sizeof(payload));

	if (l <= 0)
	{
		return false;
	}

	return RFNdefAddMime(pMsg, s_LeOobMime, payload, l);
}
