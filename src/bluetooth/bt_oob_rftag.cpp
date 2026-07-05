/*--------------------------------------------------------------------------
File   : bt_oob_rftag.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : Bluetooth LE Secure Connections OOB over an NFC tag

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

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
	if (pOob == nullptr || pBuf == nullptr)
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
