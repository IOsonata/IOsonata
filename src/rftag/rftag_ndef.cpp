/**-------------------------------------------------------------------------
@file	rftag_ndef.cpp

@brief	NDEF message helper implementation

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

#include "rftag/rftag_ndef.h"

#define RFNDEF_FLAG_MB		0x80
#define RFNDEF_FLAG_ME		0x40
#define RFNDEF_FLAG_CF		0x20
#define RFNDEF_FLAG_SR		0x10
#define RFNDEF_FLAG_IL		0x08
#define RFNDEF_FLAG_TNF_MASK	0x07

bool RFNdefInit(RFNdefMsg_t * const pMsg, uint8_t *pBuf, size_t Size)
{
	if (pMsg == nullptr || pBuf == nullptr || Size == 0)
	{
		return false;
	}

	pMsg->pBuf = pBuf;
	pMsg->Size = Size;
	pMsg->Len = 0;
	pMsg->LastRecOffset = 0;

	return true;
}

bool RFNdefAddRecord(RFNdefMsg_t * const pMsg,
                     uint8_t Tnf,
                     const uint8_t *pType,
                     size_t TypeLen,
                     const uint8_t *pPayload,
                     size_t PayloadLen)
{
	if (pMsg == nullptr || pMsg->pBuf == nullptr)
	{
		return false;
	}

	if ((TypeLen > 255) || (TypeLen > 0 && pType == nullptr) || (PayloadLen > 0 && pPayload == nullptr))
	{
		return false;
	}

	bool sr = PayloadLen <= 255;
	size_t hdrlen = 2 + (sr ? 1 : 4) + TypeLen;
	size_t reclen = hdrlen + PayloadLen;

	if ((pMsg->Len + reclen) > pMsg->Size)
	{
		return false;
	}

	if (pMsg->Len > 0)
	{
		pMsg->pBuf[pMsg->LastRecOffset] &= (uint8_t)~RFNDEF_FLAG_ME;
	}

	size_t off = pMsg->Len;
	uint8_t flags = (uint8_t)(Tnf & RFNDEF_FLAG_TNF_MASK);

	if (pMsg->Len == 0)
	{
		flags |= RFNDEF_FLAG_MB;
	}

	flags |= RFNDEF_FLAG_ME;

	if (sr)
	{
		flags |= RFNDEF_FLAG_SR;
	}

	pMsg->pBuf[off++] = flags;
	pMsg->pBuf[off++] = (uint8_t)TypeLen;

	if (sr)
	{
		pMsg->pBuf[off++] = (uint8_t)PayloadLen;
	}
	else
	{
		pMsg->pBuf[off++] = (uint8_t)(PayloadLen >> 24);
		pMsg->pBuf[off++] = (uint8_t)(PayloadLen >> 16);
		pMsg->pBuf[off++] = (uint8_t)(PayloadLen >> 8);
		pMsg->pBuf[off++] = (uint8_t)PayloadLen;
	}

	if (TypeLen > 0)
	{
		memcpy(&pMsg->pBuf[off], pType, TypeLen);
		off += TypeLen;
	}

	if (PayloadLen > 0)
	{
		memcpy(&pMsg->pBuf[off], pPayload, PayloadLen);
		off += PayloadLen;
	}

	pMsg->LastRecOffset = pMsg->Len;
	pMsg->Len = off;

	return true;
}

bool RFNdefAddText(RFNdefMsg_t * const pMsg,
                   const char *pLang,
                   const char *pText)
{
	if (pLang == nullptr || pText == nullptr)
	{
		return false;
	}

	size_t langlen = strlen(pLang);
	size_t textlen = strlen(pText);

	if (langlen > 63)
	{
		return false;
	}

	uint8_t payload[256];

	if ((1 + langlen + textlen) > sizeof(payload))
	{
		return false;
	}

	payload[0] = (uint8_t)langlen;
	memcpy(&payload[1], pLang, langlen);
	memcpy(&payload[1 + langlen], pText, textlen);

	const uint8_t type = 'T';

	return RFNdefAddRecord(pMsg, RFNDEF_TNF_WELL_KNOWN, &type, 1, payload, 1 + langlen + textlen);
}

bool RFNdefAddUri(RFNdefMsg_t * const pMsg,
                  const char *pUri)
{
	if (pUri == nullptr)
	{
		return false;
	}

	size_t urilen = strlen(pUri);
	uint8_t payload[256];

	if ((1 + urilen) > sizeof(payload))
	{
		return false;
	}

	payload[0] = 0x00;
	memcpy(&payload[1], pUri, urilen);

	const uint8_t type = 'U';

	return RFNdefAddRecord(pMsg, RFNDEF_TNF_WELL_KNOWN, &type, 1, payload, 1 + urilen);
}

bool RFNdefAddMime(RFNdefMsg_t * const pMsg,
                   const char *pMime,
                   const uint8_t *pData,
                   size_t Len)
{
	if (pMime == nullptr)
	{
		return false;
	}

	return RFNdefAddRecord(pMsg, RFNDEF_TNF_MIME_MEDIA, (const uint8_t*)pMime, strlen(pMime), pData, Len);
}

bool RFNdefParse(const uint8_t *pData,
                 size_t Len,
                 RFNDEFCB Handler,
                 void *pCtx)
{
	size_t off = 0;

	if (pData == nullptr || Handler == nullptr)
	{
		return false;
	}

	while (off < Len)
	{
		if ((Len - off) < 3)
		{
			return false;
		}

		uint8_t flags = pData[off++];
		uint8_t typeLen = pData[off++];
		uint32_t payloadLen = 0;

		if (flags & RFNDEF_FLAG_SR)
		{
			payloadLen = pData[off++];
		}
		else
		{
			if ((Len - off) < 4)
			{
				return false;
			}
			payloadLen = ((uint32_t)pData[off] << 24) |
			             ((uint32_t)pData[off + 1] << 16) |
			             ((uint32_t)pData[off + 2] << 8) |
			             pData[off + 3];
			off += 4;
		}

		uint8_t idLen = 0;

		if (flags & RFNDEF_FLAG_IL)
		{
			if (off >= Len)
			{
				return false;
			}
			idLen = pData[off++];
		}

		// Check each field against the remaining bytes one at a time.
		// A 4 byte payloadLen could wrap an added sum, so subtract instead.
		size_t remain = Len - off;

		if (typeLen > remain)
		{
			return false;
		}
		remain -= typeLen;

		if (idLen > remain)
		{
			return false;
		}
		remain -= idLen;

		if (payloadLen > remain)
		{
			return false;
		}

		const uint8_t *pType = &pData[off];
		off += typeLen;

		off += idLen;

		const uint8_t *pPayload = &pData[off];
		off += payloadLen;

		if (Handler(pCtx, flags & RFNDEF_FLAG_TNF_MASK, pType, typeLen, pPayload, payloadLen) == false)
		{
			return false;
		}

		if (flags & RFNDEF_FLAG_ME)
		{
			break;
		}
	}

	return true;
}
