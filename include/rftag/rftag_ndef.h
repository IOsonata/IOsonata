/**-------------------------------------------------------------------------
@file	rftag_ndef.h

@brief	NDEF message helper


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
#ifndef __RFTAG_NDEF_H__
#define __RFTAG_NDEF_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#pragma pack(push, 4)

typedef enum {
	RFNDEF_TNF_EMPTY       = 0x00,
	RFNDEF_TNF_WELL_KNOWN  = 0x01,
	RFNDEF_TNF_MIME_MEDIA  = 0x02,
	RFNDEF_TNF_ABS_URI     = 0x03,
	RFNDEF_TNF_EXTERNAL    = 0x04,
	RFNDEF_TNF_UNKNOWN     = 0x05,
} RFNDEF_TNF;

typedef struct {
	uint8_t *pBuf;
	size_t Size;
	size_t Len;
	size_t LastRecOffset;
} RFNdefMsg_t;

typedef bool (*RFNDEFCB)(void *pCtx,
                         uint8_t Tnf,
                         const uint8_t *pType,
                         size_t TypeLen,
                         const uint8_t *pPayload,
                         size_t PayloadLen);

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool RFNdefInit(RFNdefMsg_t * const pMsg, uint8_t *pBuf, size_t Size);

bool RFNdefAddRecord(RFNdefMsg_t * const pMsg,
                     uint8_t Tnf,
                     const uint8_t *pType,
                     size_t TypeLen,
                     const uint8_t *pPayload,
                     size_t PayloadLen);

bool RFNdefAddText(RFNdefMsg_t * const pMsg,
                   const char *pLang,
                   const char *pText);

bool RFNdefAddUri(RFNdefMsg_t * const pMsg,
                  const char *pUri);

bool RFNdefAddMime(RFNdefMsg_t * const pMsg,
                   const char *pMime,
                   const uint8_t *pData,
                   size_t Len);

bool RFNdefParse(const uint8_t *pData,
                 size_t Len,
                 RFNDEFCB Handler,
                 void *pCtx);

#ifdef __cplusplus
}
#endif

#endif	// __RFTAG_NDEF_H__
