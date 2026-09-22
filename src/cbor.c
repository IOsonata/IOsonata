/**-------------------------------------------------------------------------
@file	cbor.c

@brief	Small CBOR (RFC 8949) reader and writer.

See cbor.h for the subset. Nothing is allocated; the reader walks the input
in place and the writer fills the caller's buffer.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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

#include "cbor.h"

/** @addtogroup Utilities
  * @{
  */

#define CBOR_MAJ_UINT		0
#define CBOR_MAJ_NINT		1
#define CBOR_MAJ_BSTR		2
#define CBOR_MAJ_TSTR		3
#define CBOR_MAJ_ARRAY		4
#define CBOR_MAJ_MAP		5
#define CBOR_MAJ_TAG		6
#define CBOR_MAJ_SIMPLE		7

#define CBOR_AI_INDEF		31
#define CBOR_BREAK			0xFF
#define CBOR_FALSE			20
#define CBOR_TRUE			21
#define CBOR_NULL			22
#define CBOR_UNDEF			23

typedef struct {
	const uint8_t *p;
	const uint8_t *pEnd;
} CborRd_t;

// One item head. Arg is the count, length or value; bIndef an indefinite
// length container.
static bool CborRdHeadAi(CborRd_t *pRd, uint8_t *pMaj, uint64_t *pArg,
						 bool *pIndef, uint8_t *pAi)
{
	if (pRd->p >= pRd->pEnd)
	{
		return false;
	}

	uint8_t ib = *pRd->p++;
	uint8_t ai = ib & 0x1F;

	*pAi = ai;
	*pMaj = ib >> 5;
	*pIndef = false;
	*pArg = 0;

	if (ai < 24)
	{
		*pArg = ai;
		return true;
	}
	if (ai == CBOR_AI_INDEF)
	{
		// Only containers here, strings in pieces are not taken.
		*pIndef = true;
		return *pMaj == CBOR_MAJ_ARRAY || *pMaj == CBOR_MAJ_MAP;
	}
	if (ai > 27)
	{
		return false;
	}

	int n = 1 << (ai - 24);
	if (pRd->pEnd - pRd->p < n)
	{
		return false;
	}
	for (int i = 0; i < n; i++)
	{
		*pArg = (*pArg << 8) | *pRd->p++;
	}

	return true;
}

static bool CborRdHead(CborRd_t *pRd, uint8_t *pMaj, uint64_t *pArg,
					   bool *pIndef)
{
	uint8_t ai;

	return CborRdHeadAi(pRd, pMaj, pArg, pIndef, &ai);
}

static bool CborRdBreak(CborRd_t *pRd)
{
	if (pRd->p < pRd->pEnd && *pRd->p == CBOR_BREAK)
	{
		pRd->p++;
		return true;
	}

	return false;
}

static bool CborRdSkip(CborRd_t *pRd, int Depth);

// The items of an array (Pairs 1) or map (Pairs 2), definite or not.
static bool CborRdSkipItems(CborRd_t *pRd, uint64_t Cnt, bool bIndef,
							int Pairs, int Depth)
{
	if (bIndef)
	{
		while (CborRdBreak(pRd) == false)
		{
			for (int i = 0; i < Pairs; i++)
			{
				if (CborRdSkip(pRd, Depth) == false)
				{
					return false;
				}
			}
		}
		return true;
	}

	// Each item takes at least a byte, so a count beyond the input is a
	// bad one, found here before any looping.
	if (Cnt > (uint64_t)(pRd->pEnd - pRd->p))
	{
		return false;
	}
	for (uint64_t n = 0; n < Cnt * (uint64_t)Pairs; n++)
	{
		if (CborRdSkip(pRd, Depth) == false)
		{
			return false;
		}
	}

	return true;
}

static bool CborRdSkip(CborRd_t *pRd, int Depth)
{
	uint8_t maj;
	uint64_t arg;
	bool indef;
	uint8_t ai;

	if (Depth >= CBOR_DEPTH_MAX ||
		CborRdHeadAi(pRd, &maj, &arg, &indef, &ai) == false)
	{
		return false;
	}

	switch (maj)
	{
		case CBOR_MAJ_UINT:
		case CBOR_MAJ_NINT:
			return true;

		case CBOR_MAJ_BSTR:
		case CBOR_MAJ_TSTR:
			if (arg > (uint64_t)(pRd->pEnd - pRd->p))
			{
				return false;
			}
			pRd->p += arg;
			return true;

		case CBOR_MAJ_ARRAY:
			return CborRdSkipItems(pRd, arg, indef, 1, Depth + 1);

		case CBOR_MAJ_MAP:
			return CborRdSkipItems(pRd, arg, indef, 2, Depth + 1);

		case CBOR_MAJ_SIMPLE:
			// false, true, null, undefined, and the one byte simple values.
			// Floats, the two to eight byte forms, are not taken.
			return (ai < 24 && arg >= CBOR_FALSE && arg <= CBOR_UNDEF) ||
				   ai == 24;

		default:
			return false;
	}
}

// The value of a wanted key, into its field.
static bool CborRdFld(CborRd_t *pRd, CborFld_t *pFld)
{
	uint8_t maj;
	uint64_t arg;
	bool indef;

	if (CborRdHead(pRd, &maj, &arg, &indef) == false || indef)
	{
		return false;
	}

	switch (pFld->Type)
	{
		case CBOR_FLD_UINT:
			if (maj != CBOR_MAJ_UINT)
			{
				return false;
			}
			pFld->U = arg;
			break;

		case CBOR_FLD_INT:
			if ((maj != CBOR_MAJ_UINT && maj != CBOR_MAJ_NINT) ||
				arg > (uint64_t)INT64_MAX)
			{
				return false;
			}
			pFld->I = maj == CBOR_MAJ_UINT ? (int64_t)arg : -1 - (int64_t)arg;
			break;

		case CBOR_FLD_BOOL:
			if (maj != CBOR_MAJ_SIMPLE ||
				(arg != CBOR_FALSE && arg != CBOR_TRUE))
			{
				return false;
			}
			pFld->B = arg == CBOR_TRUE;
			break;

		case CBOR_FLD_BSTR:
		case CBOR_FLD_TSTR:
			if (maj != (pFld->Type == CBOR_FLD_BSTR ? CBOR_MAJ_BSTR :
													  CBOR_MAJ_TSTR) ||
				arg > (uint64_t)(pRd->pEnd - pRd->p))
			{
				return false;
			}
			pFld->S.p = pRd->p;
			pFld->S.Len = (uint32_t)arg;
			pRd->p += arg;
			break;

		default:
			return false;
	}

	pFld->bFound = true;

	return true;
}

// One key and value pair of the top map.
static bool CborRdPair(CborRd_t *pRd, CborFld_t *pFld, int NbFld)
{
	const uint8_t *start = pRd->p;
	uint8_t maj;
	uint64_t arg;
	bool indef;

	if (CborRdHead(pRd, &maj, &arg, &indef) == false)
	{
		return false;
	}
	if (maj != CBOR_MAJ_TSTR)
	{
		// Not a key anyone here asks for; skip it, it may still be valid.
		pRd->p = start;
		return CborRdSkip(pRd, 1) && CborRdSkip(pRd, 1);
	}
	if (arg > (uint64_t)(pRd->pEnd - pRd->p))
	{
		return false;
	}

	const uint8_t *key = pRd->p;
	pRd->p += arg;

	for (int i = 0; i < NbFld; i++)
	{
		if (strlen(pFld[i].pKey) == arg && memcmp(pFld[i].pKey, key, arg) == 0)
		{
			if (pFld[i].bFound)
			{
				return false;
			}
			return CborRdFld(pRd, &pFld[i]);
		}
	}

	return CborRdSkip(pRd, 1);
}

bool CborMapRead(const uint8_t *pData, uint32_t Len, CborFld_t *pFld,
				 int NbFld)
{
	CborRd_t rd = { pData, pData + Len };
	uint8_t maj;
	uint64_t arg;
	bool indef;

	for (int i = 0; i < NbFld; i++)
	{
		pFld[i].bFound = false;
	}

	if (pData == NULL || CborRdHead(&rd, &maj, &arg, &indef) == false ||
		maj != CBOR_MAJ_MAP)
	{
		return false;
	}

	if (indef)
	{
		while (CborRdBreak(&rd) == false)
		{
			if (CborRdPair(&rd, pFld, NbFld) == false)
			{
				return false;
			}
		}
	}
	else
	{
		if (arg > Len)
		{
			return false;
		}
		for (uint64_t n = 0; n < arg; n++)
		{
			if (CborRdPair(&rd, pFld, NbFld) == false)
			{
				return false;
			}
		}
	}

	return rd.p == rd.pEnd;
}

void CborWrInit(CborWr_t *pWr, uint8_t *pBuf, uint32_t Size)
{
	pWr->pBuf = pBuf;
	pWr->Size = pBuf != NULL ? Size : 0;
	pWr->Len = 0;
	pWr->bOvf = false;
}

static void CborWrBytes(CborWr_t *pWr, const void *pData, uint32_t Len)
{
	if (pWr->bOvf || Len > pWr->Size - pWr->Len)
	{
		pWr->bOvf = true;
		return;
	}
	if (Len == 0)
	{
		return;
	}

	memcpy(pWr->pBuf + pWr->Len, pData, Len);
	pWr->Len += Len;
}

// Smallest head that holds Arg.
static void CborWrHead(CborWr_t *pWr, uint8_t Maj, uint64_t Arg)
{
	uint8_t h[9];
	int n;

	if (Arg < 24)
	{
		h[0] = (Maj << 5) | (uint8_t)Arg;
		n = 0;
	}
	else if (Arg <= 0xFF)
	{
		h[0] = (Maj << 5) | 24;
		n = 1;
	}
	else if (Arg <= 0xFFFF)
	{
		h[0] = (Maj << 5) | 25;
		n = 2;
	}
	else if (Arg <= 0xFFFFFFFFULL)
	{
		h[0] = (Maj << 5) | 26;
		n = 4;
	}
	else
	{
		h[0] = (Maj << 5) | 27;
		n = 8;
	}

	for (int i = 0; i < n; i++)
	{
		h[n - i] = (uint8_t)(Arg >> (8 * i));
	}

	CborWrBytes(pWr, h, (uint32_t)n + 1);
}

void CborPutMap(CborWr_t *pWr, uint32_t NbPair)
{
	CborWrHead(pWr, CBOR_MAJ_MAP, NbPair);
}

void CborPutArray(CborWr_t *pWr, uint32_t NbItem)
{
	CborWrHead(pWr, CBOR_MAJ_ARRAY, NbItem);
}

void CborPutUint(CborWr_t *pWr, uint64_t Val)
{
	CborWrHead(pWr, CBOR_MAJ_UINT, Val);
}

void CborPutInt(CborWr_t *pWr, int64_t Val)
{
	if (Val < 0)
	{
		CborWrHead(pWr, CBOR_MAJ_NINT, (uint64_t)(-1 - Val));
	}
	else
	{
		CborWrHead(pWr, CBOR_MAJ_UINT, (uint64_t)Val);
	}
}

void CborPutBool(CborWr_t *pWr, bool Val)
{
	uint8_t b = (CBOR_MAJ_SIMPLE << 5) | (Val ? CBOR_TRUE : CBOR_FALSE);

	CborWrBytes(pWr, &b, 1);
}

void CborPutBstr(CborWr_t *pWr, const uint8_t *pData, uint32_t Len)
{
	CborWrHead(pWr, CBOR_MAJ_BSTR, Len);
	CborWrBytes(pWr, pData, Len);
}

void CborPutTstr(CborWr_t *pWr, const char *pStr, uint32_t Len)
{
	CborWrHead(pWr, CBOR_MAJ_TSTR, Len);
	CborWrBytes(pWr, pStr, Len);
}

void CborPutStr(CborWr_t *pWr, const char *pStr)
{
	CborPutTstr(pWr, pStr, (uint32_t)strlen(pStr));
}

/** @} */
