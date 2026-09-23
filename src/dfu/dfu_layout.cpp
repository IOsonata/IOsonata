/**-------------------------------------------------------------------------
@file	dfu_layout.cpp

@brief	DFU layout from the linker, the record, the trailer and the
		recovery request.

Used by both the stage 0 boot and the application. Everything read here is
internal memory, memory mapped on every target. What is written here goes
through the target layer, dfu_target.h.

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

#include "dfu/dfu_boot.h"

/** @addtogroup DFU
  * @{
  */

// The layout, as dfu_layout_*.ld declares it. A linker symbol's value is its
// address, which is why these are arrays. Weak, so a project without a DFU
// layout links and DfuLayoutGet says there is none instead of guessing. A
// layout with no slot 1 leaves both slot 1 symbols out, or sets them equal.
extern "C" const char __dfu_slot0_start[] __attribute__((weak));
extern "C" const char __dfu_slot0_end[] __attribute__((weak));
extern "C" const char __dfu_slot1_start[] __attribute__((weak));
extern "C" const char __dfu_slot1_end[] __attribute__((weak));
extern "C" const char __dfu_rec_start[] __attribute__((weak));
extern "C" const char __dfu_rec_end[] __attribute__((weak));

// The recovery request word: RAM that neither the boot nor the application
// links anything into, and that their startup code does not clear.
extern "C" char __dfu_flag[] __attribute__((weak));

// State unit a layout declares when it is not 16, for off target tools. Its
// value is its address, 0 when the layout leaves it out.
extern "C" const char __dfu_state_unit[] __attribute__((weak));

uint32_t DfuStateUnit(void)
{
	uint32_t u = DfuTgtWriteUnit();

	return u > DFU_STATE_UNIT_MIN ? u : DFU_STATE_UNIT_MIN;
}

bool DfuLayoutGet(DfuLayout_t *pLay)
{
	if (pLay == nullptr)
	{
		return false;
	}

	pLay->Slot0 = (uintptr_t)__dfu_slot0_start;
	pLay->Slot0Size = (uint32_t)(__dfu_slot0_end - __dfu_slot0_start);
	pLay->Slot1 = (uintptr_t)__dfu_slot1_start;
	pLay->Slot1Size = (uint32_t)(__dfu_slot1_end - __dfu_slot1_start);
	pLay->Rec = (uintptr_t)__dfu_rec_start;
	pLay->RecSize = (uint32_t)(__dfu_rec_end - __dfu_rec_start);
	pLay->Unit = DfuStateUnit();

	if ((uintptr_t)__dfu_slot1_end <= pLay->Slot1)
	{
		pLay->Slot1 = 0;
		pLay->Slot1Size = 0;
	}

	uint32_t u = pLay->Unit;
	uint32_t declared = (uint32_t)(uintptr_t)__dfu_state_unit;

	// A layout that states a unit must state this target's, or the tools
	// that build a record off target put it where the boot does not look.
	if (declared != 0 && declared != u)
	{
		return false;
	}

	// Slot 0 may start at 0 on some layout, so the ends say whether the
	// layout is there: an absent weak symbol is 0.
	return (uintptr_t)__dfu_slot0_end > pLay->Slot0 &&
		   (uintptr_t)__dfu_rec_end > pLay->Rec &&
		   pLay->RecSize >= DfuRecMax(u) &&
		   (pLay->Rec % u) == 0 && (pLay->Slot0 % u) == 0 &&
		   (pLay->Slot1Size == 0 ||
			(pLay->Slot1Size > 2 * DfuTrailerSize(*pLay) &&
			 (pLay->Slot1 % u) == 0 && (pLay->Slot1Size % u) == 0));
}

bool DfuMapRead(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len)
{
	const DfuMap_t *map = (const DfuMap_t *)pCtx;

	if (map == nullptr || pBuf == nullptr || Off > map->Size ||
		Len > map->Size - Off)
	{
		return false;
	}

	memcpy(pBuf, (const void *)(map->Addr + Off), Len);

	return true;
}

const DfuRecInfo_t *DfuRecGet(const DfuLayout_t &Lay)
{
	if (Lay.RecSize < DfuRecMax(Lay.Unit) ||
		*(const volatile uint32_t *)Lay.Rec != DFU_REC_MAGIC)
	{
		return nullptr;
	}

	const DfuRecInfo_t *rec = (const DfuRecInfo_t *)(Lay.Rec + Lay.Unit);

	if (rec->HdrLen < sizeof(DfuImgHdr_t) || rec->HdrLen > DFU_IMG_HDR_MAX ||
		rec->TlvLen > DFU_IMG_TLV_MAX)
	{
		return nullptr;
	}

	const DfuImgHdr_t *hdr = (const DfuImgHdr_t *)(rec + 1);
	if (hdr->HdrSize != rec->HdrLen || hdr->ImgSize > Lay.Slot0Size)
	{
		return nullptr;
	}

	return rec;
}

bool DfuSplitRead(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len)
{
	const DfuSplit_t *sp = (const DfuSplit_t *)pCtx;

	if (sp == nullptr || sp->pRec == nullptr || pBuf == nullptr)
	{
		return false;
	}

	const DfuRecInfo_t *rec = sp->pRec;
	const uint8_t *prec = (const uint8_t *)(rec + 1);
	uint32_t imgsize = ((const DfuImgHdr_t *)prec)->ImgSize;
	uint32_t tlvoff = rec->HdrLen + imgsize;
	uint32_t total = tlvoff + rec->TlvLen;
	uint8_t *p = (uint8_t *)pBuf;

	if (Off > total || Len > total - Off)
	{
		return false;
	}

	// The three pieces, each copied for the part of Off, Len it covers.
	while (Len > 0)
	{
		const uint8_t *src;
		uint32_t room;

		if (Off < rec->HdrLen)
		{
			src = prec + Off;
			room = rec->HdrLen - Off;
		}
		else if (Off < tlvoff)
		{
			src = (const uint8_t *)sp->Payload + (Off - rec->HdrLen);
			room = tlvoff - Off;
		}
		else
		{
			src = prec + rec->HdrLen + (Off - tlvoff);
			room = total - Off;
		}

		uint32_t l = Len < room ? Len : room;
		memcpy(p, src, l);
		p += l;
		Off += l;
		Len -= l;
	}

	return true;
}

bool DfuSlot0Read(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len)
{
	const DfuLayout_t *lay = (const DfuLayout_t *)pCtx;

	if (lay == nullptr)
	{
		return false;
	}

	DfuSplit_t sp = { DfuRecGet(*lay), lay->Slot0 };

	return DfuSplitRead(&sp, Off, pBuf, Len);
}

int DfuSlot0Parse(const DfuLayout_t &Lay, DfuImgInfo_t *pInfo)
{
	const DfuRecInfo_t *rec = DfuRecGet(Lay);

	if (rec == nullptr)
	{
		return DFU_IMG_ERR_MAGIC;
	}

	uint32_t max = rec->HdrLen + Lay.Slot0Size + rec->TlvLen;
	int res = DfuImgParse(DfuSlot0Read, (void *)&Lay, max, pInfo);
	if (res != DFU_IMG_OK)
	{
		return res;
	}

	// The record holds exactly the areas the header describes.
	if (pInfo->TlvLen != rec->TlvLen)
	{
		return DFU_IMG_ERR_TLV;
	}

	return DFU_IMG_OK;
}

static bool DfuIsErased(uintptr_t Addr, uint32_t Len)
{
	const volatile uint32_t *p = (const volatile uint32_t *)Addr;

	for (uint32_t i = 0; i < Len / 4; i++)
	{
		if (p[i] != DFU_ERASED_WORD)
		{
			return false;
		}
	}

	return true;
}

bool DfuEraseRange(uintptr_t Addr, uint32_t Len)
{
	uintptr_t end = Addr + Len;
	uint32_t first = DfuTgtEraseUnit(Addr);

	if (first == 0)
	{
		return false;
	}

	// From the start of the unit holding Addr. Units, sectors included, sit
	// at a multiple of their own size.
	Addr -= Addr % first;

	while (Addr < end)
	{
		uint32_t unit = DfuTgtEraseUnit(Addr);

		if (unit == 0 || unit % 4 != 0)
		{
			return false;
		}

		// Erased whatever it reads: on ECC flash a unit programmed with ones
		// reads as erased, yet takes no second program before an erase.
		if (DfuTgtErase(Addr) == false || DfuIsErased(Addr, unit) == false)
		{
			return false;
		}

		Addr += unit;
	}

	return true;
}

// Word aligned, with room to pad the body to whole program units.
static uint32_t s_RecBody[(DFU_REC_BODY_MAX + DFU_TGT_WRITE_UNIT_MAX + 3) / 4];

// The state unit is built after the record body it may follow, so a record
// written in the body and then its magic unit need no second buffer.
bool DfuWriteState(uintptr_t Addr, uint32_t Unit, uint32_t Value)
{
	if (Unit < DFU_STATE_UNIT_MIN || Unit > DFU_STATE_UNIT_MAX)
	{
		return false;
	}

	uint32_t *u = s_RecBody + (sizeof(s_RecBody) - DFU_STATE_UNIT_MAX) / 4;

	memset(u, 0xFF, Unit);
	u[0] = Value;

	return DfuTgtWrite(Addr, u, Unit);
}

uint8_t *DfuRecBody(void)
{
	return (uint8_t *)s_RecBody;
}

bool DfuRecClear(const DfuLayout_t &Lay)
{
	return DfuEraseRange(Lay.Rec, Lay.RecSize);
}

bool DfuRecWrite(const DfuLayout_t &Lay, uint32_t BodyLen)
{
	uint32_t wu = DfuTgtWriteUnit();

	if (BodyLen < sizeof(DfuRecInfo_t) + sizeof(DfuImgHdr_t) ||
		BodyLen > DFU_REC_BODY_MAX || Lay.Unit + BodyLen > Lay.RecSize ||
		wu == 0 || wu > DFU_TGT_WRITE_UNIT_MAX)
	{
		return false;
	}

	uint8_t *body = DfuRecBody();
	uint32_t len = (BodyLen + wu - 1) / wu * wu;

	memset(body + BodyLen, 0xFF, len - BodyLen);

	return DfuRecClear(Lay) &&
		   DfuTgtWrite(Lay.Rec + Lay.Unit, body, len) &&
		   DfuWriteState(Lay.Rec, Lay.Unit, DFU_REC_MAGIC);
}

// The address of a weak symbol the layout may leave out, through an integer
// so the compiler does not take it as never null.
static volatile uint32_t *DfuFlag(void)
{
	uintptr_t a = (uintptr_t)__dfu_flag;

	return (volatile uint32_t *)a;
}

bool DfuRecoveryAsked(void)
{
	volatile uint32_t *f = DfuFlag();

	if (f == nullptr)
	{
		return false;
	}

	bool asked = *f == DFU_FLAG_RECOVERY;

	*f = 0;

	return asked;
}

void DfuRecoveryRequest(void)
{
	volatile uint32_t *f = DfuFlag();

	if (f != nullptr)
	{
		*f = DFU_FLAG_RECOVERY;
	}

	DfuTgtReset();
}

/** @} */
