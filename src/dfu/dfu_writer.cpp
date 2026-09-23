/**-------------------------------------------------------------------------
@file	dfu_writer.cpp

@brief	Writing an uploaded image into slot 1, or into slot 0 and the
		record, as it arrives. See dfu_writer.h.

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

#include "dfu/dfu_writer.h"

/** @addtogroup DFU
  * @{
  */

// ---------------------------------------------------------------------------
// Store over internal memory, through the target layer.
// ---------------------------------------------------------------------------

static bool DfuTgtStRead(const DfuStore_t *pSt, uint32_t Off, void *pBuf,
						 uint32_t Len)
{
	if (Off > pSt->Size || Len > pSt->Size - Off)
	{
		return false;
	}

	memcpy(pBuf, (const void *)(pSt->Addr + Off), Len);

	return true;
}

static bool DfuTgtStWrite(const DfuStore_t *pSt, uint32_t Off,
						  const void *pData, uint32_t Len)
{
	if (Off > pSt->Size || Len > pSt->Size - Off)
	{
		return false;
	}

	return DfuTgtWrite(pSt->Addr + Off, pData, Len);
}

static uint32_t DfuTgtStUnitAt(const DfuStore_t *pSt, uint32_t Off)
{
	return DfuTgtEraseUnit(pSt->Addr + Off);
}

static bool DfuTgtStErase(const DfuStore_t *pSt, uint32_t Off, uint32_t Len)
{
	if (Off > pSt->Size || Len > pSt->Size - Off)
	{
		return false;
	}

	return DfuEraseRange(pSt->Addr + Off, Len);
}

bool DfuStoreTgt(DfuStore_t *pSt, uintptr_t Addr, uint32_t Size)
{
	if (pSt == nullptr)
	{
		return false;
	}

	uint32_t wu = DfuTgtWriteUnit();
	uint32_t eu = DfuTgtEraseUnit(Addr);

	if (wu == 0 || wu > DFU_TGT_WRITE_UNIT_MAX || eu == 0 ||
		(Addr % wu) != 0 || (Size % wu) != 0)
	{
		return false;
	}

	pSt->Size = Size;
	pSt->EraseSize = eu;
	pSt->WriteGran = wu;
	pSt->Addr = Addr;
	pSt->Read = DfuTgtStRead;
	pSt->Write = DfuTgtStWrite;
	pSt->Erase = DfuTgtStErase;
	pSt->UnitAt = DfuTgtStUnitAt;
	pSt->pCtx = nullptr;

	return true;
}

// ---------------------------------------------------------------------------
// Writer
// ---------------------------------------------------------------------------

DfuWriter::DfuWriter()
{
	memset(&vLay, 0, sizeof(vLay));
	vpStore = nullptr;
	vbDirect = false;
	vbInit = false;
	vLen = 0;
	vOff = 0;
	vErasedEnd = 0;
	vHdrSize = 0;
	vImgSize = 0;
	vTlvLen = 0;
	vStage = vStageBuf;
	vStageSize = sizeof(vStageBuf);
	vStageLen = 0;
	vStageOff = 0;
}

bool DfuWriter::Init(const DfuLayout_t &Lay, const DfuStore_t *pStore,
					 bool bDirect)
{
	vbInit = false;
	vLen = 0;

	if (pStore == nullptr || pStore->Read == nullptr ||
		pStore->Write == nullptr || pStore->Erase == nullptr ||
		pStore->WriteGran == 0 || pStore->WriteGran > DFU_TGT_WRITE_UNIT_MAX ||
		(pStore->WriteGran & (pStore->WriteGran - 1)) != 0 ||
		(Lay.Unit % pStore->WriteGran) != 0)
	{
		return false;
	}

	// The store must be the region the layout says and nothing else, or an
	// upload writes over something that is not ours.
	if (bDirect)
	{
		if (pStore->Size != Lay.Slot0Size)
		{
			return false;
		}
	}
	else if (Lay.Slot1Size == 0 || pStore->Size != Lay.Slot1Size)
	{
		return false;
	}

	if (pStore->WriteGran <= sizeof(vStageBuf) && Lay.Unit <= sizeof(vStageBuf))
	{
		vStage = vStageBuf;
		vStageSize = sizeof(vStageBuf);
	}
	else if (bDirect)
	{
		// Past the largest record body, the part DfuRecWrite pads into only
		// once the upload is over.
		vStage = (uint32_t *)(DfuRecBody() + DFU_REC_BODY_MAX);
		vStage = (uint32_t *)(((uintptr_t)vStage + 3) & ~(uintptr_t)3);
		vStageSize = DFU_TGT_WRITE_UNIT_MAX;
	}
	else
	{
		return false;
	}

	vLay = Lay;
	vpStore = pStore;
	vbDirect = bDirect;
	vbInit = true;

	return true;
}

uint32_t DfuWriter::MaxLen(void) const
{
	if (vbInit == false)
	{
		return 0;
	}
	if (vbDirect)
	{
		return DFU_IMG_HDR_MAX + vLay.Slot0Size + DFU_IMG_TLV_MAX;
	}

	return DfuSlot1Room(vLay);
}

bool DfuWriter::EraseTo(uint32_t End)
{
	uint32_t e = vpStore->EraseSize;

	if (e == 0)
	{
		return true;
	}

	while (vErasedEnd < End)
	{
		// The unit that starts here, which on a part with sectors of several
		// sizes is not e: stepping by e would erase a large sector again
		// over what was written into it.
		uint32_t u = vpStore->UnitAt != nullptr ?
					 vpStore->UnitAt(vpStore, vErasedEnd) : e;
		uint32_t l = u;

		if (u == 0)
		{
			return false;
		}
		if (vErasedEnd + l > vpStore->Size)
		{
			l = vpStore->Size - vErasedEnd;
		}
		if (vpStore->Erase(vpStore, vErasedEnd, l) == false)
		{
			return false;
		}
		vErasedEnd += u;
	}

	return true;
}

// Bytes for store offset Off, in order: whole units go straight out, the part
// of a unit left over waits in vStage.
bool DfuWriter::Put(uint32_t Off, const uint8_t *pData, uint32_t Len)
{
	uint32_t g = vpStore->WriteGran;
	uint8_t *stage = (uint8_t *)vStage;

	if (Off != vStageOff + vStageLen)
	{
		return false;
	}

	if (vStageLen != 0)
	{
		uint32_t n = g - vStageLen;
		if (n > Len)
		{
			n = Len;
		}
		memcpy(stage + vStageLen, pData, n);
		vStageLen += n;
		pData += n;
		Len -= n;

		if (vStageLen < g)
		{
			return true;
		}
		if (EraseTo(vStageOff + g) == false ||
			vpStore->Write(vpStore, vStageOff, stage, g) == false)
		{
			return false;
		}
		vStageOff += g;
		vStageLen = 0;
	}

	uint32_t bulk = Len - (Len % g);
	if (bulk != 0)
	{
		if (EraseTo(vStageOff + bulk) == false ||
			vpStore->Write(vpStore, vStageOff, pData, bulk) == false)
		{
			return false;
		}
		vStageOff += bulk;
		pData += bulk;
		Len -= bulk;
	}

	memcpy(stage, pData, Len);
	vStageLen = Len;

	return true;
}

bool DfuWriter::Flush(void)
{
	if (vStageLen == 0)
	{
		return true;
	}

	uint32_t g = vpStore->WriteGran;
	uint8_t *stage = (uint8_t *)vStage;

	memset(stage + vStageLen, 0xFF, g - vStageLen);
	vStageLen = 0;

	if (EraseTo(vStageOff + g) == false ||
		vpStore->Write(vpStore, vStageOff, stage, g) == false)
	{
		return false;
	}
	vStageOff += g;

	return true;
}

bool DfuWriter::ClearTrailer(void)
{
	uint32_t t = DfuTrailerSize(vLay);
	uint32_t off = DfuTrailerOff(vLay);

	if (vpStore->EraseSize != 0)
	{
		return vpStore->Erase(vpStore, off, t);
	}

	// No erase on this medium: ones over it, a stage buffer at a time.
	uint8_t *ones = (uint8_t *)vStage;
	memset(ones, 0xFF, vStageSize);

	for (uint32_t o = 0; o < t; o += vStageSize)
	{
		uint32_t l = t - o < vStageSize ? t - o : vStageSize;
		if (vpStore->Write(vpStore, off + o, ones, l) == false)
		{
			return false;
		}
	}

	return true;
}

int DfuWriter::Begin(uint32_t Len)
{
	vLen = 0;
	vStageLen = 0;

	if (vbInit == false)
	{
		return DFU_IMG_ERR_READ;
	}
	if (Len < sizeof(DfuImgHdr_t) || Len > MaxLen())
	{
		return DFU_IMG_ERR_SIZE;
	}

	vOff = 0;
	vErasedEnd = 0;
	vStageOff = 0;
	vHdrSize = 0;
	vImgSize = 0;
	vTlvLen = 0;

	// Before any byte lands: nothing half written may be taken for an image.
	if ((vbDirect ? DfuRecClear(vLay) : ClearTrailer()) == false)
	{
		return DFU_IMG_ERR_READ;
	}

	vLen = Len;

	return DFU_IMG_OK;
}

int DfuWriter::Write(const uint8_t *pData, uint32_t Len)
{
	if (vLen == 0 || pData == nullptr || Len > vLen - vOff)
	{
		return DFU_IMG_ERR_SIZE;
	}

	if (vbDirect == false)
	{
		if (Put(vOff, pData, Len) == false)
		{
			vLen = 0;
			return DFU_IMG_ERR_READ;
		}
		vOff += Len;

		return DFU_IMG_OK;
	}

	// Direct: header to the record body, payload to slot 0, TLVs to the
	// record body after the header.
	uint8_t *body = DfuRecBody() + sizeof(DfuRecInfo_t);

	while (Len > 0)
	{
		uint32_t n;

		if (vHdrSize == 0 || vOff < vHdrSize)
		{
			// The first 16 bytes say where the rest goes.
			uint32_t lim = vHdrSize == 0 ? 16 : vHdrSize;

			n = lim - vOff < Len ? lim - vOff : Len;
			// May be the record body itself: DfuMgr keeps a manifest there.
			memmove(body + vOff, pData, n);

			if (vHdrSize == 0 && vOff + n == 16)
			{
				DfuImgHdr_t h;

				memcpy(&h, body, 16);
				if (h.Magic != DFU_IMG_MAGIC)
				{
					vLen = 0;
					return DFU_IMG_ERR_MAGIC;
				}
				if (h.HdrSize < sizeof(DfuImgHdr_t) ||
					h.HdrSize > DFU_IMG_HDR_MAX)
				{
					vLen = 0;
					return DFU_IMG_ERR_HDR;
				}
				if (h.ImgSize > vLay.Slot0Size ||
					(uint32_t)h.HdrSize + h.ImgSize > vLen ||
					vLen - h.HdrSize - h.ImgSize > DFU_IMG_TLV_MAX)
				{
					vLen = 0;
					return DFU_IMG_ERR_SIZE;
				}
				vHdrSize = h.HdrSize;
				vImgSize = h.ImgSize;
			}
		}
		else if (vOff < vHdrSize + vImgSize)
		{
			n = vHdrSize + vImgSize - vOff;
			if (n > Len)
			{
				n = Len;
			}
			if (Put(vOff - vHdrSize, pData, n) == false)
			{
				vLen = 0;
				return DFU_IMG_ERR_READ;
			}
		}
		else
		{
			// Bounded by the length check once the header was known.
			n = Len;
			memmove(body + vOff - vImgSize, pData, n);
		}

		vOff += n;
		pData += n;
		Len -= n;
	}

	return DFU_IMG_OK;
}

bool DfuWriter::Read(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len)
{
	DfuWriter *w = (DfuWriter *)pCtx;

	if (w == nullptr || w->vbInit == false)
	{
		return false;
	}

	if (w->vbDirect == false)
	{
		return w->vpStore->Read(w->vpStore, Off, pBuf, Len);
	}

	DfuSplit_t sp = { (const DfuRecInfo_t *)DfuRecBody(), w->vLay.Slot0 };

	return DfuSplitRead(&sp, Off, pBuf, Len);
}

int DfuWriter::Finish(HashEngine *pHash, DfuImgInfo_t *pInfo)
{
	uint32_t len = vLen;

	if (len == 0 || vOff != len || pInfo == nullptr)
	{
		return DFU_IMG_ERR_SIZE;
	}

	vLen = 0;

	if (Flush() == false)
	{
		return DFU_IMG_ERR_READ;
	}

	int res;

	if (vbDirect)
	{
		if (vHdrSize == 0)
		{
			return DFU_IMG_ERR_HDR;
		}

		DfuRecInfo_t *ri = (DfuRecInfo_t *)DfuRecBody();

		ri->HdrLen = (uint16_t)vHdrSize;
		ri->TlvLen = (uint16_t)(len - vHdrSize - vImgSize);

		// Everything sent stays readable until Commit, so a host's hash of
		// the whole file can be checked; the record keeps only the TLVs.
		res = DfuImgParse(Read, this, len, pInfo);
		vTlvLen = res == DFU_IMG_OK ? pInfo->TlvLen : 0;
	}
	else
	{
		res = DfuImgParse(Read, this, DfuSlot1Room(vLay), pInfo);
	}

	if (res == DFU_IMG_OK && pInfo->TotalLen > len)
	{
		// The header describes more than was sent.
		res = DFU_IMG_ERR_SIZE;
	}
	if (res == DFU_IMG_OK &&
		(pInfo->Hdr.ImgSize > vLay.Slot0Size ||
		 sizeof(DfuRecInfo_t) + pInfo->Hdr.HdrSize + pInfo->TlvLen >
		 DFU_REC_BODY_MAX ||
		 vLay.Unit + sizeof(DfuRecInfo_t) + pInfo->Hdr.HdrSize +
		 pInfo->TlvLen > vLay.RecSize))
	{
		res = DFU_IMG_ERR_SIZE;
	}
	if (res == DFU_IMG_OK)
	{
		res = DfuImgVerify(pHash, nullptr, DfuImgKey_t{}, Read, this, *pInfo);
	}
	if (res == DFU_IMG_OK)
	{
		uint32_t vec[2];

		if (Read(this, pInfo->Hdr.HdrSize, vec, sizeof(vec)) == false)
		{
			res = DFU_IMG_ERR_READ;
		}
		else if (DfuTgtEntryValid(vec, vLay.Slot0, pInfo->Hdr.ImgSize) == false)
		{
			res = DFU_IMG_ERR_VECTOR;
		}
	}

	return res;
}

bool DfuWriter::Slot1State(uint32_t *pPending, uint32_t *pDone)
{
	if (vbInit == false || vbDirect)
	{
		return false;
	}

	uint32_t off = DfuTrailerOff(vLay);

	return vpStore->Read(vpStore, off, pPending, 4) &&
		   vpStore->Read(vpStore, off + vLay.Unit, pDone, 4);
}

bool DfuWriter::Commit(void)
{
	if (vbInit == false)
	{
		return false;
	}

	if (vbDirect)
	{
		DfuRecInfo_t *ri = (DfuRecInfo_t *)DfuRecBody();

		if (ri->HdrLen != vHdrSize || vTlvLen == 0)
		{
			return false;
		}

		// Bytes sent past the TLV areas are not the image's.
		ri->TlvLen = (uint16_t)vTlvLen;

		return DfuRecWrite(vLay, sizeof(DfuRecInfo_t) + ri->HdrLen + ri->TlvLen);
	}

	uint32_t pend, done;

	if (Slot1State(&pend, &done) == false)
	{
		return false;
	}
	if (done != DFU_ERASED_WORD)
	{
		// The boot has had this image already; it needs uploading again.
		return false;
	}
	if (pend == DFU_TRAILER_PENDING)
	{
		return true;
	}
	if (pend != DFU_ERASED_WORD)
	{
		return false;
	}

	uint32_t *u = vStage;

	memset(u, 0xFF, vLay.Unit);
	u[0] = DFU_TRAILER_PENDING;

	return vpStore->Write(vpStore, DfuTrailerOff(vLay), u, vLay.Unit);
}

int DfuWriter::Slot1Parse(DfuImgInfo_t *pInfo)
{
	if (vbInit == false || vbDirect)
	{
		return DFU_IMG_ERR_READ;
	}

	return DfuImgParse(Read, this, DfuSlot1Room(vLay), pInfo);
}

// Slot 1 is empty once its first unit and its trailer are. The rest is erased
// ahead of the next upload anyway.
bool DfuWriter::Slot1Erase(void)
{
	if (vbInit == false || vbDirect || vLen != 0)
	{
		return false;
	}

	bool ok;

	if (vpStore->EraseSize != 0)
	{
		ok = vpStore->Erase(vpStore, 0, vpStore->EraseSize);
	}
	else
	{
		uint8_t *ones = (uint8_t *)vStage;
		memset(ones, 0xFF, vLay.Unit);
		ok = vpStore->Write(vpStore, 0, ones, vLay.Unit);
	}

	return ok && ClearTrailer();
}

/** @} */
