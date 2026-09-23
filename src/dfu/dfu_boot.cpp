/**-------------------------------------------------------------------------
@file	dfu_boot.cpp

@brief	Stage 0 boot: install a pending slot 1 image, then start slot 0.

Install is an overwrite, never a swap: the payload is copied to slot 0, the
header and TLV areas to the record. There is no revert, so an image is
verified, signature included, before anything in slot 0 is touched.

The order of the writes is what makes a power loss safe, see dfu_boot.h.

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

// Verify with each key until one takes the image. The key hash TLV, when the
// image has one, already says which, so a wrong key fails fast on it.
int DfuBootVerify(const DfuBootCfg_t &Cfg, DfuImgRead_t Read, void *pCtx,
				  const DfuImgInfo_t &Info)
{
	int res = DFU_IMG_ERR_KEY;

	for (int i = 0; i < Cfg.NbKey; i++)
	{
		res = DfuImgVerify(Cfg.pHash, Cfg.pSign, Cfg.pKey[i], Read, pCtx,
						   Info);
		if (res != DFU_IMG_ERR_KEY && res != DFU_IMG_ERR_SIG)
		{
			// Success, or a failure no other key changes.
			break;
		}
	}

	return res;
}

// Slot 1 to slot 0. Returns DFU_IMG_OK once slot 0 verifies from the record.
static int DfuBootInstall(const DfuBootCfg_t &Cfg, const DfuLayout_t &Lay)
{
	uintptr_t done = Lay.Slot1 + DfuTrailerOff(Lay) + Lay.Unit;
	DfuMap_t map = { Lay.Slot1, DfuSlot1Room(Lay) };
	DfuImgInfo_t info;

	int res = DfuImgParse(DfuMapRead, &map, map.Size, &info);
	if (res == DFU_IMG_OK)
	{
		if (info.Hdr.ImgSize > Lay.Slot0Size ||
			sizeof(DfuRecInfo_t) + info.Hdr.HdrSize + info.TlvLen >
			DFU_REC_BODY_MAX ||
			Lay.Unit + sizeof(DfuRecInfo_t) + info.Hdr.HdrSize + info.TlvLen >
			Lay.RecSize)
		{
			res = DFU_IMG_ERR_SIZE;
		}
		else
		{
			// Copied out: the header size need not keep the payload word
			// aligned in slot 1.
			uint32_t vec[2];

			memcpy(vec, (const void *)(Lay.Slot1 + info.Hdr.HdrSize),
				   sizeof(vec));
			if (DfuTgtEntryValid(vec, Lay.Slot0, info.Hdr.ImgSize) == false)
			{
				res = DFU_IMG_ERR_VECTOR;
			}
		}
		if (res == DFU_IMG_OK)
		{
			res = DfuBootVerify(Cfg, DfuMapRead, &map, info);
		}
	}

	if (res != DFU_IMG_OK)
	{
		// Refused for good: nothing about this image changes by retrying,
		// and slot 0 has not been touched. Should the Done write fail, the
		// next boot refuses the image again the same way.
		(void)DfuWriteState(done, Lay.Unit, DFU_TRAILER_DONE);

		return res;
	}

	uint32_t wu = DfuTgtWriteUnit();
	uint32_t len = (info.Hdr.ImgSize + wu - 1) / wu * wu;

	// The record goes first, so a power loss from here on leaves no
	// startable slot 0 and the next boot copies again.
	if (DfuRecClear(Lay) == false)
	{
		return DFU_IMG_ERR_READ;
	}

	// The payload is copied straight from slot 1, which is memory mapped.
	// A payload whose size is not a unit multiple takes the bytes after it
	// in slot 1 into the last unit; they are outside the hashed range.
	if (DfuEraseRange(Lay.Slot0, len) == false ||
		DfuTgtWrite(Lay.Slot0, (const void *)(Lay.Slot1 + info.Hdr.HdrSize),
					len) == false)
	{
		return DFU_IMG_ERR_READ;
	}

	// Record body: information, then the header and TLV areas from slot 1.
	uint8_t *body = DfuRecBody();
	DfuRecInfo_t ri = {
		.HdrLen = info.Hdr.HdrSize,
		.TlvLen = (uint16_t)info.TlvLen,
	};

	memcpy(body, &ri, sizeof(ri));
	memcpy(body + sizeof(ri), (const void *)Lay.Slot1, info.Hdr.HdrSize);
	memcpy(body + sizeof(ri) + info.Hdr.HdrSize,
		   (const void *)(Lay.Slot1 + info.TlvOff), info.TlvLen);

	if (DfuRecWrite(Lay, sizeof(ri) + info.Hdr.HdrSize + info.TlvLen) == false)
	{
		return DFU_IMG_ERR_READ;
	}

	// Read back through the record, the way every later boot will.
	DfuImgInfo_t chk;
	res = DfuSlot0Parse(Lay, &chk);
	if (res == DFU_IMG_OK)
	{
		res = DfuBootVerify(Cfg, DfuSlot0Read, (void *)&Lay, chk);
	}
	if (res != DFU_IMG_OK)
	{
		// Not what was verified in slot 1: no record, so nothing starts,
		// and slot 1 stays pending so the next boot copies again.
		DfuRecClear(Lay);

		return res;
	}

	// Should the Done write fail, slot 1 stays pending and the next boot
	// installs it again; slot 0 is good either way.
	(void)DfuWriteState(done, Lay.Unit, DFU_TRAILER_DONE);

	return DFU_IMG_OK;
}

int DfuBootRun(const DfuBootCfg_t &Cfg)
{
	DfuLayout_t lay;

	if (DfuLayoutGet(&lay) == false)
	{
		return DFU_IMG_ERR_SIZE;
	}
	if (Cfg.pHash == nullptr || Cfg.pSign == nullptr || Cfg.pKey == nullptr ||
		Cfg.NbKey <= 0)
	{
		return DFU_IMG_ERR_CRYPTO;
	}

	bool installed = false;

	// Done is anything but erased: a Done unit whose write was cut by a power
	// loss counts as done, since on ECC flash it cannot be written again
	// without an erase.
	if (lay.Slot1Size != 0 && DfuTrailerPending(lay) == DFU_TRAILER_PENDING &&
		DfuTrailerDone(lay) == DFU_ERASED_WORD)
	{
		installed = DfuBootInstall(Cfg, lay) == DFU_IMG_OK;
	}

	// Asked by the application: stay, whatever slot 0 holds. After an
	// install, so an update the application received is not held back.
	if (DfuRecoveryAsked())
	{
		return DFU_IMG_OK;
	}

	DfuImgInfo_t info;
	int res = DfuSlot0Parse(lay, &info);

	if (res == DFU_IMG_ERR_MAGIC)
	{
		// No record: only a debugger puts an application there that way.
		if (Cfg.bAllowNoRec &&
			DfuTgtEntryValid((const void *)lay.Slot0, lay.Slot0, lay.Slot0Size))
		{
			DfuTgtStart(lay.Slot0);
		}

		return res;
	}
	if (res != DFU_IMG_OK)
	{
		return res;
	}

	if (DfuTgtEntryValid((const void *)lay.Slot0, lay.Slot0,
						 info.Hdr.ImgSize) == false)
	{
		return DFU_IMG_ERR_VECTOR;
	}

	// Just installed means just verified, from the record, with the keys.
	if (Cfg.bVerifySlot0 && installed == false)
	{
		res = DfuBootVerify(Cfg, DfuSlot0Read, &lay, info);
		if (res != DFU_IMG_OK)
		{
			return res;
		}
	}

	DfuTgtStart(lay.Slot0);
}

/** @} */
