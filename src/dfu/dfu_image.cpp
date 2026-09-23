/**-------------------------------------------------------------------------
@file	dfu_image.cpp

@brief	MCUboot image format: header, TLV areas, hash and signature check.

See dfu_image.h for the layout. Reads go through the caller's function in
small pieces, so nothing needs the image in RAM or memory mapped.

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

#include "dfu/dfu_image.h"

/** @addtogroup DFU
  * @{
  */

// Hash chunk. Small enough for a boot stack, large enough that the read
// function overhead does not dominate.
#define DFU_IMG_CHUNK		128

// Cortex-M SRAM region, architecture defined.

static bool DfuImgReadTlv(DfuImgRead_t Read, void *pCtx, uint32_t Off,
						  DfuImgTlv_t &Tlv)
{
	return Read(pCtx, Off, &Tlv, sizeof(Tlv));
}

// One DER INTEGER into a 32 byte big endian field, left padded.
static bool DfuImgDerInt(const uint8_t *&p, const uint8_t *pEnd, uint8_t *pOut)
{
	if (pEnd - p < 2 || p[0] != 0x02)
	{
		return false;
	}
	uint32_t len = p[1];
	p += 2;
	if (len == 0 || len > (uint32_t)(pEnd - p))
	{
		return false;
	}

	// A leading zero only keeps a high bit from reading as a sign.
	while (len > 32 && *p == 0)
	{
		p++;
		len--;
	}
	if (len > 32)
	{
		return false;
	}

	memset(pOut, 0, 32 - len);
	memcpy(pOut + 32 - len, p, len);
	p += len;

	return true;
}

// DER SEQUENCE { INTEGER r, INTEGER s } into r then s. imgtool --pad-sig
// adds zeros after the sequence to make the TLV a fixed size; they are
// allowed, and nothing else is.
static bool DfuImgDerSig(const uint8_t *pDer, uint32_t Len, uint8_t *pSig)
{
	if (Len < 8 || pDer[0] != 0x30 || pDer[1] > Len - 2)
	{
		return false;
	}

	const uint8_t *p = pDer + 2;
	const uint8_t *end = p + pDer[1];

	if (DfuImgDerInt(p, end, pSig) == false ||
		DfuImgDerInt(p, end, pSig + 32) == false || p != end)
	{
		return false;
	}

	for (p = end; p < pDer + Len; p++)
	{
		if (*p != 0)
		{
			return false;
		}
	}

	return true;
}

int DfuImgParse(DfuImgRead_t Read, void *pCtx, uint32_t MaxLen,
				DfuImgInfo_t *pInfo)
{
	if (Read == nullptr || pInfo == nullptr)
	{
		return DFU_IMG_ERR_READ;
	}

	memset(pInfo, 0, sizeof(DfuImgInfo_t));

	DfuImgHdr_t &hdr = pInfo->Hdr;

	if (Read(pCtx, 0, &hdr, sizeof(hdr)) == false)
	{
		return DFU_IMG_ERR_READ;
	}
	if (hdr.Magic != DFU_IMG_MAGIC)
	{
		return DFU_IMG_ERR_MAGIC;
	}
	if (hdr.HdrSize < sizeof(DfuImgHdr_t) || hdr.HdrSize > DFU_IMG_HDR_MAX ||
		(hdr.Flags & DFU_IMG_F_UNSUPPORTED) != 0 || hdr.LoadAddr != 0 ||
		hdr.ImgSize == 0)
	{
		return DFU_IMG_ERR_HDR;
	}
	if (hdr.ImgSize > MaxLen || hdr.HdrSize > MaxLen - hdr.ImgSize)
	{
		return DFU_IMG_ERR_SIZE;
	}

	uint32_t off = hdr.HdrSize + hdr.ImgSize;
	pInfo->TlvOff = off;

	DfuImgTlvInfo_t info;

	// Protected area first, when there is one. Nothing in it is needed here,
	// only that it is well formed, because the hash covers it.
	if (hdr.ProtTlvSize != 0)
	{
		if (hdr.ProtTlvSize < sizeof(info) ||
			hdr.ProtTlvSize > MaxLen - off)
		{
			return DFU_IMG_ERR_SIZE;
		}
		if (Read(pCtx, off, &info, sizeof(info)) == false)
		{
			return DFU_IMG_ERR_READ;
		}
		if (info.Magic != DFU_IMG_TLV_PROT_MAGIC ||
			info.Len != hdr.ProtTlvSize)
		{
			return DFU_IMG_ERR_TLV;
		}

		uint32_t p = off + sizeof(info);
		uint32_t end = off + info.Len;
		while (p < end)
		{
			DfuImgTlv_t tlv;
			if (end - p < sizeof(tlv))
			{
				return DFU_IMG_ERR_TLV;
			}
			if (DfuImgReadTlv(Read, pCtx, p, tlv) == false)
			{
				return DFU_IMG_ERR_READ;
			}
			p += sizeof(tlv);
			if (tlv.Len > end - p)
			{
				return DFU_IMG_ERR_TLV;
			}
			p += tlv.Len;
		}
		off = end;
	}

	if (MaxLen - off < sizeof(info))
	{
		return DFU_IMG_ERR_SIZE;
	}
	if (Read(pCtx, off, &info, sizeof(info)) == false)
	{
		return DFU_IMG_ERR_READ;
	}
	if (info.Magic != DFU_IMG_TLV_INFO_MAGIC || info.Len < sizeof(info))
	{
		return DFU_IMG_ERR_TLV;
	}
	if (info.Len > MaxLen - off)
	{
		return DFU_IMG_ERR_SIZE;
	}

	pInfo->TlvLen = off + info.Len - pInfo->TlvOff;
	pInfo->TotalLen = off + info.Len;
	if (pInfo->TlvLen > DFU_IMG_TLV_MAX)
	{
		return DFU_IMG_ERR_TLV;
	}

	bool bsha = false;
	uint32_t p = off + sizeof(info);
	uint32_t end = off + info.Len;

	while (p < end)
	{
		DfuImgTlv_t tlv;
		if (end - p < sizeof(tlv))
		{
			return DFU_IMG_ERR_TLV;
		}
		if (DfuImgReadTlv(Read, pCtx, p, tlv) == false)
		{
			return DFU_IMG_ERR_READ;
		}
		p += sizeof(tlv);
		if (tlv.Len > end - p)
		{
			return DFU_IMG_ERR_TLV;
		}

		switch (tlv.Type)
		{
			case DFU_IMG_TLV_SHA256:
				if (bsha || tlv.Len != sizeof(pInfo->Sha))
				{
					return DFU_IMG_ERR_TLV;
				}
				if (Read(pCtx, p, pInfo->Sha, tlv.Len) == false)
				{
					return DFU_IMG_ERR_READ;
				}
				bsha = true;
				break;

			case DFU_IMG_TLV_KEYHASH:
				if (pInfo->bKeyHash || tlv.Len != sizeof(pInfo->KeyHash))
				{
					return DFU_IMG_ERR_TLV;
				}
				if (Read(pCtx, p, pInfo->KeyHash, tlv.Len) == false)
				{
					return DFU_IMG_ERR_READ;
				}
				pInfo->bKeyHash = true;
				break;

			case DFU_IMG_TLV_ECDSA_SIG:
			{
				uint8_t der[72];

				if (pInfo->bSig || tlv.Len > sizeof(der))
				{
					return DFU_IMG_ERR_TLV;
				}
				if (Read(pCtx, p, der, tlv.Len) == false)
				{
					return DFU_IMG_ERR_READ;
				}
				if (DfuImgDerSig(der, tlv.Len, pInfo->Sig) == false)
				{
					return DFU_IMG_ERR_SIG;
				}
				pInfo->bSig = true;
				break;
			}

			default:
				// Other TLVs are not ours to judge; the hash still covers
				// the protected ones.
				break;
		}
		p += tlv.Len;
	}

	return bsha ? DFU_IMG_OK : DFU_IMG_ERR_NOHASH;
}

int DfuImgHash(HashEngine *pHash, DfuImgRead_t Read, void *pCtx,
			   const DfuImgInfo_t &Info, uint8_t *pDigest)
{
	alignas(CRYPTO_HASHCTX_ALIGN_MAX) uint8_t ctx[CRYPTO_HASHCTX_MAX];
	uint8_t buf[DFU_IMG_CHUNK];

	if (pHash == nullptr || Read == nullptr || pDigest == nullptr ||
		pHash->HashCtxSize() > sizeof(ctx))
	{
		return DFU_IMG_ERR_CRYPTO;
	}
	if (pHash->HashInit(CRYPTO_HASH_SHA256, ctx) != CRYPTO_STATUS_OK)
	{
		return DFU_IMG_ERR_CRYPTO;
	}

	int res = DFU_IMG_OK;
	uint32_t len = Info.TlvOff + Info.Hdr.ProtTlvSize;

	for (uint32_t off = 0; off < len; )
	{
		uint32_t l = len - off;
		if (l > sizeof(buf))
		{
			l = sizeof(buf);
		}
		if (Read(pCtx, off, buf, l) == false)
		{
			res = DFU_IMG_ERR_READ;
			break;
		}
		if (pHash->HashUpdate(ctx, buf, l) != CRYPTO_STATUS_OK)
		{
			res = DFU_IMG_ERR_CRYPTO;
			break;
		}
		off += l;
	}

	// Final wipes the context, so it runs on the error path too.
	uint8_t digest[32];
	if (pHash->HashFinal(ctx, digest) != CRYPTO_STATUS_OK &&
		res == DFU_IMG_OK)
	{
		res = DFU_IMG_ERR_CRYPTO;
	}
	if (res == DFU_IMG_OK)
	{
		memcpy(pDigest, digest, sizeof(digest));
	}

	return res;
}

// Compare without an early exit, so the time taken does not say where the
// first difference is.
static bool DfuImgSame(const uint8_t *pA, const uint8_t *pB, size_t Len)
{
	uint8_t d = 0;

	for (size_t i = 0; i < Len; i++)
	{
		d |= pA[i] ^ pB[i];
	}

	return d == 0;
}

int DfuImgVerify(HashEngine *pHash, SignEngine *pSign, const DfuImgKey_t &Key,
				 DfuImgRead_t Read, void *pCtx, const DfuImgInfo_t &Info)
{
	uint8_t digest[32];

	if (Read == nullptr && pSign == nullptr)
	{
		// Neither the payload nor the signature: nothing would be checked.
		return DFU_IMG_ERR_SIG;
	}

	if (Read == nullptr)
	{
		// Signature over the digest the image claims, before its payload is
		// there. The caller checks the payload against Info.Sha later.
		memcpy(digest, Info.Sha, sizeof(digest));
	}
	else
	{
		int res = DfuImgHash(pHash, Read, pCtx, Info, digest);
		if (res != DFU_IMG_OK)
		{
			return res;
		}
		if (DfuImgSame(digest, Info.Sha, sizeof(digest)) == false)
		{
			return DFU_IMG_ERR_HASH;
		}
	}

	if (pSign == nullptr)
	{
		return DFU_IMG_OK;
	}

	if (Key.pDer == nullptr || Key.DerLen != DFU_IMG_P256_DER_LEN ||
		Key.pDer[DFU_IMG_P256_DER_POINT - 1] != 0x04)
	{
		return DFU_IMG_ERR_KEY;
	}
	if (Info.bSig == false)
	{
		return DFU_IMG_ERR_SIG;
	}

	if (Info.bKeyHash)
	{
		uint8_t kh[32];

		if (pHash->Hash(CRYPTO_HASH_SHA256, Key.pDer, Key.DerLen, kh) !=
			CRYPTO_STATUS_OK)
		{
			return DFU_IMG_ERR_CRYPTO;
		}
		if (DfuImgSame(kh, Info.KeyHash, sizeof(kh)) == false)
		{
			return DFU_IMG_ERR_KEY;
		}
	}

	CRYPTO_STATUS st = pSign->Verify(CRYPTO_CURVE_P256,
									 Key.pDer + DFU_IMG_P256_DER_POINT,
									 digest, sizeof(digest), Info.Sig);
	if (st == CRYPTO_STATUS_FAIL)
	{
		return DFU_IMG_ERR_SIG;
	}

	return st == CRYPTO_STATUS_OK ? DFU_IMG_OK : DFU_IMG_ERR_CRYPTO;
}

bool DfuImgVectorValid(const uint32_t *pVec, uintptr_t RunAddr,
					   uint32_t ImgSize, uintptr_t RamStart, uintptr_t RamEnd)
{
	if (pVec == nullptr)
	{
		return false;
	}

	uint32_t sp = pVec[0];
	uint32_t pc = pVec[1];

	if ((sp & 3) != 0 || sp <= RamStart || sp > RamEnd)
	{
		return false;
	}
	if ((pc & 1) == 0)
	{
		return false;
	}
	pc &= ~1UL;

	return pc >= RunAddr && pc - RunAddr < ImgSize;
}

/** @} */
