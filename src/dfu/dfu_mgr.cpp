/**-------------------------------------------------------------------------
@file	dfu_mgr.cpp

@brief	DFU manager: the one API every DFU protocol uses. See dfu_mgr.h.

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

#include "crc.h"
#include "dfu/dfu_mgr.h"

/** @addtogroup DFU
  * @{
  */

// Where a manifest waits: where the writer builds the record, header first,
// TLVs after it, so in direct mode it is already in place.
static inline uint8_t *DfuMgrBody(void)
{
	return DfuRecBody() + sizeof(DfuRecInfo_t);
}

// An image seen through its manifest alone: header and TLVs, no payload.
typedef struct {
	const uint8_t *p;
	uint32_t HdrSize;
	uint32_t ImgSize;
	uint32_t TlvLen;
} DfuMgrMan_t;

static bool DfuMgrManRead(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len)
{
	const DfuMgrMan_t *m = (const DfuMgrMan_t *)pCtx;

	if (Off <= m->HdrSize && Len <= m->HdrSize - Off)
	{
		memcpy(pBuf, m->p + Off, Len);
		return true;
	}

	uint32_t t = m->HdrSize + m->ImgSize;

	if (Off >= t && Off - t <= m->TlvLen && Len <= m->TlvLen - (Off - t))
	{
		memcpy(pBuf, m->p + m->HdrSize + (Off - t), Len);
		return true;
	}

	// The payload is not there yet.
	return false;
}

// Negative when A is older than B.
static int DfuMgrVerCmp(const DfuImgVer_t &A, const DfuImgVer_t &B)
{
	if (A.Major != B.Major)
	{
		return A.Major < B.Major ? -1 : 1;
	}
	if (A.Minor != B.Minor)
	{
		return A.Minor < B.Minor ? -1 : 1;
	}
	if (A.Rev != B.Rev)
	{
		return A.Rev < B.Rev ? -1 : 1;
	}
	if (A.Build != B.Build)
	{
		return A.Build < B.Build ? -1 : 1;
	}

	return 0;
}

DfuMgr::DfuMgr()
{
	memset(&vCfg, 0, sizeof(vCfg));
	memset(&vLay, 0, sizeof(vLay));
	memset(&vCurVer, 0, sizeof(vCurVer));
	memset(vSha, 0, sizeof(vSha));
	vbInit = false;
	vState = DFU_MGR_STATE_IDLE;
	vbManifest = false;
	vOff = 0;
	vLen = 0;
	vCrc = 0;
	vHdrSize = 0;
	vTlvLen = 0;
}

bool DfuMgr::Init(const DfuMgrCfg_t &Cfg)
{
	vbInit = false;
	vState = DFU_MGR_STATE_IDLE;

	if (Cfg.pStore == nullptr || Cfg.pHash == nullptr)
	{
		return false;
	}
	// Direct mode puts an image in place with nothing after it to check the
	// signature, so it is checked here or not at all.
	if (Cfg.bDirect && (Cfg.pBoot == nullptr || Cfg.pBoot->pSign == nullptr ||
						Cfg.pBoot->pKey == nullptr || Cfg.pBoot->NbKey <= 0))
	{
		return false;
	}
	if (DfuLayoutGet(&vLay) == false ||
		vWr.Init(vLay, Cfg.pStore, Cfg.bDirect) == false)
	{
		return false;
	}

	vCfg = Cfg;

	DfuImgInfo_t info;

	memset(&vCurVer, 0, sizeof(vCurVer));
	if (DfuSlot0Parse(vLay, &info) == DFU_IMG_OK)
	{
		vCurVer = info.Hdr.Ver;
	}

	vbInit = true;

	return true;
}

int DfuMgr::BeginManifest(const uint8_t *pMan, uint32_t Len)
{
	Abort();

	if (vbInit == false)
	{
		return DFU_MGR_ERR_STATE;
	}
	if (pMan == nullptr || Len < sizeof(DfuImgHdr_t) ||
		Len > DFU_MGR_MANIFEST_MAX)
	{
		return DFU_MGR_ERR_LEN;
	}

	DfuImgHdr_t h;

	memcpy(&h, pMan, sizeof(h));
	if (h.Magic != DFU_IMG_MAGIC)
	{
		return DFU_IMG_ERR_MAGIC;
	}
	if (h.HdrSize < sizeof(DfuImgHdr_t) || h.HdrSize > DFU_IMG_HDR_MAX ||
		h.HdrSize >= Len)
	{
		return DFU_IMG_ERR_HDR;
	}

	uint32_t tlv = Len - h.HdrSize;
	uint32_t total = h.HdrSize + h.ImgSize + tlv;

	if (h.ImgSize == 0 || h.ImgSize > vLay.Slot0Size || tlv > DFU_IMG_TLV_MAX ||
		total > vWr.MaxLen() ||
		vLay.Unit + sizeof(DfuRecInfo_t) + Len > vLay.RecSize)
	{
		return DFU_IMG_ERR_SIZE;
	}

	// Parsed where it will stay: the record body. A protocol may have
	// gathered it there already (ManifestBuf).
	uint8_t *body = DfuMgrBody();
	memmove(body, pMan, Len);

	DfuMgrMan_t man = { body, h.HdrSize, h.ImgSize, tlv };
	DfuImgInfo_t info;

	int res = DfuImgParse(DfuMgrManRead, &man, total, &info);
	if (res != DFU_IMG_OK)
	{
		return res;
	}
	if (info.TotalLen != total)
	{
		// Bytes after the TLV areas are not the image's.
		return DFU_IMG_ERR_TLV;
	}

	// The signature over the digest the image claims. Nothing has been
	// erased or written yet, and nothing is unless this holds.
	if (vCfg.pBoot != nullptr && vCfg.pBoot->NbKey > 0)
	{
		res = DfuBootVerify(*vCfg.pBoot, nullptr, nullptr, info);
		if (res != DFU_IMG_OK)
		{
			return res;
		}
	}

	if (vCfg.bAllowDowngrade == false && DfuMgrVerCmp(h.Ver, vCurVer) < 0)
	{
		return DFU_MGR_ERR_VERSION;
	}

	res = vWr.Begin(total);
	if (res != DFU_IMG_OK)
	{
		return res;
	}

	// The header goes first; the TLVs wait in the body until the payload
	// is in.
	res = vWr.Write(body, h.HdrSize);
	if (res != DFU_IMG_OK)
	{
		return Fail(res);
	}

	memcpy(vSha, info.Sha, sizeof(vSha));
	vHdrSize = h.HdrSize;
	vTlvLen = tlv;
	vbManifest = true;
	vOff = 0;
	vLen = h.ImgSize;
	vCrc = 0;
	vState = DFU_MGR_STATE_RECV;

	return DFU_IMG_OK;
}

int DfuMgr::BeginImage(uint32_t Len)
{
	Abort();

	if (vbInit == false || vCfg.bManifestOnly)
	{
		return DFU_MGR_ERR_STATE;
	}

	int res = vWr.Begin(Len);
	if (res != DFU_IMG_OK)
	{
		return res;
	}

	vbManifest = false;
	vOff = 0;
	vLen = Len;
	vCrc = 0;
	vState = DFU_MGR_STATE_RECV;

	return DFU_IMG_OK;
}

int DfuMgr::Write(uint32_t Off, const uint8_t *pData, uint32_t Len)
{
	if (vState != DFU_MGR_STATE_RECV)
	{
		return DFU_MGR_ERR_STATE;
	}
	if (Off > vOff)
	{
		return DFU_MGR_ERR_OFFSET;
	}
	if (pData == nullptr || Len > vLen || Off > vLen - Len)
	{
		return DFU_IMG_ERR_SIZE;
	}

	// A resend: skip what was taken already.
	uint32_t skip = vOff - Off;
	if (skip >= Len)
	{
		return DFU_IMG_OK;
	}
	pData += skip;
	Len -= skip;

	int res = vWr.Write(pData, Len);
	if (res != DFU_IMG_OK)
	{
		return Fail(res);
	}

	vCrc = crc32_ieee_cont(vCrc, pData, (int)Len);
	vOff += Len;

	return DFU_IMG_OK;
}

int DfuMgr::Finish(DfuImgInfo_t *pInfo)
{
	DfuImgInfo_t info;

	if (vState != DFU_MGR_STATE_RECV || vOff != vLen)
	{
		return DFU_MGR_ERR_STATE;
	}

	int res;

	if (vbManifest)
	{
		// The TLVs kept since BeginManifest, now in their place.
		res = vWr.Write(DfuMgrBody() + vHdrSize, vTlvLen);
		if (res != DFU_IMG_OK)
		{
			return Fail(res);
		}
	}

	res = vWr.Finish(vCfg.pHash, &info);
	if (res != DFU_IMG_OK)
	{
		return Fail(res);
	}

	if (vbManifest)
	{
		// The value hashed against is the one whose signature was checked.
		if (memcmp(info.Sha, vSha, sizeof(vSha)) != 0)
		{
			return Fail(DFU_IMG_ERR_HASH);
		}
	}
	else
	{
		if (vCfg.bDirect)
		{
			// In place with nothing after to check it: the signature decides
			// here whether slot 0 becomes startable.
			res = DfuBootVerify(*vCfg.pBoot, DfuWriter::Read, &vWr, info);
			if (res != DFU_IMG_OK)
			{
				return Fail(res);
			}
		}
		if (vCfg.bAllowDowngrade == false &&
			DfuMgrVerCmp(info.Hdr.Ver, vCurVer) < 0)
		{
			return Fail(DFU_MGR_ERR_VERSION);
		}
	}

	if (pInfo != nullptr)
	{
		*pInfo = info;
	}
	vState = DFU_MGR_STATE_DONE;

	return DFU_IMG_OK;
}

bool DfuMgr::Commit(void)
{
	if (vbInit == false || vState == DFU_MGR_STATE_RECV)
	{
		return false;
	}
	// In place, only the image just checked. In slot 1 the boot checks
	// again, so an image left there by an earlier session may be marked.
	if (vCfg.bDirect && vState != DFU_MGR_STATE_DONE)
	{
		return false;
	}
	if (vWr.Commit() == false)
	{
		return false;
	}

	if (vCfg.bDirect)
	{
		DfuImgInfo_t info;

		if (DfuSlot0Parse(vLay, &info) == DFU_IMG_OK)
		{
			vCurVer = info.Hdr.Ver;
		}
	}
	vState = DFU_MGR_STATE_IDLE;

	return true;
}

void DfuMgr::Abort(void)
{
	vWr.Abort();
	vState = DFU_MGR_STATE_IDLE;
	vbManifest = false;
	vOff = 0;
	vLen = 0;
	vCrc = 0;
}

void DfuMgr::Reset(bool bRecovery)
{
	if (bRecovery)
	{
		DfuRecoveryRequest();
	}

	DfuTgtReset();
}

uint8_t *DfuMgr::ManifestBuf(void)
{
	return DfuMgrBody();
}

bool DfuMgr::Read(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len)
{
	DfuMgr *m = (DfuMgr *)pCtx;

	return m != nullptr && DfuWriter::Read(&m->vWr, Off, pBuf, Len);
}

/** @} */
