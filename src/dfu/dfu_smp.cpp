/**-------------------------------------------------------------------------
@file	dfu_smp.cpp

@brief	SMP server: OS and image management groups.

See dfu_smp.h. Field names and result codes are the ones the SMP host
libraries use, so a host needs nothing specific to IOsonata.

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
#include <errno.h>

#include "dfu/dfu_smp.h"

/** @addtogroup DFU
  * @{
  */

// Hash chunk for the whole file check.
#define DFUSMP_HASH_CHUNK		128

static uint16_t DfuSmpGet16(const uint8_t *p)
{
	return (uint16_t)((p[0] << 8) | p[1]);
}

static void DfuSmpPut16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)v;
}

// Decimal, into p, without a formatter; returns the length.
static int DfuSmpUtoa(uint32_t v, char *p)
{
	char t[10];
	int n = 0;

	do
	{
		t[n++] = (char)('0' + v % 10);
		v /= 10;
	} while (v != 0);

	for (int i = 0; i < n; i++)
	{
		p[i] = t[n - 1 - i];
	}

	return n;
}

// Version as the Zephyr image manager prints it: major.minor.revision, then
// .build when there is one.
static void DfuSmpPutVer(CborWr_t &Wr, const DfuImgVer_t &Ver)
{
	char s[4 * 11];
	int n = DfuSmpUtoa(Ver.Major, s);

	s[n++] = '.';
	n += DfuSmpUtoa(Ver.Minor, s + n);
	s[n++] = '.';
	n += DfuSmpUtoa(Ver.Rev, s + n);
	if (Ver.Build != 0)
	{
		s[n++] = '.';
		n += DfuSmpUtoa(Ver.Build, s + n);
	}

	CborPutTstr(&Wr, s, (uint32_t)n);
}

DfuSmp::DfuSmp()
{
	memset(&vCfg, 0, sizeof(vCfg));
	vbInit = false;
	vbSha = false;
}

bool DfuSmp::Init(const DfuSmpCfg_t &Cfg)
{
	vbInit = false;

	// The manager is set up already, over its layout: that is where the
	// store, the mode and the keys are.
	if (Cfg.pMgr == nullptr || Cfg.pMgr->Ready() == false ||
		Cfg.BufSize < DFUSMP_HDR_SIZE)
	{
		return false;
	}

	vCfg = Cfg;
	vbInit = true;

	return true;
}

// Result of an image check as a group result.
static int DfuSmpImgErr(int Res)
{
	switch (Res)
	{
		case DFU_IMG_ERR_READ:		return DFUSMP_IMG_ERR_READ_FAILED;
		case DFU_IMG_ERR_MAGIC:		return DFUSMP_IMG_ERR_INVALID_MAGIC;
		case DFU_IMG_ERR_HDR:		return DFUSMP_IMG_ERR_INVALID_HEADER;
		case DFU_IMG_ERR_SIZE:		return DFUSMP_IMG_ERR_TOO_LARGE;
		case DFU_IMG_ERR_TLV:		return DFUSMP_IMG_ERR_INVALID_TLV;
		case DFU_IMG_ERR_NOHASH:	return DFUSMP_IMG_ERR_NO_TLVS;
		case DFU_IMG_ERR_HASH:		return DFUSMP_IMG_ERR_INVALID_HASH;
		case DFU_IMG_ERR_VECTOR:	return DFUSMP_IMG_ERR_VECTOR_TABLE;
		case DFU_IMG_ERR_KEY:
		case DFU_IMG_ERR_SIG:		return DFUSMP_IMG_ERR_INVALID_TLV;
		default:					return DFUSMP_IMG_ERR_UNKNOWN;
	}
}

int DfuSmp::OsEcho(const uint8_t *p, uint32_t Len, CborWr_t &Wr)
{
	CborFld_t fld[] = {
		CBOR_FLD("d", CBOR_FLD_TSTR),
	};

	if (CborMapRead(p, Len, fld, 1) == false || fld[0].bFound == false)
	{
		return DFUSMP_ERR_EINVAL;
	}

	CborPutMap(&Wr, 1);
	CborPutStr(&Wr, "r");
	CborPutTstr(&Wr, (const char *)fld[0].S.p, fld[0].S.Len);

	return DFUSMP_ERR_OK;
}

int DfuSmp::OsReset(CborWr_t &Wr)
{
	if (vCfg.ResetCB == nullptr)
	{
		return DFUSMP_ERR_ENOTSUP;
	}

	CborPutMap(&Wr, 0);
	vCfg.ResetCB(vCfg.pCtx);

	return DFUSMP_ERR_OK;
}

int DfuSmp::OsParams(CborWr_t &Wr)
{
	CborPutMap(&Wr, 2);
	CborPutStr(&Wr, "buf_size");
	CborPutUint(&Wr, vCfg.BufSize);
	CborPutStr(&Wr, "buf_count");
	CborPutUint(&Wr, 1);

	return DFUSMP_ERR_OK;
}

// Hosts ask this to learn how the boot installs an image. The answer is the
// one MCUboot gives in overwrite only mode, which is what the boot here does,
// so a host picks the upload sequence that fits: no revert, no test boot.
int DfuSmp::OsBootInfo(const uint8_t *p, uint32_t Len, CborWr_t &Wr,
					   int &GrpRc)
{
	CborFld_t fld[] = {
		CBOR_FLD("query", CBOR_FLD_TSTR),
	};

	if (Len != 0 && CborMapRead(p, Len, fld, 1) == false)
	{
		return DFUSMP_ERR_EINVAL;
	}

	if (fld[0].bFound == false || fld[0].S.Len == 0)
	{
		CborPutMap(&Wr, 1);
		CborPutStr(&Wr, "bootloader");
		CborPutStr(&Wr, "MCUboot");

		return DFUSMP_ERR_OK;
	}

	if (fld[0].S.Len == 4 && memcmp(fld[0].S.p, "mode", 4) == 0)
	{
		CborPutMap(&Wr, 2);
		CborPutStr(&Wr, "mode");
		CborPutInt(&Wr, vCfg.pMgr->Direct() ? DFUSMP_BOOT_MODE_SINGLE :
									   DFUSMP_BOOT_MODE_OVERWRITE);
		CborPutStr(&Wr, "no-downgrade");
		CborPutBool(&Wr, false);

		return DFUSMP_ERR_OK;
	}

	GrpRc = DFUSMP_OS_ERR_NO_ANSWER;

	return DFUSMP_ERR_EINVAL;
}

int DfuSmp::ImgList(CborWr_t &Wr, int &GrpRc)
{
	(void)GrpRc;

	DfuImgInfo_t i0, i1;
	bool b0 = DfuSlot0Parse(vCfg.pMgr->Layout(), &i0) == DFU_IMG_OK;
	uint32_t pend = DFU_ERASED_WORD, done = DFU_ERASED_WORD;
	bool b1 = false;

	// Slot 1 is listed while it holds an image the boot has not dealt with.
	// A half uploaded one is not an image yet. Direct mode has no slot 1.
	if (vCfg.pMgr->Direct() == false && vCfg.pMgr->Active() == false &&
		vCfg.pMgr->Writer().Slot1State(&pend, &done) && done == DFU_ERASED_WORD)
	{
		b1 = vCfg.pMgr->Writer().Slot1Parse(&i1) == DFU_IMG_OK;
	}

	CborPutMap(&Wr, 2);
	CborPutStr(&Wr, "images");
	CborPutArray(&Wr, b1 ? 2 : 1);

	// Slot 0 runs, and with no revert it is confirmed by being there. Without
	// a record, a debugger put it there, or recovery has not written one
	// yet: no hash to give.
	CborPutMap(&Wr, b0 ? 6 : 5);
	CborPutStr(&Wr, "slot");
	CborPutUint(&Wr, 0);
	CborPutStr(&Wr, "version");
	if (b0)
	{
		DfuSmpPutVer(Wr, i0.Hdr.Ver);
		CborPutStr(&Wr, "hash");
		CborPutBstr(&Wr, i0.Sha, sizeof(i0.Sha));
	}
	else
	{
		CborPutStr(&Wr, "0.0.0");
	}
	CborPutStr(&Wr, "bootable");
	CborPutBool(&Wr, b0 || vCfg.pMgr->Direct() == false);
	CborPutStr(&Wr, "confirmed");
	CborPutBool(&Wr, true);
	CborPutStr(&Wr, "active");
	CborPutBool(&Wr, vCfg.pMgr->Direct() == false);

	if (b1)
	{
		bool p = pend == DFU_TRAILER_PENDING;

		CborPutMap(&Wr, 6);
		CborPutStr(&Wr, "slot");
		CborPutUint(&Wr, 1);
		CborPutStr(&Wr, "version");
		DfuSmpPutVer(Wr, i1.Hdr.Ver);
		CborPutStr(&Wr, "hash");
		CborPutBstr(&Wr, i1.Sha, sizeof(i1.Sha));
		CborPutStr(&Wr, "bootable");
		CborPutBool(&Wr, true);
		// No revert: once pending it stays, which is what permanent means.
		CborPutStr(&Wr, "pending");
		CborPutBool(&Wr, p);
		CborPutStr(&Wr, "permanent");
		CborPutBool(&Wr, p);
	}

	CborPutStr(&Wr, "splitStatus");
	CborPutUint(&Wr, 0);

	return DFUSMP_ERR_OK;
}

int DfuSmp::ImgState(const uint8_t *p, uint32_t Len, CborWr_t &Wr, int &GrpRc)
{
	CborFld_t fld[] = {
		CBOR_FLD("hash", CBOR_FLD_BSTR),
		CBOR_FLD("confirm", CBOR_FLD_BOOL),
	};

	if (CborMapRead(p, Len, fld, 2) == false ||
		(fld[0].bFound && fld[0].S.Len != 32))
	{
		return DFUSMP_ERR_EINVAL;
	}
	if (vCfg.pMgr->Active())
	{
		return DFUSMP_ERR_EBUSY;
	}

	bool confirm = fld[1].bFound && fld[1].B;
	DfuImgInfo_t info;

	if (fld[0].bFound == false)
	{
		// Confirm the running image: already so.
		if (confirm == false)
		{
			return DFUSMP_ERR_EINVAL;
		}
		return ImgList(Wr, GrpRc);
	}

	if (DfuSlot0Parse(vCfg.pMgr->Layout(), &info) == DFU_IMG_OK &&
		memcmp(info.Sha, fld[0].S.p, 32) == 0)
	{
		return ImgList(Wr, GrpRc);
	}

	if (vCfg.pMgr->Direct() || vCfg.pMgr->Writer().Slot1Parse(&info) != DFU_IMG_OK ||
		memcmp(info.Sha, fld[0].S.p, 32) != 0)
	{
		GrpRc = DFUSMP_IMG_ERR_HASH_NOT_FOUND;
		return DFUSMP_ERR_EINVAL;
	}

	// Checked in full before it is marked: the boot refuses a bad image, but
	// a host told now can say why. The signature is the boot's to check.
	uint32_t vec[2];
	int res = DfuImgVerify(vCfg.pMgr->Hash(), nullptr, DfuImgKey_t{}, DfuMgr::Read,
						   vCfg.pMgr, info);
	if (res == DFU_IMG_OK)
	{
		if (info.Hdr.ImgSize > vCfg.pMgr->Layout().Slot0Size ||
			vCfg.pMgr->Layout().Unit + sizeof(DfuRecInfo_t) + info.Hdr.HdrSize + info.TlvLen >
			vCfg.pMgr->Layout().RecSize)
		{
			res = DFU_IMG_ERR_SIZE;
		}
		else if (DfuMgr::Read(vCfg.pMgr, info.Hdr.HdrSize, vec, sizeof(vec)) ==
				 false)
		{
			res = DFU_IMG_ERR_READ;
		}
		else if (DfuTgtEntryValid(vec, vCfg.pMgr->Layout().Slot0, info.Hdr.ImgSize) == false)
		{
			res = DFU_IMG_ERR_VECTOR;
		}
	}
	if (res != DFU_IMG_OK)
	{
		GrpRc = DfuSmpImgErr(res);
		return DFUSMP_ERR_EINVAL;
	}

	if (vCfg.pMgr->Commit() == false)
	{
		GrpRc = DFUSMP_IMG_ERR_WRITE_FAILED;
		return DFUSMP_ERR_EBADSTATE;
	}

	return ImgList(Wr, GrpRc);
}

int DfuSmp::UploadDone(CborWr_t &Wr, int &GrpRc)
{
	uint32_t len = vCfg.pMgr->Length();
	DfuImgInfo_t info;

	int res = vCfg.pMgr->Finish(&info);
	if (res != DFU_IMG_OK)
	{
		GrpRc = DfuSmpImgErr(res);
		return res == DFU_IMG_ERR_READ ? DFUSMP_ERR_EUNKNOWN : DFUSMP_ERR_EINVAL;
	}

	bool match = false;
	if (vbSha)
	{
		// The host's hash is of the file as sent, all of it.
		alignas(CRYPTO_HASHCTX_ALIGN_MAX) uint8_t ctx[CRYPTO_HASHCTX_MAX];
		uint8_t buf[DFUSMP_HASH_CHUNK];
		uint8_t digest[32];
		bool ok = vCfg.pMgr->Hash()->HashCtxSize() <= sizeof(ctx) &&
				  vCfg.pMgr->Hash()->HashInit(CRYPTO_HASH_SHA256, ctx) ==
				  CRYPTO_STATUS_OK;

		for (uint32_t off = 0; ok && off < len; )
		{
			uint32_t l = len - off < sizeof(buf) ? len - off : sizeof(buf);
			ok = DfuMgr::Read(vCfg.pMgr, off, buf, l) &&
				 vCfg.pMgr->Hash()->HashUpdate(ctx, buf, l) == CRYPTO_STATUS_OK;
			off += l;
		}
		if (vCfg.pMgr->Hash()->HashFinal(ctx, digest) != CRYPTO_STATUS_OK)
		{
			ok = false;
		}
		match = ok && memcmp(digest, vSha, sizeof(digest)) == 0;
	}

	if (vCfg.pMgr->Direct())
	{
		// In place: Finish checked the signature, slot 0 becomes startable
		// now.
		if (vCfg.pMgr->Commit() == false)
		{
			GrpRc = DFUSMP_IMG_ERR_WRITE_FAILED;
			return DFUSMP_ERR_EUNKNOWN;
		}
	}

	CborPutMap(&Wr, vbSha ? 2 : 1);
	CborPutStr(&Wr, "off");
	CborPutUint(&Wr, len);
	if (vbSha)
	{
		CborPutStr(&Wr, "match");
		CborPutBool(&Wr, match);
	}

	return DFUSMP_ERR_OK;
}

int DfuSmp::ImgUpload(const uint8_t *p, uint32_t Len, CborWr_t &Wr, int &GrpRc)
{
	CborFld_t fld[] = {
		CBOR_FLD("off", CBOR_FLD_UINT),
		CBOR_FLD("data", CBOR_FLD_BSTR),
		CBOR_FLD("len", CBOR_FLD_UINT),
		CBOR_FLD("image", CBOR_FLD_UINT),
		CBOR_FLD("sha", CBOR_FLD_BSTR),
		CBOR_FLD("upgrade", CBOR_FLD_BOOL),
	};
	CborFld_t &off = fld[0], &data = fld[1], &len = fld[2], &img = fld[3],
			  &sha = fld[4];

	if (CborMapRead(p, Len, fld, 6) == false || off.bFound == false ||
		data.bFound == false)
	{
		return DFUSMP_ERR_EINVAL;
	}
	if (img.bFound && img.U != 0)
	{
		GrpRc = DFUSMP_IMG_ERR_INVALID_SLOT;
		return DFUSMP_ERR_EINVAL;
	}

	if (off.U == 0)
	{
		// A new upload. Whatever was going on before is dropped.
		vCfg.pMgr->Abort();

		if (len.bFound == false || len.U < sizeof(DfuImgHdr_t))
		{
			GrpRc = DFUSMP_IMG_ERR_INVALID_LENGTH;
			return DFUSMP_ERR_EINVAL;
		}
		if (len.U > vCfg.pMgr->MaxLen())
		{
			GrpRc = DFUSMP_IMG_ERR_TOO_LARGE;
			return DFUSMP_ERR_EINVAL;
		}
		if (data.S.Len >= 4)
		{
			uint32_t magic;
			memcpy(&magic, data.S.p, sizeof(magic));
			if (magic != DFU_IMG_MAGIC)
			{
				GrpRc = DFUSMP_IMG_ERR_INVALID_MAGIC;
				return DFUSMP_ERR_EINVAL;
			}
		}

		vbSha = sha.bFound && sha.S.Len == sizeof(vSha);
		if (vbSha)
		{
			memcpy(vSha, sha.S.p, sizeof(vSha));
		}

		// Not pending, or no startable slot 0, from here on, whatever
		// happens to the upload.
		if (vCfg.pMgr->BeginImage((uint32_t)len.U) != DFU_IMG_OK)
		{
			GrpRc = DFUSMP_IMG_ERR_ERASE_FAILED;
			return DFUSMP_ERR_EUNKNOWN;
		}
	}

	// Not where this upload is: tell the host where it is, it resumes there.
	if (vCfg.pMgr->Active() == false || off.U != vCfg.pMgr->Offset())
	{
		CborPutMap(&Wr, 1);
		CborPutStr(&Wr, "off");
		CborPutUint(&Wr, vCfg.pMgr->Active() ? vCfg.pMgr->Offset() : 0);

		return DFUSMP_ERR_OK;
	}

	if (data.S.Len > vCfg.pMgr->Length() - vCfg.pMgr->Offset())
	{
		GrpRc = DFUSMP_IMG_ERR_DATA_OVERRUN;
		return DFUSMP_ERR_EINVAL;
	}

	int res = vCfg.pMgr->Write((uint32_t)off.U, data.S.p, data.S.Len);
	if (res != DFU_IMG_OK)
	{
		vCfg.pMgr->Abort();
		GrpRc = res == DFU_IMG_ERR_READ ? DFUSMP_IMG_ERR_WRITE_FAILED :
										  DfuSmpImgErr(res);
		return res == DFU_IMG_ERR_READ ? DFUSMP_ERR_EUNKNOWN : DFUSMP_ERR_EINVAL;
	}

	if (vCfg.pMgr->Offset() == vCfg.pMgr->Length())
	{
		return UploadDone(Wr, GrpRc);
	}

	CborPutMap(&Wr, 1);
	CborPutStr(&Wr, "off");
	CborPutUint(&Wr, vCfg.pMgr->Offset());

	return DFUSMP_ERR_OK;
}

// Slot 1 is empty once its first unit and its trailer are. In direct mode the
// record goes, which is what makes slot 0 not startable.
int DfuSmp::ImgErase(CborWr_t &Wr, int &GrpRc)
{
	if (vCfg.pMgr->Active())
	{
		return DFUSMP_ERR_EBUSY;
	}

	bool ok = vCfg.pMgr->Direct() ? DfuRecClear(vCfg.pMgr->Layout()) : vCfg.pMgr->Writer().Slot1Erase();

	if (ok == false)
	{
		GrpRc = DFUSMP_IMG_ERR_ERASE_FAILED;
		return DFUSMP_ERR_EUNKNOWN;
	}

	CborPutMap(&Wr, 0);

	return DFUSMP_ERR_OK;
}

int DfuSmp::Process(const uint8_t *pReq, uint32_t ReqLen, uint8_t *pRsp,
					uint32_t RspSize)
{
	if (vbInit == false || pReq == nullptr || pRsp == nullptr ||
		ReqLen < DFUSMP_HDR_SIZE || RspSize < DFUSMP_HDR_SIZE + 32)
	{
		return 0;
	}

	uint8_t op = pReq[0] & 7;
	uint8_t ver = (pReq[0] >> 3) & 3;
	uint16_t len = DfuSmpGet16(&pReq[2]);
	uint16_t grp = DfuSmpGet16(&pReq[4]);
	uint8_t id = pReq[7];

	if ((op != DFUSMP_OP_READ && op != DFUSMP_OP_WRITE) ||
		(uint32_t)len + DFUSMP_HDR_SIZE != ReqLen)
	{
		return 0;
	}

	const uint8_t *pl = pReq + DFUSMP_HDR_SIZE;
	CborWr_t wr;
	int rc = DFUSMP_ERR_ENOTSUP;
	int grc = 0;

	CborWrInit(&wr, pRsp + DFUSMP_HDR_SIZE, RspSize - DFUSMP_HDR_SIZE);

	if (ver > DFUSMP_VER_2)
	{
		ver = DFUSMP_VER_2;
		rc = DFUSMP_ERR_TOO_NEW;
	}
	else if (grp == DFUSMP_GRP_OS)
	{
		switch (id)
		{
			case DFUSMP_OS_ECHO:
				if (op == DFUSMP_OP_WRITE)
				{
					rc = OsEcho(pl, len, wr);
				}
				break;
			case DFUSMP_OS_RESET:
				if (op == DFUSMP_OP_WRITE)
				{
					rc = OsReset(wr);
				}
				break;
			case DFUSMP_OS_PARAMS:
				if (op == DFUSMP_OP_READ)
				{
					rc = OsParams(wr);
				}
				break;
			case DFUSMP_OS_BOOTLOADER:
				if (op == DFUSMP_OP_READ)
				{
					rc = OsBootInfo(pl, len, wr, grc);
				}
				break;
		}
	}
	else if (grp == DFUSMP_GRP_IMG)
	{
		switch (id)
		{
			case DFUSMP_IMG_STATE:
				rc = op == DFUSMP_OP_READ ? ImgList(wr, grc) :
											ImgState(pl, len, wr, grc);
				break;
			case DFUSMP_IMG_UPLOAD:
				if (op == DFUSMP_OP_WRITE)
				{
					rc = ImgUpload(pl, len, wr, grc);
				}
				break;
			case DFUSMP_IMG_ERASE:
				if (op == DFUSMP_OP_WRITE)
				{
					rc = ImgErase(wr, grc);
				}
				break;
		}
	}

	if (rc == DFUSMP_ERR_OK && wr.bOvf)
	{
		rc = DFUSMP_ERR_EMSGSIZE;
	}

	if (rc != DFUSMP_ERR_OK || grc != 0)
	{
		// The partial body goes; the error is the whole response. Version 2
		// reports a group result in its own map, version 1 only knows rc.
		CborWrInit(&wr, pRsp + DFUSMP_HDR_SIZE, RspSize - DFUSMP_HDR_SIZE);
		if (ver == DFUSMP_VER_2 && grc != 0)
		{
			CborPutMap(&wr, 1);
			CborPutStr(&wr, "err");
			CborPutMap(&wr, 2);
			CborPutStr(&wr, "group");
			CborPutUint(&wr, grp);
			CborPutStr(&wr, "rc");
			CborPutUint(&wr, (uint64_t)grc);
		}
		else
		{
			CborPutMap(&wr, 1);
			CborPutStr(&wr, "rc");
			CborPutUint(&wr, (uint64_t)(rc != DFUSMP_ERR_OK ? rc :
										DFUSMP_ERR_EUNKNOWN));
		}
	}

	pRsp[0] = (uint8_t)((ver << 3) | (op + 1));
	pRsp[1] = 0;
	DfuSmpPut16(&pRsp[2], (uint16_t)wr.Len);
	DfuSmpPut16(&pRsp[4], grp);
	pRsp[6] = pReq[6];
	pRsp[7] = id;

	return DFUSMP_HDR_SIZE + (int)wr.Len;
}

/** @} */
