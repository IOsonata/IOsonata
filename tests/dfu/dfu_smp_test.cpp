// SMP server on simulated memory: every command, the error paths, uploads in
// odd sized pieces with interruptions, then an install by the stage 0 boot and
// the image list after it. On NOR flash and RRAM.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "dfu/dfu_smp.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

static int s_Fail = 0;
static int s_Pass = 0;

#define CHECK(c) do { if (c) { s_Pass++; } else { s_Fail++; \
	fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)

static std::string s_Dir;

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];
static HashEngine *s_Sha;
static std::vector<uint8_t> s_Key1;
static DfuImgKey_t s_Key;
static DfuBootCfg_t s_BootCfg;

static int s_ResetCnt = 0;
static void ResetCB(void *)
{
	s_ResetCnt++;
}

static std::vector<uint8_t> Img(const char *pName)
{
	return HostReadFile(s_Dir + "/" + pName);
}

struct Rsp {
	int Op;
	int Ver;
	int Grp;
	int Id;
	int Seq;
	std::vector<uint8_t> Body;
};

static uint8_t s_Seq = 0;

// One request; the body is built by Fill, or empty map when null.
static Rsp Req(DfuSmp &Mgr, int Op, int Grp, int Id,
			   std::function<void(CborWr_t &)> Fill = nullptr, int Ver = 1)
{
	uint8_t req[2048], rsp[512];
	CborWr_t w;

	CborWrInit(&w, req + 8, sizeof(req) - 8);
	if (Fill)
	{
		Fill(w);
	}
	else
	{
		CborPutMap(&w, 0);
	}
	req[0] = (uint8_t)((Ver << 3) | Op);
	req[1] = 0;
	req[2] = (uint8_t)(w.Len >> 8);
	req[3] = (uint8_t)w.Len;
	req[4] = (uint8_t)(Grp >> 8);
	req[5] = (uint8_t)Grp;
	req[6] = ++s_Seq;
	req[7] = (uint8_t)Id;

	int n = Mgr.Process(req, 8 + w.Len, rsp, sizeof(rsp));

	Rsp r = { -1, 0, 0, 0, 0, {} };
	if (n >= 8)
	{
		r.Op = rsp[0] & 7;
		r.Ver = (rsp[0] >> 3) & 3;
		r.Grp = (rsp[4] << 8) | rsp[5];
		r.Seq = rsp[6];
		r.Id = rsp[7];
		CHECK(((rsp[2] << 8) | rsp[3]) == n - 8);
		CHECK(r.Seq == s_Seq && r.Grp == Grp && r.Id == Id && r.Op == Op + 1);
		r.Body.assign(rsp + 8, rsp + n);
	}
	return r;
}

static int64_t Rc(const Rsp &R)
{
	CborFld_t f[] = { CBOR_FLD("rc", CBOR_FLD_INT) };
	if (CborMapRead(R.Body.data(), (uint32_t)R.Body.size(), f, 1) && f[0].bFound)
	{
		return f[0].I;
	}
	return 0;
}

// Version 2 group result, 0 when none.
static int64_t GrpRc(const Rsp &R)
{
	CborFld_t f[] = { CBOR_FLD("err", CBOR_FLD_UINT) };
	const uint8_t *p = R.Body.data();
	// {"err": {"group": g, "rc": rc}} is small; find the inner map by hand.
	if (R.Body.size() > 6 && p[0] == 0xA1 && memcmp(p + 1, "\x63" "err", 4) == 0)
	{
		CborFld_t g[] = {
			CBOR_FLD("group", CBOR_FLD_UINT),
			CBOR_FLD("rc", CBOR_FLD_UINT),
		};
		if (CborMapRead(p + 5, (uint32_t)R.Body.size() - 5, g, 2) &&
			g[0].bFound && g[1].bFound)
		{
			return (int64_t)g[1].U;
		}
	}
	(void)f;
	return 0;
}

static uint64_t UploadOff(const Rsp &R, bool *pMatch = nullptr)
{
	CborFld_t f[] = {
		CBOR_FLD("off", CBOR_FLD_UINT),
		CBOR_FLD("match", CBOR_FLD_BOOL),
	};
	CHECK(CborMapRead(R.Body.data(), (uint32_t)R.Body.size(), f, 2));
	if (pMatch != nullptr)
	{
		*pMatch = f[1].bFound && f[1].B;
	}
	return f[0].bFound ? f[0].U : ~0ULL;
}

static Rsp Chunk(DfuSmp &Mgr, const std::vector<uint8_t> &File, uint32_t Off,
				 uint32_t Len, bool bSha, int Ver = 1)
{
	return Req(Mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_UPLOAD,
		[&](CborWr_t &w) {
			CborPutMap(&w, Off == 0 ? (bSha ? 5 : 4) : 2);
			if (Off == 0)
			{
				CborPutStr(&w, "image");
				CborPutUint(&w, 0);
				CborPutStr(&w, "len");
				CborPutUint(&w, File.size());
				if (bSha)
				{
					uint8_t d[32];
					s_Sha->Hash(CRYPTO_HASH_SHA256, File.data(), File.size(), d);
					CborPutStr(&w, "sha");
					CborPutBstr(&w, d, 32);
				}
			}
			CborPutStr(&w, "off");
			CborPutUint(&w, Off);
			CborPutStr(&w, "data");
			CborPutBstr(&w, File.data() + Off, Len);
		}, Ver);
}

// Upload in pieces of random size, 1 to MaxChunk bytes.
static bool Upload(DfuSmp &Mgr, const std::vector<uint8_t> &File,
				   std::mt19937 &Rng, uint32_t MaxChunk, bool *pMatch)
{
	uint32_t off = 0;
	while (off < File.size())
	{
		uint32_t l = 1 + Rng() % MaxChunk;
		if (l > File.size() - off)
		{
			l = (uint32_t)(File.size() - off);
		}
		Rsp r = Chunk(Mgr, File, off, l, true);
		if (Rc(r) != 0 || GrpRc(r) != 0)
		{
			return false;
		}
		uint64_t n = UploadOff(r, pMatch);
		if (n != off + l)
		{
			return false;
		}
		off = (uint32_t)n;
	}
	return true;
}

struct ImgEntry {
	bool bFound;
	uint64_t Slot;
	std::string Ver;
	std::vector<uint8_t> Hash;
	bool bPending, bActive, bConfirmed;
};

// Image list, parsed from its known shape: {"images": [..], "splitStatus": 0}
static std::vector<ImgEntry> List(DfuSmp &Mgr)
{
	Rsp r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_IMG, DFUSMP_IMG_STATE);
	std::vector<ImgEntry> out;
	const uint8_t *p = r.Body.data();

	CHECK(Rc(r) == 0);
	if (r.Body.size() < 10 || p[0] != 0xA2 || memcmp(p + 1, "\x66images", 7) != 0)
	{
		CHECK(false);
		return out;
	}
	int n = p[8] & 0x1F;
	CHECK((p[8] >> 5) == 4 && n >= 1 && n <= 2);
	const uint8_t *q = p + 9;
	const uint8_t *end = p + r.Body.size();

	for (int i = 0; i < n; i++)
	{
		// Find the extent of this entry map: count pairs, walk them.
		int pairs = q[0] & 0x1F;
		const uint8_t *s = q + 1;
		for (int k = 0; k < pairs * 2; k++)
		{
			uint8_t ib = *s++;
			uint8_t maj = ib >> 5, ai = ib & 0x1F;
			uint64_t a = ai;
			if (ai == 24) { a = *s++; }
			else if (ai == 25) { a = (s[0] << 8) | s[1]; s += 2; }
			if (maj == 2 || maj == 3) { s += a; }
		}
		CborFld_t f[] = {
			CBOR_FLD("slot", CBOR_FLD_UINT),
			CBOR_FLD("version", CBOR_FLD_TSTR),
			CBOR_FLD("hash", CBOR_FLD_BSTR),
			CBOR_FLD("pending", CBOR_FLD_BOOL),
			CBOR_FLD("active", CBOR_FLD_BOOL),
			CBOR_FLD("confirmed", CBOR_FLD_BOOL),
			CBOR_FLD("bootable", CBOR_FLD_BOOL),
			CBOR_FLD("permanent", CBOR_FLD_BOOL),
		};
		CHECK(s <= end && CborMapRead(q, (uint32_t)(s - q), f, 8));
		ImgEntry e;
		e.bFound = true;
		e.Slot = f[0].U;
		e.Ver.assign((const char *)f[1].S.p, f[1].S.Len);
		if (f[2].bFound)
		{
			e.Hash.assign(f[2].S.p, f[2].S.p + f[2].S.Len);
		}
		e.bPending = f[3].bFound && f[3].B;
		e.bActive = f[4].bFound && f[4].B;
		e.bConfirmed = f[5].bFound && f[5].B;
		out.push_back(e);
		q = s;
	}
	return out;
}

static std::vector<uint8_t> ShaTlv(const std::vector<uint8_t> &File)
{
	DfuMap_t m = { (uintptr_t)File.data(), (uint32_t)File.size() };
	DfuImgInfo_t info;
	CHECK(DfuImgParse(DfuMapRead, &m, m.Size, &info) == DFU_IMG_OK);
	return std::vector<uint8_t>(info.Sha, info.Sha + 32);
}

static Rsp SetState(DfuSmp &Mgr, const std::vector<uint8_t> *pHash, bool bConfirm,
					int Ver = 1)
{
	return Req(Mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_STATE,
		[&](CborWr_t &w) {
			CborPutMap(&w, pHash ? 2 : 1);
			if (pHash)
			{
				CborPutStr(&w, "hash");
				CborPutBstr(&w, pHash->data(), (uint32_t)pHash->size());
			}
			CborPutStr(&w, "confirm");
			CborPutBool(&w, bConfirm);
		}, Ver);
}

// The boot, as after a reset.
static bool Reboot(void)
{
	if (setjmp(g_HostStartJmp) != 0)
	{
		return g_HostStartAddr == HOST_SLOT0;
	}
	(void)DfuBootRun(s_BootCfg);
	return false;
}

static void TestOs(DfuSmp &Mgr)
{
	Rsp r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_PARAMS);
	CborFld_t f[] = {
		CBOR_FLD("buf_size", CBOR_FLD_UINT),
		CBOR_FLD("buf_count", CBOR_FLD_UINT),
	};
	CHECK(CborMapRead(r.Body.data(), (uint32_t)r.Body.size(), f, 2));
	CHECK(f[0].U == 1024 && f[1].U == 1);

	r = Req(Mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_OS, DFUSMP_OS_ECHO,
		[](CborWr_t &w) { CborPutMap(&w, 1); CborPutStr(&w, "d");
						  CborPutStr(&w, "hello"); });
	CborFld_t e[] = { CBOR_FLD("r", CBOR_FLD_TSTR) };
	CHECK(CborMapRead(r.Body.data(), (uint32_t)r.Body.size(), e, 1));
	CHECK(e[0].S.Len == 5 && memcmp(e[0].S.p, "hello", 5) == 0);

	r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_BOOTLOADER);
	CborFld_t b[] = { CBOR_FLD("bootloader", CBOR_FLD_TSTR) };
	CHECK(CborMapRead(r.Body.data(), (uint32_t)r.Body.size(), b, 1));
	CHECK(b[0].S.Len == 7 && memcmp(b[0].S.p, "MCUboot", 7) == 0);

	r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_BOOTLOADER,
		[](CborWr_t &w) { CborPutMap(&w, 1); CborPutStr(&w, "query");
						  CborPutStr(&w, "mode"); });
	CborFld_t m[] = {
		CBOR_FLD("mode", CBOR_FLD_INT),
		CBOR_FLD("no-downgrade", CBOR_FLD_BOOL),
	};
	CHECK(CborMapRead(r.Body.data(), (uint32_t)r.Body.size(), m, 2));
	CHECK(m[0].I == 2 && m[1].bFound && m[1].B == false);

	r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_BOOTLOADER,
		[](CborWr_t &w) { CborPutMap(&w, 1); CborPutStr(&w, "query");
						  CborPutStr(&w, "what"); });
	CHECK(GrpRc(r) == DFUSMP_OS_ERR_NO_ANSWER);
	r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_BOOTLOADER,
		[](CborWr_t &w) { CborPutMap(&w, 1); CborPutStr(&w, "query");
						  CborPutStr(&w, "what"); }, 0);
	CHECK(Rc(r) == DFUSMP_ERR_EINVAL);

	s_ResetCnt = 0;
	r = Req(Mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_OS, DFUSMP_OS_RESET);
	CHECK(Rc(r) == 0 && s_ResetCnt == 1);

	// Not supported: another group, another command, a wrong operation.
	r = Req(Mgr, DFUSMP_OP_READ, 2, 0);
	CHECK(Rc(r) == DFUSMP_ERR_ENOTSUP);
	r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, 3);
	CHECK(Rc(r) == DFUSMP_ERR_ENOTSUP);
	r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_ECHO);
	CHECK(Rc(r) == DFUSMP_ERR_ENOTSUP);
	r = Req(Mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_PARAMS, nullptr, 2);
	CHECK(Rc(r) == DFUSMP_ERR_TOO_NEW && r.Ver == 1);

	// Malformed headers and responses get nothing back.
	uint8_t bad[12] = { 0x08, 0, 0, 9, 0, 0, 1, 0, 0xA0 };
	uint8_t rsp[256];
	CHECK(Mgr.Process(bad, 9, rsp, sizeof(rsp)) == 0);
	bad[3] = 1;
	bad[0] = 0x09;
	CHECK(Mgr.Process(bad, 9, rsp, sizeof(rsp)) == 0);
	CHECK(Mgr.Process(bad, 7, rsp, sizeof(rsp)) == 0);
	bad[0] = 0x08;
	CHECK(Mgr.Process(bad, 9, rsp, 16) == 0);

	// Random requests: no crash, and every response is well formed.
	std::mt19937 rng(3);
	for (int i = 0; i < 5000; i++)
	{
		uint8_t q[64];
		uint32_t n = 8 + rng() % 56;
		for (uint32_t k = 0; k < n; k++)
		{
			q[k] = (uint8_t)rng();
		}
		q[0] = (q[0] & 0x1A);
		q[2] = 0;
		q[3] = (uint8_t)(n - 8);
		q[4] = 0;
		q[5] &= 1;
		q[7] &= 7;
		int l = Mgr.Process(q, n, rsp, sizeof(rsp));
		if (l > 0)
		{
			CHECK(l >= 9 && ((rsp[2] << 8) | rsp[3]) == l - 8);
		}
	}
}

// Slot 1 through an Nvm (bTgt false, the way an nRF52 SoftDevice or nRF54L
// MPSL application does it) or through the target layer (bTgt true).
static void TestImg(HostMem Kind, bool bTgt)
{
	HostFlashInit(Kind);

	HostNvm slot1(HOST_SLOT1, HOST_SLOT1_SIZE);
	DfuStore_t st;
	if (bTgt)
	{
		CHECK(DfuStoreTgt(&st, HOST_SLOT1, HOST_SLOT1_SIZE));
	}
	else
	{
		CHECK(DfuStoreNvm(&st, &slot1));
	}
	static DfuMgr dm;
	DfuMgrCfg_t dcfg = {
		.pStore = &st, .bDirect = false, .bManifestOnly = false,
		.bAllowDowngrade = true, .pHash = s_Sha, .pBoot = nullptr,
	};
	DfuSmp mgr;
	DfuSmpCfg_t cfg = {
		.pMgr = &dm,
		.BufSize = 1024,
		.ResetCB = ResetCB,
		.pCtx = nullptr,
	};

	// Slot 1 memory of the wrong size is refused, and direct mode without
	// the keys, and a manager not set up.
	HostNvm small(HOST_SLOT1, HOST_SLOT1_SIZE - HOST_UNIT);
	DfuStore_t sst;
	CHECK(DfuStoreNvm(&sst, &small));
	DfuMgrCfg_t bcfg = dcfg;
	bcfg.pStore = &sst;
	DfuMgr bm;
	CHECK(bm.Init(bcfg) == false);
	bcfg = dcfg;
	bcfg.bDirect = true;
	CHECK(bm.Init(bcfg) == false);
	DfuSmpCfg_t ncfg = cfg;
	ncfg.pMgr = &bm;
	CHECK(mgr.Init(ncfg) == false);
	CHECK(dm.Init(dcfg));
	CHECK(mgr.Init(cfg));

	TestOs(mgr);

	std::vector<uint8_t> v1 = Img("v1.bin"), v2 = Img("v2.bin"),
						 vp = Img("v2_prot.bin");
	std::vector<uint8_t> h1 = ShaTlv(v1), h2 = ShaTlv(v2), hp = ShaTlv(vp);
	std::mt19937 rng((unsigned)Kind * 2 + bTgt);
	bool match = false;

	// Empty device: slot 0 has no record.
	std::vector<ImgEntry> l = List(mgr);
	CHECK(l.size() == 1 && l[0].Slot == 0 && l[0].Ver == "0.0.0" &&
		  l[0].Hash.empty() && l[0].bActive);

	// Upload v1, odd pieces, then list: slot 1 not pending.
	CHECK(Upload(mgr, v1, rng, 300, &match));
	CHECK(match);
	l = List(mgr);
	CHECK(l.size() == 2 && l[1].Slot == 1 && l[1].Ver == "1.0.0" &&
		  l[1].Hash == h1 && l[1].bPending == false);

	// Test with the hash: pending. Twice is the same.
	Rsp r = SetState(mgr, &h1, false);
	CHECK(Rc(r) == 0);
	l = List(mgr);
	CHECK(l.size() == 2 && l[1].bPending);
	r = SetState(mgr, &h1, true);
	CHECK(Rc(r) == 0);

	// Reset: the boot installs it.
	CHECK(Reboot());
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Ver == "1.0.0" && l[0].Hash == h1 &&
		  l[0].bConfirmed && l[0].bActive);

	// Confirm the running image, with and without its hash.
	r = SetState(mgr, nullptr, true);
	CHECK(Rc(r) == 0);
	r = SetState(mgr, &h1, true);
	CHECK(Rc(r) == 0);
	r = SetState(mgr, nullptr, false);
	CHECK(Rc(r) == DFUSMP_ERR_EINVAL);

	// Installed image cannot be marked again: slot 1 has been consumed.
	r = SetState(mgr, &h2, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_HASH_NOT_FOUND);

	// Interrupted uploads: a restart from 0, a resend of an old piece, a
	// piece past where the upload is. Each answer says where to go on.
	r = Chunk(mgr, v2, 0, 700, false);
	CHECK(UploadOff(r) == 700);
	r = Chunk(mgr, v2, 700, 500, false);
	CHECK(UploadOff(r) == 1200);
	r = Chunk(mgr, v2, 700, 500, false);
	CHECK(UploadOff(r) == 1200);
	r = Chunk(mgr, v2, 5000, 100, false);
	CHECK(UploadOff(r) == 1200);
	l = List(mgr);
	CHECK(l.size() == 1);
	r = SetState(mgr, &h2, false);
	CHECK(Rc(r) == DFUSMP_ERR_EBUSY);
	r = Req(mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_ERASE);
	CHECK(Rc(r) == DFUSMP_ERR_EBUSY);
	// The link gone: the upload is dropped, the server is not busy any more.
	mgr.UploadAbort();
	CHECK(mgr.Uploading() == false);
	r = SetState(mgr, &h2, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_HASH_NOT_FOUND);
	CHECK(Upload(mgr, v2, rng, 1000, &match) && match);

	// A protected TLV image uploaded over it before marking: the last one wins.
	CHECK(Upload(mgr, vp, rng, 257, &match) && match);
	r = SetState(mgr, &h2, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_HASH_NOT_FOUND);
	r = SetState(mgr, &hp, false);
	CHECK(Rc(r) == 0);
	CHECK(Reboot());
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Ver == "2.2.0" && l[0].Hash == hp);

	// Errors, version 2 and version 1.
	r = Chunk(mgr, v2, 0, 64, false);
	std::vector<uint8_t> junk(v2);
	junk[0] ^= 0xFF;
	r = Chunk(mgr, junk, 0, 64, false, 1);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_INVALID_MAGIC);
	r = Chunk(mgr, junk, 0, 64, false, 0);
	CHECK(Rc(r) == DFUSMP_ERR_EINVAL);

	std::vector<uint8_t> huge(HOST_SLOT1_SIZE - 2 * HostLayout().Unit + 1, 0);
	memcpy(huge.data(), v1.data(), 64);
	r = Chunk(mgr, huge, 0, 64, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_TOO_LARGE);

	r = Req(mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_UPLOAD,
		[&](CborWr_t &w) { CborPutMap(&w, 3); CborPutStr(&w, "image");
			CborPutUint(&w, 1); CborPutStr(&w, "off"); CborPutUint(&w, 0);
			CborPutStr(&w, "data"); CborPutBstr(&w, v1.data(), 16); });
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_INVALID_SLOT);
	r = Req(mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_UPLOAD,
		[&](CborWr_t &w) { CborPutMap(&w, 2); CborPutStr(&w, "off");
			CborPutUint(&w, 0); CborPutStr(&w, "data");
			CborPutBstr(&w, v1.data(), 16); });
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_INVALID_LENGTH);

	// Last piece longer than announced.
	r = Req(mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_UPLOAD,
		[&](CborWr_t &w) { CborPutMap(&w, 3); CborPutStr(&w, "len");
			CborPutUint(&w, 100); CborPutStr(&w, "off"); CborPutUint(&w, 0);
			CborPutStr(&w, "data"); CborPutBstr(&w, v1.data(), 200); });
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_DATA_OVERRUN);

	// A changed payload is found at the end of the upload.
	std::vector<uint8_t> tam = Img("v1_tamper.bin");
	CHECK(Upload(mgr, tam, rng, 400, &match) == false);
	l = List(mgr);
	CHECK(l.size() == 2);
	std::vector<uint8_t> ht = ShaTlv(tam);
	r = SetState(mgr, &ht, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_INVALID_HASH);

	// Bad vector table: refused at the end of the upload, and when marked.
	std::vector<uint8_t> vec = Img("vec.bin"), hv = ShaTlv(vec);
	CHECK(Upload(mgr, vec, rng, 400, &match) == false);
	r = SetState(mgr, &hv, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_VECTOR_TABLE);

	// A bad signature passes the server, which does not hold the key; the
	// boot refuses it and keeps the running image.
	std::vector<uint8_t> bs = Img("v1_badsig.bin"), hb = ShaTlv(bs);
	CHECK(Upload(mgr, bs, rng, 400, &match) && match);
	r = SetState(mgr, &hb, false);
	CHECK(Rc(r) == 0);
	CHECK(Reboot());
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Hash == hp);

	// Erase, then the list shows no slot 1.
	CHECK(Upload(mgr, v1, rng, 400, &match) && match);
	l = List(mgr);
	CHECK(l.size() == 2);
	r = Req(mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_ERASE);
	CHECK(Rc(r) == 0);
	l = List(mgr);
	CHECK(l.size() == 1);

	// The memory busy now and then: retried, nothing lost.
	slot1.vBusyLeft = bTgt ? 0 : 3;
	CHECK(Upload(mgr, v2, rng, 600, &match) && match);
	r = SetState(mgr, &h2, false);
	CHECK(Rc(r) == 0);
	CHECK(Reboot());
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Hash == h2 && l[0].Ver == "2.1.3.7");

	CHECK(g_HostNorViolations == 0);
	printf("  %s %s: %d writes, %d erases\n", HostMemName(Kind),
		   bTgt ? "target" : "Nvm", slot1.vWrites, slot1.vErases);
}

// Power lost at the n-th memory operation of a direct upload. The requests
// are built here, with nothing that owns memory on the stack, so the jump
// out of the cut leaves nothing behind.
static bool CutUpload(DfuSmp &Mgr, const std::vector<uint8_t> &File, int n)
{
	static uint8_t req[1024], rsp[512];
	static uint32_t off;

	off = 0;
	g_HostFailAfter = n;
	if (setjmp(g_HostPowerJmp) != 0)
	{
		g_HostFailAfter = -1;
		Mgr.UploadAbort();
		return true;
	}
	while (off < File.size())
	{
		uint32_t l = File.size() - off < 500 ? (uint32_t)(File.size() - off) : 500;
		CborWr_t w;

		CborWrInit(&w, req + 8, sizeof(req) - 8);
		CborPutMap(&w, off == 0 ? 3 : 2);
		if (off == 0)
		{
			CborPutStr(&w, "len");
			CborPutUint(&w, File.size());
		}
		CborPutStr(&w, "off");
		CborPutUint(&w, off);
		CborPutStr(&w, "data");
		CborPutBstr(&w, File.data() + off, l);
		req[0] = DFUSMP_OP_WRITE | (1 << 3);
		req[1] = 0;
		req[2] = (uint8_t)(w.Len >> 8);
		req[3] = (uint8_t)w.Len;
		req[4] = 0;
		req[5] = DFUSMP_GRP_IMG;
		req[6] = 0;
		req[7] = DFUSMP_IMG_UPLOAD;
		(void)Mgr.Process(req, 8 + w.Len, rsp, sizeof(rsp));
		off += l;
	}
	g_HostFailAfter = -1;

	return false;
}

// Stage 0 recovery: uploads straight to slot 0 and the record.
static void TestDirect(HostMem Kind)
{
	HostFlashInit(Kind);

	DfuStore_t st;
	CHECK(DfuStoreTgt(&st, HOST_SLOT0, HOST_SLOT0_SIZE));
	static DfuMgr dm;
	DfuMgrCfg_t dcfg = {
		.pStore = &st, .bDirect = true, .bManifestOnly = false,
		.bAllowDowngrade = true, .pHash = s_Sha, .pBoot = &s_BootCfg,
	};
	CHECK(dm.Init(dcfg));
	DfuSmp mgr;
	DfuSmpCfg_t cfg = {
		.pMgr = &dm,
		.BufSize = 1024,
		.ResetCB = ResetCB,
		.pCtx = nullptr,
	};
	CHECK(mgr.Init(cfg));

	std::vector<uint8_t> v1 = Img("v1.bin"), v2 = Img("v2.bin"),
						 vp = Img("v2_prot.bin");
	std::vector<uint8_t> h1 = ShaTlv(v1), h2 = ShaTlv(v2), hp = ShaTlv(vp);
	std::mt19937 rng(10 + (unsigned)Kind);
	bool match = false;

	Rsp r = Req(mgr, DFUSMP_OP_READ, DFUSMP_GRP_OS, DFUSMP_OS_BOOTLOADER,
		[](CborWr_t &w) { CborPutMap(&w, 1); CborPutStr(&w, "query");
						  CborPutStr(&w, "mode"); });
	CborFld_t m[] = { CBOR_FLD("mode", CBOR_FLD_INT) };
	CHECK(CborMapRead(r.Body.data(), (uint32_t)r.Body.size(), m, 1));
	CHECK(m[0].I == DFUSMP_BOOT_MODE_SINGLE);

	// Nothing there.
	std::vector<ImgEntry> l = List(mgr);
	CHECK(l.size() == 1 && l[0].Ver == "0.0.0" && l[0].bActive == false);
	CHECK(Reboot() == false);

	// Uploaded in odd pieces, first ones shorter than the header: in place
	// and startable at the next reset.
	CHECK(Upload(mgr, v1, rng, 7, &match) && match);
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Hash == h1 && l[0].Ver == "1.0.0");
	CHECK(Reboot());

	// A padded header, then a protected TLV.
	CHECK(Upload(mgr, v2, rng, 700, &match) && match);
	CHECK(Reboot());
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Hash == h2);
	CHECK(Upload(mgr, vp, rng, 300, &match) && match);
	CHECK(Reboot());

	// Confirm and test of the image there: nothing to do. Another hash is
	// not found, there is no slot 1.
	r = SetState(mgr, &hp, true);
	CHECK(Rc(r) == 0);
	r = SetState(mgr, &h1, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_HASH_NOT_FOUND);

	// Refused at the end of the upload; slot 0 is then not startable.
	struct { const char *pName; int Grc; } bad[] = {
		{ "v1_badsig.bin", DFUSMP_IMG_ERR_INVALID_TLV },
		{ "v2_key2.bin", DFUSMP_IMG_ERR_INVALID_TLV },
		{ "v2_nosig.bin", DFUSMP_IMG_ERR_INVALID_TLV },
		{ "v1_tamper.bin", DFUSMP_IMG_ERR_INVALID_HASH },
		{ "vec.bin", DFUSMP_IMG_ERR_VECTOR_TABLE },
	};
	for (auto &b : bad)
	{
		std::vector<uint8_t> f = Img(b.pName);
		uint32_t off = 0;
		Rsp last;
		while (off < f.size())
		{
			uint32_t n = f.size() - off < 400 ? (uint32_t)(f.size() - off) : 400;
			last = Chunk(mgr, f, off, n, true);
			off += n;
		}
		CHECK(GrpRc(last) == b.Grc);
		CHECK(Reboot() == false);
		CHECK(Upload(mgr, v1, rng, 400, &match) && match);
		CHECK(Reboot());
	}

	// Too large for slot 0.
	std::vector<uint8_t> huge(DFU_IMG_HDR_MAX + HOST_SLOT0_SIZE +
							  DFU_IMG_TLV_MAX + 1, 0);
	memcpy(huge.data(), v1.data(), 64);
	r = Chunk(mgr, huge, 0, 64, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_TOO_LARGE);
	CHECK(Reboot());

	// A header claiming a payload larger than slot 0.
	std::vector<uint8_t> big(v1);
	uint32_t isz = HOST_SLOT0_SIZE + 4;
	memcpy(big.data() + 12, &isz, 4);
	r = Chunk(mgr, big, 0, 64, false);
	CHECK(GrpRc(r) == DFUSMP_IMG_ERR_TOO_LARGE);
	CHECK(Reboot() == false);

	// An upload left half way: nothing startable. Then a whole one.
	CHECK(Upload(mgr, v1, rng, 400, &match) && match);
	r = Chunk(mgr, v2, 0, 1500, false);
	CHECK(UploadOff(r) == 1500);
	mgr.UploadAbort();
	CHECK(Reboot() == false);
	CHECK(Upload(mgr, v2, rng, 400, &match) && match);
	CHECK(Reboot());

	// Larger than the small sectors of the SECT kind, into its large one.
	std::vector<uint8_t> v3 = Img("v3.bin"), h3 = ShaTlv(v3);
	CHECK(Upload(mgr, v3, rng, 777, &match) && match);
	CHECK(Reboot());
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Hash == h3);

	// Erase: nothing startable.
	r = Req(mgr, DFUSMP_OP_WRITE, DFUSMP_GRP_IMG, DFUSMP_IMG_ERASE);
	CHECK(Rc(r) == 0);
	CHECK(Reboot() == false);

	// Power lost anywhere in an upload: never a startable partial image, and
	// the next whole upload starts.
	int cuts = 0;
	for (int n = 0; n < 400; n += 7)
	{
		CHECK(Upload(mgr, v1, rng, 400, &match) && match);
		if (CutUpload(mgr, v2, n) == false)
		{
			CHECK(Reboot());
			break;
		}
		cuts++;
		// Nothing starts, unless the cut came at the record magic, the last
		// write, when the whole image is in and verified.
		if (Reboot())
		{
			l = List(mgr);
			CHECK(l.size() == 1 && l[0].Hash == h2);
		}
	}
	CHECK(cuts > 3);
	CHECK(Upload(mgr, v2, rng, 400, &match) && match);
	CHECK(Reboot());
	l = List(mgr);
	CHECK(l.size() == 1 && l[0].Hash == h2);

	CHECK(g_HostNorViolations == 0);
	printf("  %s direct: %d power cut points\n", HostMemName(Kind), cuts);
}

int main(int argc, char **argv)
{
	s_Dir = argc > 1 ? argv[1] : "build/img";

	s_Sha = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem));
	s_Key1 = Img("key1.der");
	s_Key = { s_Key1.data(), (uint32_t)s_Key1.size() };
	s_BootCfg.pHash = s_Sha;
	s_BootCfg.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr);
	s_BootCfg.pKey = &s_Key;
	s_BootCfg.NbKey = 1;
	s_BootCfg.bVerifySlot0 = true;
	s_BootCfg.bAllowNoRec = false;

	const HostMem kinds[] = {
		HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16, HOST_MEM_BIG256,
	};
	for (HostMem k : kinds)
	{
		// A program unit above 16 bytes only exists on parts updated by
		// wire, with no slot 1: the slot 1 server refuses such a memory.
		if (k == HOST_MEM_BIG256)
		{
			HostFlashInit(k);
			DfuStore_t st;
			DfuMgr dm;
			CHECK(DfuStoreTgt(&st, HOST_SLOT1, HOST_SLOT1_SIZE));
			DfuMgrCfg_t cfg = {
				.pStore = &st, .bDirect = false, .bManifestOnly = false,
				.bAllowDowngrade = true, .pHash = s_Sha, .pBoot = nullptr,
			};
			CHECK(dm.Init(cfg) == false);
		}
		else
		{
			TestImg(k, false);
			TestImg(k, true);
		}
		TestDirect(k);
	}
	TestDirect(HOST_MEM_SECT);

	printf("dfu_smp_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail != 0;
}
