// DfuMgr, the API every DFU protocol uses, on simulated memory.
//
// Manifest first: every bad manifest is refused with no memory operation at
// all, good ones install in pieces of any size with resends, older versions
// are refused, a tampered payload never becomes startable, and power lost at
// any memory operation leaves either the old image, no image, or the new one.
// Image in order: refused when manifest only, signature checked at Finish.
// On every memory kind, direct (stage 0) and slot 1 (application).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "crc.h"
#include "dfu/dfu_mgr.h"
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

static std::vector<uint8_t> Img(const char *pName)
{
	return HostReadFile(s_Dir + "/" + pName);
}

// An image as the host tool splits it: manifest (header + TLVs), payload.
struct Split {
	std::vector<uint8_t> Man;
	std::vector<uint8_t> Pay;
};

static Split SplitImg(const std::vector<uint8_t> &F)
{
	DfuImgHdr_t h;
	memcpy(&h, F.data(), sizeof(h));

	Split s;
	s.Man.assign(F.begin(), F.begin() + h.HdrSize);
	s.Man.insert(s.Man.end(), F.begin() + h.HdrSize + h.ImgSize, F.end());
	s.Pay.assign(F.begin() + h.HdrSize, F.begin() + h.HdrSize + h.ImgSize);

	return s;
}

// The boot, as after a reset. Returns the address started, 0 when none.
static uintptr_t Reboot(void)
{
	if (setjmp(g_HostStartJmp) != 0)
	{
		return g_HostStartAddr;
	}
	(void)DfuBootRun(s_BootCfg);
	return 0;
}

// Version of slot 0 through its record, major only, -1 when none.
static int Slot0Major(void)
{
	DfuImgInfo_t info;
	DfuLayout_t lay = HostLayout();

	return DfuSlot0Parse(lay, &info) == DFU_IMG_OK ? info.Hdr.Ver.Major : -1;
}

// A whole manifest upload in random pieces, some sent twice or overlapping.
static int Send(DfuMgr &Mgr, const Split &S, std::mt19937 &Rng, bool bCommit)
{
	int res = Mgr.BeginManifest(S.Man.data(), (uint32_t)S.Man.size());
	if (res != DFU_IMG_OK)
	{
		return res;
	}

	uint32_t off = 0;

	while (off < S.Pay.size())
	{
		uint32_t l = 1 + Rng() % 700;
		if (l > S.Pay.size() - off)
		{
			l = (uint32_t)(S.Pay.size() - off);
		}

		// Now and then back up a little, as a host resending would.
		uint32_t from = off;
		if (off > 64 && Rng() % 5 == 0)
		{
			from = off - 1 - Rng() % 64;
		}
		uint32_t len = l + (off - from);

		res = Mgr.Write(from, S.Pay.data() + from, len);
		if (res != DFU_IMG_OK)
		{
			return res;
		}
		off += l;
		CHECK(Mgr.Offset() == off);
		CHECK(Mgr.Crc() == crc32_ieee((uint8_t *)S.Pay.data(), (int)off));
	}

	res = Mgr.Finish(nullptr);
	if (res != DFU_IMG_OK)
	{
		return res;
	}

	return bCommit ? (Mgr.Commit() ? DFU_IMG_OK : DFU_IMG_ERR_READ) : res;
}

// Each bad manifest: refused, and not one erase or write.
static void BadManifests(DfuMgr &Mgr)
{
	std::vector<std::vector<uint8_t>> bad;

	bad.push_back(SplitImg(Img("v1_badsig.bin")).Man);
	bad.push_back(SplitImg(Img("v2_key2.bin")).Man);
	bad.push_back(SplitImg(Img("v2_nosig.bin")).Man);

	std::vector<uint8_t> m = SplitImg(Img("v2.bin")).Man;
	std::vector<uint8_t> t;

	t = m; t[0] ^= 1; bad.push_back(t);					// magic
	t = m; t.push_back(0); bad.push_back(t);			// junk after TLVs
	t = m; t.pop_back(); bad.push_back(t);				// TLVs cut short
	t.assign(m.begin(), m.begin() + 32); bad.push_back(t);	// header alone
	t.assign(m.begin(), m.begin() + 16); bad.push_back(t);	// half a header
	t = m; t[12] = 0xFF; t[13] = 0xFF; t[14] = 0xFF; t[15] = 0x7F;
	bad.push_back(t);									// ImgSize huge
	t = m; t[8] = 0x10; t[9] = 0; bad.push_back(t);		// HdrSize 16
	t = m; t[t.size() - 5] ^= 0x40; bad.push_back(t);	// signature bytes

	for (size_t i = 0; i < bad.size(); i++)
	{
		int ops = g_HostMemOps;
		int res = Mgr.BeginManifest(bad[i].data(), (uint32_t)bad[i].size());

		CHECK(res != DFU_IMG_OK);
		CHECK(g_HostMemOps == ops);
		CHECK(Mgr.State() == DFU_MGR_STATE_IDLE);
		if (res == DFU_IMG_OK || g_HostMemOps != ops)
		{
			fprintf(stderr, "  bad manifest %zu: res %d, ops %d\n", i, res,
					g_HostMemOps - ops);
		}
	}

	int ops = g_HostMemOps;
	CHECK(Mgr.BeginManifest(nullptr, 100) != DFU_IMG_OK);
	CHECK(Mgr.BeginManifest(m.data(), DFU_MGR_MANIFEST_MAX + 1) != DFU_IMG_OK);
	CHECK(g_HostMemOps == ops);
}

// Power lost at the n-th memory operation of a manifest upload. Nothing that
// owns memory is live across the jump.
static bool CutSend(DfuMgr &Mgr, const Split &S, int n)
{
	static uint32_t off;

	off = 0;
	g_HostFailAfter = n;
	if (setjmp(g_HostPowerJmp) != 0)
	{
		g_HostFailAfter = -1;
		Mgr.Abort();
		return true;
	}
	if (Mgr.BeginManifest(S.Man.data(), (uint32_t)S.Man.size()) == DFU_IMG_OK)
	{
		while (off < S.Pay.size())
		{
			uint32_t l = S.Pay.size() - off < 512 ? (uint32_t)(S.Pay.size() - off)
												  : 512;
			(void)Mgr.Write(off, S.Pay.data() + off, l);
			off += l;
		}
		if (Mgr.Finish(nullptr) == DFU_IMG_OK)
		{
			(void)Mgr.Commit();
		}
	}
	g_HostFailAfter = -1;

	return false;
}

static void TestDirect(HostMem Kind)
{
	HostFlashInit(Kind);
	printf("direct %s\n", HostMemName(Kind));

	DfuStore_t st;
	CHECK(DfuStoreTgt(&st, HOST_SLOT0, HOST_SLOT0_SIZE));

	DfuMgrCfg_t cfg = {
		.pStore = &st, .bDirect = true, .bManifestOnly = true,
		.bAllowDowngrade = false, .pHash = s_Sha, .pBoot = &s_BootCfg,
	};
	DfuMgr mgr;

	// Direct mode without keys is refused.
	DfuMgrCfg_t nokey = cfg;
	nokey.pBoot = nullptr;
	CHECK(mgr.Init(nokey) == false);

	CHECK(mgr.Init(cfg));

	std::mt19937 rng(100 + (unsigned)Kind);
	Split s1 = SplitImg(Img("v1.bin")), s2 = SplitImg(Img("v2.bin")),
		  sp = SplitImg(Img("v2_prot.bin")), s3 = SplitImg(Img("v3.bin"));

	// Nothing there: every bad manifest still touches nothing.
	BadManifests(mgr);
	CHECK(Reboot() == 0);

	// v1, then it boots.
	CHECK(Send(mgr, s1, rng, true) == DFU_IMG_OK);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 1);
	CHECK(mgr.CurVer().Major == 1);

	// With v1 in place, bad manifests leave it startable.
	BadManifests(mgr);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 1);

	// State rules.
	CHECK(mgr.Write(0, s1.Pay.data(), 4) == DFU_MGR_ERR_STATE);
	CHECK(mgr.Finish(nullptr) == DFU_MGR_ERR_STATE);
	CHECK(mgr.Commit() == false);
	CHECK(mgr.BeginImage(1000) == DFU_MGR_ERR_STATE);

	// Offset rules.
	CHECK(mgr.BeginManifest(s2.Man.data(), (uint32_t)s2.Man.size()) ==
		  DFU_IMG_OK);
	CHECK(mgr.Write(4, s2.Pay.data() + 4, 4) == DFU_MGR_ERR_OFFSET);
	CHECK(mgr.Write(0, s2.Pay.data(), 100) == DFU_IMG_OK);
	CHECK(mgr.Write(0, s2.Pay.data(), 50) == DFU_IMG_OK);	// resend
	CHECK(mgr.Offset() == 100);
	CHECK(mgr.Write(100, s2.Pay.data(), (uint32_t)s2.Pay.size()) ==
		  DFU_IMG_ERR_SIZE);
	CHECK(mgr.Finish(nullptr) == DFU_MGR_ERR_STATE);		// not all in
	mgr.Abort();
	// The record went at BeginManifest: nothing startable now.
	CHECK(Reboot() == 0);

	// v2 with a padded header, then v2 with protected TLVs.
	CHECK(Send(mgr, s2, rng, true) == DFU_IMG_OK);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 2);
	CHECK(Send(mgr, sp, rng, true) == DFU_IMG_OK);
	CHECK(Reboot() == HOST_SLOT0);

	// Older: refused before anything is touched, v2 stays.
	int ops = g_HostMemOps;
	CHECK(mgr.BeginManifest(s1.Man.data(), (uint32_t)s1.Man.size()) ==
		  DFU_MGR_ERR_VERSION);
	CHECK(g_HostMemOps == ops);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 2);

	// Tampered payload, good manifest: never startable.
	{
		Split t = s3;
		t.Pay[t.Pay.size() / 2] ^= 0x01;
		CHECK(Send(mgr, t, rng, true) == DFU_IMG_ERR_HASH);
		CHECK(mgr.Commit() == false);
		CHECK(Reboot() == 0);
	}

	// Checked but not committed: not startable either.
	CHECK(Send(mgr, s3, rng, false) == DFU_IMG_OK);
	CHECK(Reboot() == 0);
	CHECK(mgr.Commit());
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 3);

	// Downgrade allowed by configuration.
	DfuMgrCfg_t dcfg = cfg;
	dcfg.bAllowDowngrade = true;
	DfuMgr dm;
	CHECK(dm.Init(dcfg));
	CHECK(dm.CurVer().Major == 3);
	CHECK(Send(dm, s1, rng, true) == DFU_IMG_OK);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 1);

	// Power lost at every memory operation of a v2 upload over v1.
	int cuts = 0;
	for (int n = 0; ; n++)
	{
		CHECK(Send(dm, s1, rng, true) == DFU_IMG_OK);
		CHECK(Slot0Major() == 1);

		bool cut = CutSend(dm, s2, n);
		uintptr_t a = Reboot();
		int maj = Slot0Major();

		// Old, none, or new; never anything else.
		CHECK(a == 0 || a == HOST_SLOT0);
		CHECK(a == 0 || maj == 1 || maj == 2);
		if (cut == false)
		{
			CHECK(a == HOST_SLOT0 && maj == 2);
			break;
		}
		cuts++;

		// A new upload always recovers.
		CHECK(Send(dm, s2, rng, true) == DFU_IMG_OK);
		CHECK(Reboot() == HOST_SLOT0);
		CHECK(Slot0Major() == 2);
	}
	CHECK(cuts > 10);
	printf("  %d power cuts\n", cuts);

	// Image in order, allowed by configuration: signature checked at Finish.
	DfuMgrCfg_t icfg = cfg;
	icfg.bManifestOnly = false;
	icfg.bAllowDowngrade = true;
	DfuMgr im;
	CHECK(im.Init(icfg));

	std::vector<uint8_t> f = Img("v3.bin");
	CHECK(im.BeginImage((uint32_t)f.size()) == DFU_IMG_OK);
	CHECK(im.Write(0, f.data(), 100) == DFU_IMG_OK);
	CHECK(im.Write(100, f.data() + 100, (uint32_t)f.size() - 100) == DFU_IMG_OK);
	CHECK(im.Finish(nullptr) == DFU_IMG_OK);
	CHECK(im.Commit());
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 3);

	f = Img("v1_badsig.bin");
	CHECK(im.BeginImage((uint32_t)f.size()) == DFU_IMG_OK);
	CHECK(im.Write(0, f.data(), (uint32_t)f.size()) == DFU_IMG_OK);
	CHECK(im.Finish(nullptr) == DFU_IMG_ERR_SIG);
	CHECK(im.Commit() == false);
	CHECK(Reboot() == 0);
}

// Slot 1, as an application does: the boot installs.
static void TestSlot1(HostMem Kind)
{
	HostFlashInit(Kind);
	printf("slot 1 %s\n", HostMemName(Kind));

	DfuStore_t st;
	CHECK(DfuStoreTgt(&st, HOST_SLOT1, HOST_SLOT1_SIZE));

	DfuMgrCfg_t cfg = {
		.pStore = &st, .bDirect = false, .bManifestOnly = false,
		.bAllowDowngrade = false, .pHash = s_Sha, .pBoot = &s_BootCfg,
	};
	DfuMgr mgr;
	CHECK(mgr.Init(cfg));

	std::mt19937 rng(200 + (unsigned)Kind);
	Split s1 = SplitImg(Img("v1.bin")), s2 = SplitImg(Img("v2.bin"));

	// With keys given, a bad signature touches nothing here either.
	BadManifests(mgr);

	CHECK(Send(mgr, s1, rng, true) == DFU_IMG_OK);
	CHECK(HostPending() == DFU_TRAILER_PENDING);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 1);

	// A new DfuMgr, as after the application restarts, sees v1.
	DfuMgr m2;
	CHECK(m2.Init(cfg));
	CHECK(m2.CurVer().Major == 1);
	CHECK(Send(m2, s2, rng, true) == DFU_IMG_OK);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 2);

	// Without keys the manifest is only checked for form; the boot still
	// refuses a bad signature at install.
	DfuMgrCfg_t ncfg = cfg;
	ncfg.pBoot = nullptr;
	ncfg.bAllowDowngrade = true;
	DfuMgr nm;
	CHECK(nm.Init(ncfg));
	Split sb = SplitImg(Img("v1_badsig.bin"));
	CHECK(Send(nm, sb, rng, true) == DFU_IMG_OK);
	CHECK(Reboot() == HOST_SLOT0);
	CHECK(Slot0Major() == 2);
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

	const HostMem direct[] = {
		HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16, HOST_MEM_BIG256,
		HOST_MEM_SECT,
	};
	for (HostMem k : direct)
	{
		TestDirect(k);
	}

	const HostMem slot1[] = { HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16 };
	for (HostMem k : slot1)
	{
		TestSlot1(k);
	}

	CHECK(g_HostNorViolations == 0);

	printf("dfu_mgr_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail ? 1 : 0;
}
