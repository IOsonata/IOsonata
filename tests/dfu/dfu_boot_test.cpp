// Stage 0 boot on simulated memory: install, refuse, verify at boot, and a
// power loss at every memory operation of an install, on NOR flash and RRAM.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include "dfu_host.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

static int s_Fail = 0;
static int s_Pass = 0;

#define CHECK(c) do { if (c) { s_Pass++; } else { s_Fail++; \
	fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)

static std::string s_Dir;

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];
static std::vector<uint8_t> s_Key1, s_Key2;
static DfuImgKey_t s_Keys[2];
static DfuBootCfg_t s_Cfg;

static std::vector<uint8_t> Img(const char *pName)
{
	return HostReadFile(s_Dir + "/" + pName);
}

// One boot. Returns 1 when it started slot 0, 0 when it stayed, -1 when the
// simulated power went.
static volatile int s_Res;

static int Boot(void)
{
	g_HostStartAddr = 0;
	if (setjmp(g_HostStartJmp) != 0)
	{
		return 1;
	}
	if (setjmp(g_HostPowerJmp) != 0)
	{
		return -1;
	}
	s_Res = DfuBootRun(s_Cfg);

	return 0;
}


// Slot 0 holds this signed image, checked against the file itself.
static bool Slot0Is(const std::vector<uint8_t> &File)
{
	const DfuImgHdr_t *h = (const DfuImgHdr_t *)File.data();
	DfuLayout_t lay;
	DfuImgInfo_t info;

	if (DfuLayoutGet(&lay) == false || DfuSlot0Parse(lay, &info) != DFU_IMG_OK)
	{
		return false;
	}
	if (memcmp(HostFlash(HOST_SLOT0), File.data() + h->HdrSize, h->ImgSize) != 0)
	{
		return false;
	}
	return DfuImgVerify(s_Cfg.pHash, s_Cfg.pSign, s_Keys[0], DfuSlot0Read, &lay,
						info) == DFU_IMG_OK;
}

static void Install(const std::vector<uint8_t> &File)
{
	HostPutSlot1(File, true);
	CHECK(Boot() == 1);
	CHECK(g_HostStartAddr == HOST_SLOT0);
	CHECK(Slot0Is(File));
	CHECK(HostDone() == DFU_TRAILER_DONE);
}

static void TestBasic(HostMem Kind)
{
	HostFlashInit(Kind);

	DfuLayout_t lay;
	CHECK(DfuLayoutGet(&lay));
	CHECK(lay.Slot0 == HOST_SLOT0 && lay.Slot1Size == HOST_SLOT1_SIZE &&
		  lay.RecSize == HOST_REC_SIZE);

	// Nothing anywhere.
	CHECK(Boot() == 0 && s_Res == DFU_IMG_ERR_MAGIC);

	// No record, an application a debugger put there.
	std::vector<uint8_t> v1 = Img("v1.bin");
	memcpy(HostFlash(HOST_SLOT0), v1.data() + 0x20, v1.size() - 0x20);
	CHECK(Boot() == 0);
	s_Cfg.bAllowNoRec = true;
	CHECK(Boot() == 1);
	s_Cfg.bAllowNoRec = false;
	HostFlashErase();

	// Install, then boot again with nothing pending.
	Install(v1);
	CHECK(Boot() == 1);

	// Upgrade with a padded header, and with a protected TLV.
	std::vector<uint8_t> v2 = Img("v2.bin");
	Install(v2);
	const DfuRecInfo_t *rec = (const DfuRecInfo_t *)HostFlash(HOST_REC +
															  lay.Unit);
	CHECK(rec->HdrLen == 0x200);
	CHECK(*(const uint32_t *)HostFlash(HOST_REC) == DFU_REC_MAGIC);
	std::vector<uint8_t> vp = Img("v2_prot.bin");
	Install(vp);
	Install(v1);

	// Refused images: marked done, slot 0 untouched, old image starts.
	const char *bad[] = {
		"v1_tamper.bin", "v1_badsig.bin", "v2_nosig.bin", "vec.bin",
	};
	for (const char *b : bad)
	{
		HostPutSlot1(Img(b), true);
		CHECK(Boot() == 1);
		CHECK(HostDone() == DFU_TRAILER_DONE);
		CHECK(Slot0Is(v1));
	}

	// Signed with the second key: refused with one key, taken with both.
	std::vector<uint8_t> k2 = Img("v2_key2.bin");
	HostPutSlot1(k2, true);
	CHECK(Boot() == 1 && Slot0Is(v1));
	s_Cfg.NbKey = 2;
	HostPutSlot1(k2, true);
	CHECK(Boot() == 1);
	CHECK(memcmp(HostFlash(HOST_SLOT0), k2.data() + 0x20,
				 ((const DfuImgHdr_t *)k2.data())->ImgSize) == 0);
	s_Cfg.NbKey = 1;

	// Now only the second key signed slot 0: with one key it does not start.
	CHECK(Boot() == 0 && s_Res == DFU_IMG_ERR_KEY);
	Install(v1);

	// Not pending: nothing happens, whatever slot 1 holds.
	HostPutSlot1(v2, false);
	CHECK(Boot() == 1 && Slot0Is(v1));

	// Slot 0 corrupted after the install: refused at boot when verifying,
	// started when the configuration trusts the install check.
	HostFlash(HOST_SLOT0)[100] ^= 0x40;
	CHECK(Boot() == 0 && s_Res == DFU_IMG_ERR_HASH);
	s_Cfg.bVerifySlot0 = false;
	CHECK(Boot() == 1);
	s_Cfg.bVerifySlot0 = true;
	HostFlash(HOST_SLOT0)[100] ^= 0x40;

	// Record damaged: no image.
	HostFlash(HOST_REC + lay.Unit)[0] ^= 0x01;
	CHECK(Boot() == 0);
	HostFlash(HOST_REC + lay.Unit)[0] ^= 0x01;
	CHECK(Boot() == 1);
	HostFlash(HOST_REC)[0] ^= 0x01;
	CHECK(Boot() == 0 && s_Res == DFU_IMG_ERR_MAGIC);
	HostFlash(HOST_REC)[0] ^= 0x01;

	// Recovery asked by the application: the boot stays once, then starts.
	*(volatile uint32_t *)HostFlash(HOST_FLAG) = DFU_FLAG_RECOVERY;
	CHECK(Boot() == 0 && s_Res == DFU_IMG_OK);
	CHECK(*(volatile uint32_t *)HostFlash(HOST_FLAG) == 0);
	CHECK(Boot() == 1);

	// A pending image is installed before recovery is served.
	HostPutSlot1(v2, true);
	*(volatile uint32_t *)HostFlash(HOST_FLAG) = DFU_FLAG_RECOVERY;
	CHECK(Boot() == 0 && s_Res == DFU_IMG_OK && Slot0Is(v2));
	Install(v1);

	// A payload with a page of ones, then another image over it: on ECC
	// flash the page must be erased although it reads as erased.
	std::vector<uint8_t> v4 = Img("v4.bin");
	Install(v4);
	Install(v1);
	Install(v4);

	// A Done write cut by power loss: not erased, not DONE. It counts as
	// done, the image is not installed again.
	HostPutSlot1(v2, true);
	uint32_t torn = 0x454EFFFFUL;
	memcpy(HostFlash(HOST_SLOT1 + DfuTrailerOff(lay) + lay.Unit), &torn, 4);
	CHECK(Boot() == 1 && Slot0Is(v4));

	CHECK(g_HostNorViolations == 0);
}

// Power lost at the n-th memory operation of an install of v2 over v1, then
// boots until one starts. Whatever runs must be v1 or v2 in full, and in the
// end it must be v2.
static void TestPowerLoss(HostMem Kind)
{
	std::vector<uint8_t> v1 = Img("v1.bin"), v2 = Img("v2.bin");
	int cuts = 0;

	for (int n = 0; ; n++)
	{
		HostFlashInit(Kind);
		Install(v1);
		HostPutSlot1(v2, true);

		g_HostFailAfter = n;
		int r = Boot();
		bool cut = r == -1;
		g_HostFailAfter = -1;
		if (cut == false)
		{
			// The install finished before the n-th operation: every point
			// has been cut.
			CHECK(r == 1 && Slot0Is(v2));
			break;
		}
		cuts++;

		// The next boot after a cut.
		r = Boot();
		CHECK(r == 1);
		CHECK(Slot0Is(v2) || Slot0Is(v1));
		if (Slot0Is(v2) == false)
		{
			fprintf(stderr, "cut %d: v1 kept, v2 not installed\n", n);
		}
		CHECK(Slot0Is(v2));
		CHECK(HostDone() == DFU_TRAILER_DONE);

		// A second cut during the retry, then a clean boot.
		HostPutSlot1(v2, true);
		g_HostFailAfter = n / 2;
		(void)Boot();
		g_HostFailAfter = -1;
		CHECK(Boot() == 1 && Slot0Is(v2));
	}

	CHECK(cuts > 5);
	CHECK(g_HostNorViolations == 0);
	printf("  %s: %d power cut points\n", HostMemName(Kind), cuts);
}

int main(int argc, char **argv)
{
	s_Dir = argc > 1 ? argv[1] : "build/img";

	s_Key1 = Img("key1.der");
	s_Key2 = Img("key2.der");
	s_Keys[0] = { s_Key1.data(), (uint32_t)s_Key1.size() };
	s_Keys[1] = { s_Key2.data(), (uint32_t)s_Key2.size() };

	s_Cfg.pHash = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem));
	s_Cfg.pSign = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr);
	s_Cfg.pKey = s_Keys;
	s_Cfg.NbKey = 1;
	s_Cfg.bVerifySlot0 = true;
	s_Cfg.bAllowNoRec = false;

	const HostMem kinds[] = {
		HOST_MEM_NOR, HOST_MEM_RRAM, HOST_MEM_ECC16, HOST_MEM_BIG256,
	};
	for (HostMem k : kinds)
	{
		TestBasic(k);
	}
	for (HostMem k : kinds)
	{
		TestPowerLoss(k);
	}

	printf("dfu_boot_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail != 0;
}
