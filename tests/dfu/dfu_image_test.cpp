// CBOR reader and writer, and MCUboot image parsing and verification against
// images signed by imgtool (mkimages.py).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <random>
#include <string>
#include <vector>

#include "cbor.h"
#include "dfu/dfu_image.h"
#include "crypto/crypto_softsha256.h"
#include "crypto/crypto_uecc.h"

static int s_Fail = 0;
static int s_Pass = 0;

#define CHECK(c) do { if (c) { s_Pass++; } else { s_Fail++; \
	fprintf(stderr, "%s:%d: CHECK(%s)\n", __FILE__, __LINE__, #c); } } while (0)

static std::string s_Dir;

static std::vector<uint8_t> Load(const char *pName)
{
	std::string p = s_Dir + "/" + pName;
	FILE *f = fopen(p.c_str(), "rb");
	if (f == nullptr)
	{
		fprintf(stderr, "missing %s\n", p.c_str());
		exit(2);
	}
	std::vector<uint8_t> v;
	int c;
	while ((c = fgetc(f)) != EOF)
	{
		v.push_back((uint8_t)c);
	}
	fclose(f);
	return v;
}

struct Buf {
	const std::vector<uint8_t> *p;
};

static bool BufRead(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len)
{
	const std::vector<uint8_t> &v = *((Buf *)pCtx)->p;
	if (Off > v.size() || Len > v.size() - Off)
	{
		return false;
	}
	memcpy(pBuf, v.data() + Off, Len);
	return true;
}

alignas(16) static uint8_t s_ShaMem[CRYPTO_SOFTSHA256_MEMSIZE];
alignas(16) static uint8_t s_EccMem[CRYPTO_UECC_MEMSIZE];
static HashEngine *s_Sha;
static SignEngine *s_Ecc;

static int Check(const std::vector<uint8_t> &Img, const DfuImgKey_t &Key,
				 bool bSig, DfuImgInfo_t *pInfo = nullptr)
{
	Buf b = { &Img };
	DfuImgInfo_t info;
	int res = DfuImgParse(BufRead, &b, (uint32_t)Img.size(), &info);
	if (res == DFU_IMG_OK)
	{
		res = DfuImgVerify(s_Sha, bSig ? s_Ecc : nullptr, Key, BufRead, &b,
						   info);
	}
	if (pInfo != nullptr)
	{
		*pInfo = info;
	}
	return res;
}

static void TestCborRead(void)
{
	// {"off": 0, "data": h'0102', "len": 1000, "x": [1, {"y": -2}], "ok": true}
	static const uint8_t m1[] = {
		0xA5, 0x63, 'o', 'f', 'f', 0x00,
		0x64, 'd', 'a', 't', 'a', 0x42, 0x01, 0x02,
		0x63, 'l', 'e', 'n', 0x19, 0x03, 0xE8,
		0x61, 'x', 0x82, 0x01, 0xA1, 0x61, 'y', 0x21,
		0x62, 'o', 'k', 0xF5,
	};
	CborFld_t f[] = {
		CBOR_FLD("off", CBOR_FLD_UINT),
		CBOR_FLD("data", CBOR_FLD_BSTR),
		CBOR_FLD("len", CBOR_FLD_UINT),
		CBOR_FLD("ok", CBOR_FLD_BOOL),
		CBOR_FLD("none", CBOR_FLD_TSTR),
	};

	CHECK(CborMapRead(m1, sizeof(m1), f, 5));
	CHECK(f[0].bFound && f[0].U == 0);
	CHECK(f[1].bFound && f[1].S.Len == 2 && f[1].S.p[1] == 2);
	CHECK(f[2].bFound && f[2].U == 1000);
	CHECK(f[3].bFound && f[3].B == true);
	CHECK(f[4].bFound == false);

	// Indefinite map with an indefinite array, null, an integer key, and a
	// signed field: what Jackson based host libraries send.
	static const uint8_t m2[] = {
		0xBF,
		0x61, 'a', 0x9F, 0x01, 0xF6, 0xFF,
		0x01, 0x02,
		0x61, 'i', 0x38, 0x63,
		0x63, 'o', 'f', 'f', 0x1A, 0x00, 0x01, 0x00, 0x00,
		0xFF,
	};
	CborFld_t g[] = {
		CBOR_FLD("off", CBOR_FLD_UINT),
		CBOR_FLD("i", CBOR_FLD_INT),
	};
	CHECK(CborMapRead(m2, sizeof(m2), g, 2));
	CHECK(g[0].bFound && g[0].U == 0x10000);
	CHECK(g[1].bFound && g[1].I == -100);

	// Every prefix of a valid map is refused.
	for (size_t n = 0; n < sizeof(m1); n++)
	{
		CHECK(CborMapRead(m1, (uint32_t)n, f, 5) == false);
	}
	for (size_t n = 0; n < sizeof(m2); n++)
	{
		CHECK(CborMapRead(m2, (uint32_t)n, g, 2) == false);
	}

	// Trailing byte, duplicate key, wrong type, float, not a map.
	uint8_t t[sizeof(m1) + 1];
	memcpy(t, m1, sizeof(m1));
	t[sizeof(m1)] = 0x00;
	CHECK(CborMapRead(t, sizeof(t), f, 5) == false);

	static const uint8_t dup[] = { 0xA2, 0x61, 'd', 0x01, 0x61, 'd', 0x02 };
	CborFld_t d[] = { CBOR_FLD("d", CBOR_FLD_UINT) };
	CHECK(CborMapRead(dup, sizeof(dup), d, 1) == false);

	static const uint8_t wt[] = { 0xA1, 0x61, 'd', 0x41, 0x00 };
	CHECK(CborMapRead(wt, sizeof(wt), d, 1) == false);

	static const uint8_t fl[] = { 0xA1, 0x61, 'z', 0xF9, 0x3C, 0x00 };
	CHECK(CborMapRead(fl, sizeof(fl), d, 1) == false);

	static const uint8_t arr[] = { 0x80 };
	CHECK(CborMapRead(arr, sizeof(arr), d, 1) == false);

	// A count larger than the input.
	static const uint8_t big[] = { 0xBA, 0xFF, 0xFF, 0xFF, 0xFF };
	CHECK(CborMapRead(big, sizeof(big), d, 1) == false);
	static const uint8_t bs[] = { 0xA1, 0x61, 'd', 0x5A, 0x7F, 0xFF, 0xFF, 0xFF };
	CborFld_t e[] = { CBOR_FLD("d", CBOR_FLD_BSTR) };
	CHECK(CborMapRead(bs, sizeof(bs), e, 1) == false);

	// Deep nesting stops at the limit instead of recursing on.
	std::vector<uint8_t> deep = { 0xA1, 0x61, 'q' };
	for (int i = 0; i < 64; i++)
	{
		deep.push_back(0x81);
	}
	deep.push_back(0x00);
	CHECK(CborMapRead(deep.data(), (uint32_t)deep.size(), d, 1) == false);

	// Random input: never a crash, which the sanitizers watch.
	std::mt19937 rng(7);
	for (int i = 0; i < 20000; i++)
	{
		uint8_t r[48];
		uint32_t n = rng() % sizeof(r);
		for (uint32_t k = 0; k < n; k++)
		{
			r[k] = (uint8_t)rng();
		}
		if (n > 0 && (i & 1))
		{
			r[0] = 0xA0 | (r[0] & 7);
		}
		(void)CborMapRead(r, n, f, 5);
	}
}

static void TestCborWrite(void)
{
	uint8_t b[64];
	CborWr_t w;

	CborWrInit(&w, b, sizeof(b));
	CborPutMap(&w, 3);
	CborPutStr(&w, "rc");
	CborPutInt(&w, -1);
	CborPutStr(&w, "n");
	CborPutUint(&w, 0x12345678);
	CborPutStr(&w, "t");
	CborPutBool(&w, true);

	static const uint8_t exp[] = {
		0xA3, 0x62, 'r', 'c', 0x20, 0x61, 'n', 0x1A, 0x12, 0x34, 0x56, 0x78,
		0x61, 't', 0xF5,
	};
	CHECK(w.bOvf == false && w.Len == sizeof(exp) &&
		  memcmp(b, exp, sizeof(exp)) == 0);

	// What was written reads back.
	CborFld_t f[] = {
		CBOR_FLD("rc", CBOR_FLD_INT),
		CBOR_FLD("n", CBOR_FLD_UINT),
		CBOR_FLD("t", CBOR_FLD_BOOL),
	};
	CHECK(CborMapRead(b, w.Len, f, 3));
	CHECK(f[0].I == -1 && f[1].U == 0x12345678 && f[2].B);

	// 64 bit and boundary heads.
	CborWrInit(&w, b, sizeof(b));
	CborPutUint(&w, 23);
	CborPutUint(&w, 24);
	CborPutUint(&w, 0x100);
	CborPutUint(&w, 0x100000000ULL);
	static const uint8_t exp2[] = {
		0x17, 0x18, 0x18, 0x19, 0x01, 0x00,
		0x1B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	};
	CHECK(w.Len == sizeof(exp2) && memcmp(b, exp2, sizeof(exp2)) == 0);

	// Overflow is sticky and writes nothing past the end.
	uint8_t s[8];
	memset(s, 0xEE, sizeof(s));
	CborWrInit(&w, s, 4);
	CborPutStr(&w, "abcdef");
	CborPutUint(&w, 1);
	CHECK(w.bOvf && w.Len <= 4 && s[4] == 0xEE);
}

static void TestImages(void)
{
	std::vector<uint8_t> k1 = Load("key1.der"), k2 = Load("key2.der");
	DfuImgKey_t key1 = { k1.data(), (uint32_t)k1.size() };
	DfuImgKey_t key2 = { k2.data(), (uint32_t)k2.size() };
	DfuImgInfo_t info;

	std::vector<uint8_t> v1 = Load("v1.bin");
	CHECK(Check(v1, key1, true, &info) == DFU_IMG_OK);
	CHECK(info.Hdr.HdrSize == 0x20 && info.Hdr.ImgSize == 8 * 1024 + 3);
	CHECK(info.Hdr.Ver.Major == 1 && info.Hdr.Ver.Minor == 0);
	CHECK(info.TotalLen == v1.size());
	CHECK(info.bKeyHash && info.bSig);

	std::vector<uint8_t> v2 = Load("v2.bin");
	CHECK(Check(v2, key1, true, &info) == DFU_IMG_OK);
	CHECK(info.Hdr.HdrSize == 0x200 && info.Hdr.Ver.Rev == 3 &&
		  info.Hdr.Ver.Build == 7);

	std::vector<uint8_t> vp = Load("v2_prot.bin");
	CHECK(Check(vp, key1, true, &info) == DFU_IMG_OK);
	CHECK(info.Hdr.ProtTlvSize != 0);

	std::vector<uint8_t> vk2 = Load("v2_key2.bin");
	CHECK(Check(vk2, key1, true) == DFU_IMG_ERR_KEY);
	CHECK(Check(vk2, key2, true) == DFU_IMG_OK);

	std::vector<uint8_t> vn = Load("v2_nosig.bin");
	CHECK(Check(vn, key1, true) == DFU_IMG_ERR_SIG);
	CHECK(Check(vn, key1, false) == DFU_IMG_OK);

	std::vector<uint8_t> vt = Load("v1_tamper.bin");
	CHECK(Check(vt, key1, true) == DFU_IMG_ERR_HASH);
	CHECK(Check(vt, key1, false) == DFU_IMG_ERR_HASH);

	std::vector<uint8_t> vb = Load("v1_badsig.bin");
	CHECK(Check(vb, key1, true) == DFU_IMG_ERR_SIG);

	// Key hash absent: the signature alone decides.
	{
		DfuImgInfo_t i2;
		Buf b = { &v1 };
		CHECK(DfuImgParse(BufRead, &b, (uint32_t)v1.size(), &i2) == DFU_IMG_OK);
		i2.bKeyHash = false;
		CHECK(DfuImgVerify(s_Sha, s_Ecc, key1, BufRead, &b, i2) == DFU_IMG_OK);
		CHECK(DfuImgVerify(s_Sha, s_Ecc, key2, BufRead, &b, i2) ==
			  DFU_IMG_ERR_SIG);
	}

	// Vector table.
	std::vector<uint8_t> vv = Load("vec.bin");
	CHECK(Check(vv, key1, true, &info) == DFU_IMG_OK);
	CHECK(DfuImgVectorValid((const uint32_t *)(vv.data() + info.Hdr.HdrSize),
							0x10000000, info.Hdr.ImgSize, 0x20000000UL, 0x40000000UL) == false);
	CHECK(Check(v1, key1, true, &info) == DFU_IMG_OK);
	CHECK(DfuImgVectorValid((const uint32_t *)(v1.data() + info.Hdr.HdrSize),
							0x10000000, info.Hdr.ImgSize, 0x20000000UL, 0x40000000UL));
	CHECK(DfuImgVectorValid((const uint32_t *)(v1.data() + info.Hdr.HdrSize),
							0x20000000, info.Hdr.ImgSize, 0x20000000UL, 0x40000000UL) == false);
	uint32_t vec[2] = { 0x20008001, 0x10000101 };
	CHECK(DfuImgVectorValid(vec, 0x10000000, 0x1000, 0x20000000UL, 0x40000000UL) == false);
	vec[0] = 0x20008000;
	vec[1] = 0x10000100;
	CHECK(DfuImgVectorValid(vec, 0x10000000, 0x1000, 0x20000000UL, 0x40000000UL) == false);

	// Too little room for it, truncated, not an image.
	{
		Buf b = { &v1 };
		CHECK(DfuImgParse(BufRead, &b, (uint32_t)v1.size() - 1, &info) ==
			  DFU_IMG_ERR_SIZE);
		std::vector<uint8_t> tr(v1.begin(), v1.end() - 1);
		CHECK(Check(tr, key1, true) != DFU_IMG_OK);
		std::vector<uint8_t> z(256, 0);
		CHECK(Check(z, key1, true) == DFU_IMG_ERR_MAGIC);
	}

	// Header flags this loader cannot honour.
	{
		std::vector<uint8_t> f = v1;
		f[16] |= 0x04;
		CHECK(Check(f, key1, true) == DFU_IMG_ERR_HDR);
		f = v1;
		f[16] |= 0x20;
		CHECK(Check(f, key1, true) == DFU_IMG_ERR_HDR);
		f = v1;
		f[17] |= 0x04;		// LZMA2
		CHECK(Check(f, key1, true) == DFU_IMG_ERR_HDR);
	}

	// imgtool --pad-sig: the signature TLV is 72 bytes whatever the DER
	// length, zeros after the sequence. Taken; anything but zeros there is not.
	{
		int padded = 0;
		for (int i = 0; i < 8; i++)
		{
			char n[32];
			snprintf(n, sizeof(n), "v1_padsig%d.bin", i);
			std::vector<uint8_t> ps = Load(n);
			DfuImgInfo_t pi;
			CHECK(Check(ps, key1, true, &pi) == DFU_IMG_OK);
			// The signature is the last TLV; 72 bytes and a short sequence
			// means padding is there.
			const uint8_t *sig = ps.data() + ps.size() - 72;
			if (sig[-4] == DFU_IMG_TLV_ECDSA_SIG && sig[1] + 2 < 72)
			{
				padded++;
				std::vector<uint8_t> bad = ps;
				bad[bad.size() - 1] = 0x01;
				CHECK(Check(bad, key1, true) == DFU_IMG_ERR_SIG);
			}
		}
		CHECK(padded > 0);
	}

	// One flipped bit anywhere, or a random byte. What the signature covers,
	// header, payload and protected TLVs, never verifies changed. The
	// unprotected TLV area is not signed, by the format's design: a change
	// there may still verify, for instance a key hash TLV whose type no
	// longer reads as one, which leaves the signature to decide. That is
	// fine only as long as the signed content is the original.
	std::mt19937 rng(11);
	int bad = 0, total = 0, unsigned_ok = 0;
	for (int i = 0; i < 3000; i++)
	{
		const std::vector<uint8_t> &orig = (i & 1) ? v1 : vp;
		std::vector<uint8_t> f = orig;
		size_t pos = rng() % f.size();
		if (i % 3 == 0)
		{
			f[pos] ^= (uint8_t)(1U << (rng() % 8));
		}
		else
		{
			uint8_t nb = (uint8_t)rng();
			if (nb == f[pos])
			{
				nb ^= 0x80;
			}
			f[pos] = nb;
		}
		DfuImgInfo_t fi;
		if (Check(f, key1, true, &fi) != DFU_IMG_OK)
		{
			continue;
		}
		total++;
		const DfuImgHdr_t *h = (const DfuImgHdr_t *)orig.data();
		size_t signedlen = h->HdrSize + h->ImgSize + h->ProtTlvSize;
		if (pos < signedlen)
		{
			bad++;
			fprintf(stderr, "verified with signed byte %zu changed\n", pos);
		}
		else
		{
			unsigned_ok++;
		}
	}
	CHECK(bad == 0);
	CHECK(total == unsigned_ok);
}

int main(int argc, char **argv)
{
	s_Dir = argc > 1 ? argv[1] : "build/img";

	s_Sha = CryptoSoftSha256Create(s_ShaMem, sizeof(s_ShaMem));
	s_Ecc = CryptoUeccCreate(s_EccMem, sizeof(s_EccMem), nullptr);
	CHECK(s_Sha != nullptr && s_Ecc != nullptr);

	TestCborRead();
	TestCborWrite();
	TestImages();

	printf("dfu_image_test: %d passed, %d failed\n", s_Pass, s_Fail);

	return s_Fail != 0;
}
