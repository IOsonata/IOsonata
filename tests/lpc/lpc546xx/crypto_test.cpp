// Host regression of the LPC546xx crypto provider (ROM RNG and the LPC54628
// SHA block) with a register model, then the shared crypto engine test on it.
//
// Model: the SHA registers need the SHA0 bus clock (AHBCLKCTRL2) and SHA out
// of reset (PRESETCTRL2), a reset clears the block. CTRL with NEW starts a
// hash, MODE 2 (SHA-256) and DMA 0, not inside a block. STATUS WAITING is set
// when the block takes the next 16 words. The words go to INDATA or ALIAS in
// any mix, the first word of a block only while WAITING is set, each word is
// 4 message bytes loaded little endian. The 16th word starts the compression:
// WAITING and DIGEST clear, both set when it ends 400 ns later. DIGEST[i]
// reads hash word i, only with DIGEST set. MEMCTRL (erratum SHA.1) and the
// interrupts are not used. Faults: no block (reads 0, writes ignored), WAITING
// or DIGEST never set again, ERROR set, a wrong digest bit, a block slower
// than the poll limit.
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <cstring>

#include "LPC546xx.h"
#include "crypto/crypto.h"
#include "crypto/crypto_softsha256.h"

uint32_t g_IrqMasked = 0;

static uint64_t s_Ns = 0;

static const uint32_t s_K[64] = {
	0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
	0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
	0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
	0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
	0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
	0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
	0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
	0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};
static const uint32_t s_Iv[8] = {
	0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
};

static uint32_t Ror(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

static void Compress(uint32_t H[8], const uint32_t In[16])
{
	uint32_t w[64], v[8];

	for (int i = 0; i < 16; i++) w[i] = In[i];
	for (int i = 16; i < 64; i++)
	{
		uint32_t s0 = Ror(w[i - 15], 7) ^ Ror(w[i - 15], 18) ^ (w[i - 15] >> 3);
		uint32_t s1 = Ror(w[i - 2], 17) ^ Ror(w[i - 2], 19) ^ (w[i - 2] >> 10);
		w[i] = w[i - 16] + s0 + w[i - 7] + s1;
	}
	memcpy(v, H, sizeof(v));
	for (int i = 0; i < 64; i++)
	{
		uint32_t t1 = v[7] + (Ror(v[4], 6) ^ Ror(v[4], 11) ^ Ror(v[4], 25)) +
					  ((v[4] & v[5]) ^ (~v[4] & v[6])) + s_K[i] + w[i];
		uint32_t t2 = (Ror(v[0], 2) ^ Ror(v[0], 13) ^ Ror(v[0], 22)) +
					  ((v[0] & v[1]) ^ (v[0] & v[2]) ^ (v[1] & v[2]));
		memmove(v + 1, v, 7 * sizeof(uint32_t));
		v[4] += t1;
		v[0] = t1 + t2;
	}
	for (int i = 0; i < 8; i++) H[i] += v[i];
}

struct ShaM {
	bool Started = false, Busy = false, Waiting = false, Digest = false, Error = false;
	uint32_t H[8] = {}, W[16] = {};
	int N = 0;
	uint64_t End = 0;
	int Blocks = 0, Fed = 0, News = 0, Resets = 0;	// Blocks done, blocks written
	// Faults
	bool Absent = false, WrongDigest = false, ErrorNext = false, DigestStuck = false;
	bool SlowNext = false;		// The next block takes 1000 ns
	int StuckAt = -1;			// WAITING never set again after this block count
};
static ShaM s_S;

static uint32_t s_Ahb[3], s_Rst[3];

static void ShaClear()
{
	ShaM keep = s_S;

	s_S = ShaM();
	s_S.Blocks = keep.Blocks;
	s_S.Fed = keep.Fed;
	s_S.News = keep.News;
	s_S.Resets = keep.Resets + 1;
	s_S.Absent = keep.Absent;
	s_S.WrongDigest = keep.WrongDigest;
	s_S.ErrorNext = keep.ErrorNext;
	s_S.DigestStuck = keep.DigestStuck;
	s_S.SlowNext = keep.SlowNext;
	s_S.StuckAt = keep.StuckAt;
}

static void Process()
{
	if (s_S.Busy && s_Ns >= s_S.End)
	{
		Compress(s_S.H, s_S.W);
		memset(s_S.W, 0, sizeof(s_S.W));
		s_S.Busy = false;
		s_S.Blocks++;
		if (s_S.ErrorNext)
		{
			s_S.Error = true;
			s_S.ErrorNext = false;
		}
		s_S.Waiting = s_S.StuckAt < 0 || s_S.Blocks < s_S.StuckAt;
		s_S.Digest = !s_S.DigestStuck;
	}
}

static void Access()
{
	assert(s_Ahb[2] & SYSCON_AHBCLKCTRL_SHA0_MASK);
	assert((s_Rst[2] & SYSCON_PRESETCTRL_SHA_RST_MASK) == 0);
	s_Ns += 10;
	Process();
}

enum { AHB_SET, AHB_CLR, RST_SET, RST_CLR };

static void SysWrite(int Kind, int Idx, uint32_t v)
{
	// The provider only touches the SHA0 clock and reset
	assert(Idx == 2 && v == (Kind < RST_SET ? SYSCON_AHBCLKCTRL_SHA0_MASK : SYSCON_PRESETCTRL_SHA_RST_MASK));
	switch (Kind)
	{
		case AHB_SET: s_Ahb[Idx] |= v; break;
		case AHB_CLR: s_Ahb[Idx] &= ~v; break;
		case RST_SET: s_Rst[Idx] |= v; ShaClear(); break;
		case RST_CLR: s_Rst[Idx] &= ~v; break;
	}
}

struct WReg {
	int Kind, Idx;
	WReg &operator=(uint32_t v) { SysWrite(Kind, Idx, v); return *this; }
};
struct SysRegs {
	WReg AHBCLKCTRLSET[3] = { { AHB_SET, 0 }, { AHB_SET, 1 }, { AHB_SET, 2 } };
	WReg AHBCLKCTRLCLR[3] = { { AHB_CLR, 0 }, { AHB_CLR, 1 }, { AHB_CLR, 2 } };
	WReg PRESETCTRLSET[3] = { { RST_SET, 0 }, { RST_SET, 1 }, { RST_SET, 2 } };
	WReg PRESETCTRLCLR[3] = { { RST_CLR, 0 }, { RST_CLR, 1 }, { RST_CLR, 2 } };
};
static SysRegs s_SysRegs;

static uint32_t Bswap(uint32_t v)
{
	return (v >> 24) | ((v >> 8) & 0xFF00U) | ((v << 8) & 0xFF0000U) | (v << 24);
}

static void WordWrite(uint32_t v)
{
	Access();
	if (s_S.Absent)
	{
		return;
	}
	assert(s_S.Started && !s_S.Busy);
	if (s_S.N == 0)
	{
		assert(s_S.Waiting);
	}
	s_S.W[s_S.N++] = Bswap(v);
	if (s_S.N == 16)
	{
		s_S.N = 0;
		s_S.Fed++;
		s_S.Busy = true;
		s_S.Waiting = false;
		s_S.Digest = false;
		s_S.End = s_Ns + (s_S.SlowNext ? 1000 : 400);
		s_S.SlowNext = false;
	}
}

struct CtrlR {
	CtrlR &operator=(uint32_t v)
	{
		Access();
		if (s_S.Absent)
		{
			return *this;
		}
		assert((v & ~(SHA_CTRL_MODE_MASK | SHA_CTRL_NEW_MASK)) == 0);
		assert((v & SHA_CTRL_MODE_MASK) == SHA_CTRL_MODE(2) && (v & SHA_CTRL_NEW_MASK));
		assert(!s_S.Busy && s_S.N == 0);
		memcpy(s_S.H, s_Iv, sizeof(s_Iv));
		s_S.Started = true;
		s_S.Waiting = true;
		s_S.Digest = false;
		s_S.News++;
		return *this;
	}
};
struct StatusR {
	operator uint32_t() const
	{
		Access();
		if (s_S.Absent)
		{
			return 0;
		}
		return (s_S.Waiting ? SHA_STATUS_WAITING_MASK : 0U) | (s_S.Digest ? SHA_STATUS_DIGEST_MASK : 0U) |
			   (s_S.Error ? SHA_STATUS_ERROR_MASK : 0U);
	}
};
struct InR {
	InR &operator=(uint32_t v) { WordWrite(v); return *this; }
};
struct DigestR {
	int Idx;
	operator uint32_t() const
	{
		Access();
		if (s_S.Absent)
		{
			return 0;
		}
		assert(s_S.Digest && !s_S.Busy);
		return s_S.H[Idx] ^ (s_S.WrongDigest && Idx == 7 ? 1U : 0U);
	}
};
struct ShaRegs {
	CtrlR CTRL;
	StatusR STATUS;
	InR INDATA;
	InR ALIAS[7];
	DigestR DIGEST[8] = { { 0 }, { 1 }, { 2 }, { 3 }, { 4 }, { 5 }, { 6 }, { 7 } };
};
static ShaRegs s_ShaRegs;

// ROM RNG: a counter generator, each read counted
static uint32_t s_RngNext = 0x12345678U;
static int s_RngReads = 0;
static uint32_t RngRead(void)
{
	s_RngReads++;
	s_RngNext = s_RngNext * 1664525U + 1013904223U;
	return s_RngNext;
}
// The ROM tables have read-only members, they live in plain memory here
alignas(8) static uint8_t s_OtpMem[sizeof(OTP_API_Type)];
alignas(8) static uint8_t s_RomMem[sizeof(ROM_API_Type)];
static OTP_API_Type &s_Otp = *(OTP_API_Type *)s_OtpMem;
static ROM_API_Type *s_RomPtr = (ROM_API_Type *)s_RomMem;

#undef SYSCON
#undef SHA0
#undef ROM_API
#define SYSCON (&s_SysRegs)
#define SHA0 (&s_ShaRegs)
#define ROM_API (s_RomPtr)
#include "../../../ARM/NXP/LPC546xx/src/crypto_default_lpc546xx.cpp"

extern int CryptoEngineTest();

static CryptoSoftSha256 s_Ref;

struct Ctx {
	alignas(CRYPTO_HASHCTX_ALIGN_MAX) uint8_t B[CRYPTO_HASHCTX_MAX + 16];
};

static void Fill(uint8_t *p, size_t Len, uint32_t Seed)
{
	for (size_t i = 0; i < Len; i++)
	{
		Seed = Seed * 1103515245U + 12345U;
		p[i] = (uint8_t)(Seed >> 16);
	}
}

static void RefHash(const uint8_t *p, size_t Len, uint8_t *pOut)
{
	assert(s_Ref.Hash(CRYPTO_HASH_SHA256, p, Len, pOut) == CRYPTO_STATUS_OK);
}

static int Blocks(size_t Len)
{
	return (int)((Len + 9 + 63) / 64);
}

// Back to a block not tested yet, and a Crypto object initialized on it
static void Fresh(Crypto &C, bool Absent = false)
{
	s_S = ShaM();
	s_S.Absent = Absent;
	s_Ahb[2] = 0;
	s_Rst[2] = 0;
	s_ShaState = LPC546XX_SHA_UNTESTED;
	s_ShaOwner = nullptr;
	s_Busy = false;
	assert(C.Init());
	assert(g_IrqMasked == 0);
}

static bool Idle()
{
	static const uint32_t zero[8] = {};

	return s_ShaOwner == nullptr && !s_S.Started && !s_S.Busy &&
		   memcmp(s_S.H, zero, sizeof(s_S.H)) == 0 && g_IrqMasked == 0;
}

static void TestInit()
{
	{
		Crypto c;

		Fresh(c);
		assert(s_ShaState == LPC546XX_SHA_READY);
		assert(s_Ahb[2] & SYSCON_AHBCLKCTRL_SHA0_MASK);
		// Two known answers, 2 blocks each, on the block
		assert(s_S.News == 2 && s_S.Blocks == 4);
		assert(Idle());
		assert(c.IsSecure());
		assert(c.HashCtxSize() == 120U && c.HashCtxSize() <= CRYPTO_HASHCTX_MAX);
		assert(c.HashCtxAlign() == 8U && CRYPTO_HASHCTX_ALIGN_MAX % c.HashCtxAlign() == 0);

		// Init again does not run the test again
		c.Disable();
		assert(c.Init());
		assert(s_S.News == 2);
	}
	{
		// No block: software hashes
		Crypto c;
		uint8_t msg[130], d[32], e[32];

		Fresh(c, true);
		assert(s_ShaState == LPC546XX_SHA_OFF);
		assert((s_Ahb[2] & SYSCON_AHBCLKCTRL_SHA0_MASK) == 0);
		assert(s_ShaOwner == nullptr);
		Fill(msg, sizeof(msg), 1);
		for (size_t l = 0; l <= sizeof(msg); l++)
		{
			assert(c.Hash(CRYPTO_HASH_SHA256, msg, l, d) == CRYPTO_STATUS_OK);
			RefHash(msg, l, e);
			assert(memcmp(d, e, 32) == 0);
		}
		assert(s_S.Blocks == 0);
		assert(c.SelfTest() == 0);
	}
	{
		// A digest bit wrong: the block is not used
		Crypto c;

		s_S = ShaM();
		s_S.WrongDigest = true;
		s_Ahb[2] = 0;
		s_ShaState = LPC546XX_SHA_UNTESTED;
		assert(c.Init());
		assert(s_ShaState == LPC546XX_SHA_OFF && (s_Ahb[2] & SYSCON_AHBCLKCTRL_SHA0_MASK) == 0);
		assert(s_S.Blocks == 2 && s_S.News == 1);
	}
	{
		// The second known answer fails
		Crypto c;

		s_S = ShaM();
		s_S.StuckAt = 3;
		s_Ahb[2] = 0;
		s_ShaState = LPC546XX_SHA_UNTESTED;
		assert(c.Init());
		assert(s_ShaState == LPC546XX_SHA_OFF && s_S.Blocks == 3);
	}
	{
		// The known answers have to come from the block
		Crypto c;
		int other;

		s_S = ShaM();
		s_Ahb[2] = 0;
		s_ShaState = LPC546XX_SHA_UNTESTED;
		s_ShaOwner = &other;
		assert(c.Init());
		assert(s_ShaState == LPC546XX_SHA_OFF && s_S.News == 0 && s_ShaOwner == nullptr);
	}
	{
		// No RNG: Init fails, the block is not tested
		Crypto c;
		s_Otp.rngRead = nullptr;
		s_S = ShaM();
		s_ShaState = LPC546XX_SHA_UNTESTED;
		assert(!c.Init());
		assert(s_S.News == 0 && s_ShaState == LPC546XX_SHA_UNTESTED);
		uint8_t d[32];
		memset(d, 0xA5, sizeof(d));
		assert(c.Hash(CRYPTO_HASH_SHA256, (const uint8_t *)"abc", 3, d) == CRYPTO_STATUS_FAIL);
		for (int i = 0; i < 32; i++) assert(d[i] == 0);
		s_Otp.rngRead = RngRead;
	}
}

static void TestOneShot()
{
	Crypto c;
	uint8_t msg[260], out[34], e[32];

	Fresh(c);
	Fill(msg, sizeof(msg), 7);
	for (size_t l = 0; l <= 257; l++)
	{
		int blocks = s_S.Blocks;

		memset(out, 0xA5, sizeof(out));
		assert(c.Hash(CRYPTO_HASH_SHA256, msg + 1, l, out + 1) == CRYPTO_STATUS_OK);
		RefHash(msg + 1, l, e);
		assert(memcmp(out + 1, e, 32) == 0 && out[0] == 0xA5 && out[33] == 0xA5);
		assert(s_S.Blocks - blocks == Blocks(l));
		assert(Idle());
	}
	assert(c.Hash(CRYPTO_HASH_SHA256, nullptr, 0, out) == CRYPTO_STATUS_OK);
	RefHash(nullptr, 0, e);
	assert(memcmp(out, e, 32) == 0);
	assert(c.Hash(CRYPTO_HASH_SHA256, nullptr, 1, out) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.Hash(CRYPTO_HASH_SHA256, msg, 1, nullptr) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.Hash((CRYPTO_HASH_ALG)(CRYPTO_HASH_SHA256 + 1), msg, 1, out) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.SelfTest() == 0);
	assert(Idle());
}

// Streams in random pieces, one on the block and one in software, with
// one-shot hashes between the pieces
static void TestStreams()
{
	Crypto c;
	static uint8_t ma[1000], mb[777];
	uint8_t d[32], e[32], x[32];
	Ctx a, b, k;
	size_t oa = 0, ob = 0;
	uint32_t seed = 99;

	Fresh(c);
	Fill(ma, sizeof(ma), 3);
	Fill(mb, sizeof(mb), 4);
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK);
	assert(ShaHwCtx(a.B) && s_ShaOwner == a.B);
	assert(c.HashInit(CRYPTO_HASH_SHA256, b.B) == CRYPTO_STATUS_OK);
	assert(!ShaHwCtx(b.B) && s_ShaOwner == a.B);
	int fed = s_S.Fed;
	while (oa < sizeof(ma) || ob < sizeof(mb))
	{
		seed = seed * 1103515245U + 12345U;
		size_t n = (seed >> 16) % 150;
		if (oa < sizeof(ma))
		{
			size_t l = n < sizeof(ma) - oa ? n : sizeof(ma) - oa;
			assert(c.HashUpdate(a.B, ma + oa, l) == CRYPTO_STATUS_OK);
			oa += l;
		}
		int before = s_S.Fed;
		if (ob < sizeof(mb))
		{
			size_t l = (n * 7 + 3) % 90;
			l = l < sizeof(mb) - ob ? l : sizeof(mb) - ob;
			assert(c.HashUpdate(b.B, mb + ob, l) == CRYPTO_STATUS_OK);
			ob += l;
		}
		assert(c.Hash(CRYPTO_HASH_SHA256, mb, n, x) == CRYPTO_STATUS_OK);
		RefHash(mb, n, e);
		assert(memcmp(x, e, 32) == 0);
		// Neither the software stream nor the one-shot hash used the block
		assert(s_S.Fed == before);
	}
	assert(s_S.Fed - fed == (int)(sizeof(ma) / 64));
	assert(c.HashFinal(b.B, d) == CRYPTO_STATUS_OK);
	RefHash(mb, sizeof(mb), e);
	assert(memcmp(d, e, 32) == 0);
	assert(s_ShaOwner == a.B);
	assert(c.HashFinal(a.B, d) == CRYPTO_STATUS_OK);
	RefHash(ma, sizeof(ma), e);
	assert(memcmp(d, e, 32) == 0);
	assert(Idle());
	// Both finalized: dead
	assert(c.HashUpdate(a.B, ma, 1) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.HashFinal(b.B, d) == CRYPTO_STATUS_UNSUPPORTED);

	// A piece that ends on a block boundary
	assert(c.HashInit(CRYPTO_HASH_SHA256, k.B) == CRYPTO_STATUS_OK && s_ShaOwner == k.B);
	assert(c.HashUpdate(k.B, ma, 10) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, ma + 10, 54) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, ma + 64, 64) == CRYPTO_STATUS_OK);
	assert(c.HashFinal(k.B, d) == CRYPTO_STATUS_OK);
	RefHash(ma, 128, e);
	assert(memcmp(d, e, 32) == 0);
	assert(c.HashInit(CRYPTO_HASH_SHA256, k.B) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, ma, 10) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, ma + 10, 54) == CRYPTO_STATUS_OK);
	assert(c.HashFinal(k.B, d) == CRYPTO_STATUS_OK);
	RefHash(ma, 64, e);
	assert(memcmp(d, e, 32) == 0);

	// The block is free again
	assert(c.HashInit(CRYPTO_HASH_SHA256, k.B) == CRYPTO_STATUS_OK && s_ShaOwner == k.B);
	assert(c.HashUpdate(k.B, ma, 64) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, ma + 64, 0) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, nullptr, 0) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, ma + 64, 5) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(k.B, nullptr, 0) == CRYPTO_STATUS_OK);
	assert(c.HashFinal(k.B, d) == CRYPTO_STATUS_OK);
	RefHash(ma, 69, e);
	assert(memcmp(d, e, 32) == 0);
	assert(Idle());
}

static void TestOwner()
{
	Crypto c;
	static uint8_t m[300];
	uint8_t d[32], e[32];
	Ctx a, copy, b;

	Fresh(c);
	Fill(m, sizeof(m), 11);

	// HashInit on the storage of the owner restarts it
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(a.B, m, 100) == CRYPTO_STATUS_OK);
	int news = s_S.News, resets = s_S.Resets;
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK);
	assert(s_S.News == news + 1 && s_S.Resets == resets + 1 && s_ShaOwner == a.B);
	assert(c.HashUpdate(a.B, m + 5, 200) == CRYPTO_STATUS_OK);

	// A copy of the owner is refused and cleared, the owner goes on
	memcpy(copy.B, a.B, sizeof(a.B));
	assert(c.HashUpdate(copy.B, m, 10) == CRYPTO_STATUS_FAIL);
	assert(!ShaHwCtx(copy.B) && ((Lpc546xxShaCtx *)copy.B)->Magic == 0);
	memcpy(copy.B, a.B, sizeof(a.B));
	memset(d, 0xA5, sizeof(d));
	assert(c.HashFinal(copy.B, d) == CRYPTO_STATUS_FAIL);
	for (int i = 0; i < 32; i++) assert(d[i] == 0);
	assert(((Lpc546xxShaCtx *)copy.B)->Magic == 0);
	assert(s_ShaOwner == a.B);

	// A copy at an address the stream storage cannot have
	alignas(16) uint8_t odd[CRYPTO_HASHCTX_MAX + 8];
	memcpy(odd + 4, a.B, sizeof(Lpc546xxShaCtx));
	assert(c.HashUpdate(odd + 4, m, 10) == CRYPTO_STATUS_UNSUPPORTED);
	assert(s_ShaOwner == a.B);

	// Argument errors leave the stream as it was
	assert(c.HashUpdate(a.B, nullptr, 3) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.HashFinal(a.B, nullptr) == CRYPTO_STATUS_UNSUPPORTED);
	assert(s_ShaOwner == a.B && ShaHwCtx(a.B));
	assert(c.HashFinal(a.B, d) == CRYPTO_STATUS_OK);
	RefHash(m + 5, 200, e);
	assert(memcmp(d, e, 32) == 0);
	assert(Idle());

	// A dropped owner keeps the block, others use software
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(a.B, m, 130) == CRYPTO_STATUS_OK);
	memset(a.B, 0, sizeof(a.B));
	assert(c.HashInit(CRYPTO_HASH_SHA256, b.B) == CRYPTO_STATUS_OK && !ShaHwCtx(b.B));
	assert(c.HashUpdate(b.B, m, 70) == CRYPTO_STATUS_OK);
	assert(c.HashFinal(b.B, d) == CRYPTO_STATUS_OK);
	RefHash(m, 70, e);
	assert(memcmp(d, e, 32) == 0);
	// Until HashInit on the same storage
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK && ShaHwCtx(a.B));
	assert(c.HashFinal(a.B, d) == CRYPTO_STATUS_OK);
	RefHash(m, 0, e);
	assert(memcmp(d, e, 32) == 0);
	assert(Idle());

	// Length limit
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK);
	((Lpc546xxShaCtx *)a.B)->Total = UINT64_MAX / 8U - 3U;
	assert(c.HashUpdate(a.B, m, 3) == CRYPTO_STATUS_OK);
	assert(c.HashUpdate(a.B, m, 1) == CRYPTO_STATUS_FAIL);
	assert(((Lpc546xxShaCtx *)a.B)->Magic == 0 && Idle());

	// Arguments of HashInit
	assert(c.HashInit((CRYPTO_HASH_ALG)(CRYPTO_HASH_SHA256 + 1), a.B) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.HashInit(CRYPTO_HASH_SHA256, nullptr) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B + 4) == CRYPTO_STATUS_UNSUPPORTED);
	assert(Idle());
	// Storage never initialized
	memset(b.B, 0x5A, sizeof(b.B));
	assert(c.HashUpdate(b.B, m, 1) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.HashFinal(b.B, d) == CRYPTO_STATUS_UNSUPPORTED);
	assert(c.HashUpdate(nullptr, m, 1) == CRYPTO_STATUS_UNSUPPORTED);
	// The magic of a stream on the block with a byte count out of range
	assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK);
	((Lpc546xxShaCtx *)a.B)->Used = 64;
	assert(c.HashUpdate(a.B, m, 1) == CRYPTO_STATUS_UNSUPPORTED);
	((Lpc546xxShaCtx *)a.B)->Used = 0;
	assert(c.HashFinal(a.B, d) == CRYPTO_STATUS_OK);
	assert(Idle());

	// Not initialized
	Crypto n;
	memset(d, 0xA5, sizeof(d));
	assert(n.Hash(CRYPTO_HASH_SHA256, m, 3, d) == CRYPTO_STATUS_FAIL);
	for (int i = 0; i < 32; i++) assert(d[i] == 0);
	assert(n.HashCtxSize() == 0 && n.HashCtxAlign() == 1);
	assert(n.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_FAIL);
	assert(n.HashUpdate(a.B, m, 1) == CRYPTO_STATUS_FAIL);
	memset(d, 0xA5, sizeof(d));
	assert(n.HashFinal(a.B, d) == CRYPTO_STATUS_FAIL);
	for (int i = 0; i < 32; i++) assert(d[i] == 0);
	assert(n.HashFinal(a.B, nullptr) == CRYPTO_STATUS_FAIL);
	assert(n.Hash(CRYPTO_HASH_SHA256, m, 3, nullptr) == CRYPTO_STATUS_FAIL);
	assert(Idle());
}

// A failed block: the stream fails, the block is off, software from then on
static void TestFail()
{
	static uint8_t m[400];
	uint8_t d[32], e[32];
	Fill(m, sizeof(m), 21);

	for (int kind = 0; kind < 6; kind++)
	{
		Crypto c;
		Ctx a;

		Fresh(c);
		int base = s_S.Blocks;
		assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK);
		CRYPTO_STATUS st;
		memset(d, 0xA5, sizeof(d));
		switch (kind)
		{
			case 0:		// WAITING not set again, during an update from the buffer
				s_S.StuckAt = base + 1;
				assert(c.HashUpdate(a.B, m, 70) == CRYPTO_STATUS_OK);
				st = c.HashUpdate(a.B, m, 60);
				break;
			case 1:		// During an update from the message
				s_S.StuckAt = base + 1;
				st = c.HashUpdate(a.B, m, 200);
				break;
			case 2:		// ERROR
				s_S.ErrorNext = true;
				assert(c.HashUpdate(a.B, m, 64) == CRYPTO_STATUS_OK);
				st = c.HashFinal(a.B, d);
				break;
			case 3:		// DIGEST not set
				s_S.DigestStuck = true;
				st = c.HashFinal(a.B, d);
				break;
			case 4:		// WAITING not set for the length block
				s_S.StuckAt = base + 1;
				assert(c.HashUpdate(a.B, m, 60) == CRYPTO_STATUS_OK);
				st = c.HashFinal(a.B, d);
				break;
			default:	// A block slower than the poll limit, before the
						// padding block
				s_S.SlowNext = true;
				assert(c.HashUpdate(a.B, m, 124) == CRYPTO_STATUS_OK);
				st = c.HashFinal(a.B, d);
				break;
		}
		assert(st == CRYPTO_STATUS_FAIL);
		if (kind >= 2)
		{
			for (int i = 0; i < 32; i++) assert(d[i] == 0);
		}
		assert(((Lpc546xxShaCtx *)a.B)->Magic == 0);
		assert(s_ShaState == LPC546XX_SHA_OFF && (s_Ahb[2] & SYSCON_AHBCLKCTRL_SHA0_MASK) == 0);
		assert(Idle());
		assert(c.HashUpdate(a.B, m, 1) == CRYPTO_STATUS_UNSUPPORTED);
		// Software from now on
		assert(c.HashInit(CRYPTO_HASH_SHA256, a.B) == CRYPTO_STATUS_OK && !ShaHwCtx(a.B));
		assert(c.HashUpdate(a.B, m, 300) == CRYPTO_STATUS_OK);
		assert(c.HashFinal(a.B, d) == CRYPTO_STATUS_OK);
		RefHash(m, 300, e);
		assert(memcmp(d, e, 32) == 0);
		assert(c.Hash(CRYPTO_HASH_SHA256, m, 99, d) == CRYPTO_STATUS_OK);
		RefHash(m, 99, e);
		assert(memcmp(d, e, 32) == 0);
	}
	{
		// One-shot hash on a failing block
		Crypto c;

		Fresh(c);
		s_S.StuckAt = s_S.Blocks + 1;
		memset(d, 0xA5, sizeof(d));
		assert(c.Hash(CRYPTO_HASH_SHA256, m, 200, d) == CRYPTO_STATUS_FAIL);
		for (int i = 0; i < 32; i++) assert(d[i] == 0);
		assert(s_ShaState == LPC546XX_SHA_OFF && s_ShaOwner == nullptr);
	}
}

static void TestMac()
{
	Crypto c;
	uint8_t key[131], out[34], e[32];
	static uint8_t m[300];
	CryptoKey k{CRYPTO_KEY_HMAC, CRYPTO_KEY_LOC_PLAIN, CRYPTO_KEY_USE_SIGN, {}};

	Fresh(c);
	Fill(key, sizeof(key), 5);
	Fill(m, sizeof(m), 6);
	static const size_t keyLens[] = { 0, 20, 64, 65, 131 };
	static const size_t msgLens[] = { 0, 1, 55, 56, 64, 300 };

	for (size_t kl : keyLens)
	{
		for (size_t l : msgLens)
		{
			k.Plain.pData = kl ? key : nullptr;
			k.Plain.Len = kl;
			int blocks = s_S.Blocks;
			memset(out, 0xA5, sizeof(out));
			assert(c.Mac(CRYPTO_MAC_HMAC, k, l ? m : nullptr, l, out + 1, 32) == CRYPTO_STATUS_OK);
			assert(s_Ref.Mac(CRYPTO_MAC_HMAC, k, l ? m : nullptr, l, e, 32) == CRYPTO_STATUS_OK);
			assert(memcmp(out + 1, e, 32) == 0 && out[0] == 0xA5 && out[33] == 0xA5);
			// Inner and outer digests on the block
			assert(s_S.Blocks - blocks >= Blocks(64 + l) + Blocks(96));
			assert(Idle());
		}
	}
}

static void TestRng()
{
	Crypto c;
	uint8_t b[10];
	uint32_t next = s_RngNext;

	Fresh(c);
	next = s_RngNext;
	int reads = s_RngReads;
	assert(c.Random(b, sizeof(b)) == CRYPTO_STATUS_OK);
	assert(s_RngReads - reads == 3 * 33);
	for (int w = 0; w < 3; w++)
	{
		next = next * 1664525U + 1013904223U;
		uint32_t v = next;
		for (int i = 0; i < 4 && 4 * w + i < 10; i++) assert(b[4 * w + i] == (uint8_t)(v >> (8 * i)));
		for (int i = 0; i < 32; i++) next = next * 1664525U + 1013904223U;
	}
	assert(c.Random(b, 0) == CRYPTO_STATUS_OK);
	assert(c.Random(nullptr, 4) == CRYPTO_STATUS_FAIL);
	s_Busy = true;
	memset(b, 0xA5, sizeof(b));
	assert(c.Random(b, 4) == CRYPTO_STATUS_BUSY);
	for (int i = 0; i < 4; i++) assert(b[i] == 0);
	s_Busy = false;
	Crypto n;
	memset(b, 0xA5, sizeof(b));
	assert(n.Random(b, 4) == CRYPTO_STATUS_FAIL);
	for (int i = 0; i < 4; i++) assert(b[i] == 0);
	assert(!n.IsSecure() && c.IsSecure());
	assert(g_IrqMasked == 0);
}

int main()
{
	s_RomPtr->otpApiBase = &s_Otp;
	s_Otp.rngRead = RngRead;
	s_Ref.Enable();

	TestInit();
	TestOneShot();
	TestStreams();
	TestOwner();
	TestFail();
	TestMac();
	TestRng();

	// The shared engine test on the block
	Crypto c;
	Fresh(c);
	c.Disable();
	int blocks = s_S.Blocks;
	assert(CryptoEngineTest() == 0);
	assert(s_ShaState == LPC546XX_SHA_READY && s_S.Blocks > blocks + 1000);
	printf("LPC546xx crypto provider: pass\n");

	return 0;
}
