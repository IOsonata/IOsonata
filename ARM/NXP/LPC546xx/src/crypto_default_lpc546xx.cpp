/**-------------------------------------------------------------------------
@file	crypto_default_lpc546xx.cpp
@brief	LPC546xx ROM RNG and LPC54628 SHA-256 block, with inherited software
		crypto.

The LPC54628 has a SHA block (SHA-1 and SHA-256, see erratum SHA.1 in the
LPC546xx errata sheet). The processor writes each 64-byte block to INDATA
and the ALIAS registers; the AHB master mode (MEMCTRL) is not used, so
erratum SHA.1 does not apply. Padding is done in software.

The running hash stays in the block and cannot be loaded back, so one stream
at a time owns the block, from HashInit to HashFinal. A stream started while
another owns the block, and Hash called from an interrupt during a stream,
use the software SHA-256 in the same caller storage.

A known-answer test at Crypto::Init decides whether the block is used. When
it fails, or a block later times out or reports an error, the block is reset,
its clock is turned off (SYSCON AHBCLKCTRL2 SHA0) and all hashing is done in
software from then on. The SHA0 clock stays on while the block is in use.

A stream that owns the block cannot be copied, the copy is refused. A stream
dropped without HashFinal keeps the block until HashInit is called again on
the same storage, other streams use software meanwhile.

The other LPC546xx parts have no SHA block and use the software SHA-256.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#include "crypto/crypto.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "coredev/interrupt.h"
#include "LPC546xx.h"

#if defined(FSL_FEATURE_SOC_SHA_COUNT) && (FSL_FEATURE_SOC_SHA_COUNT > 0)
#define LPC546XX_SHA_HW			1
#else
#define LPC546XX_SHA_HW			0
#endif

#ifndef LPC546XX_SHA_POLL_MAX
// STATUS reads before a block is declared failed. A block takes about 70 bus
// clocks.
#define LPC546XX_SHA_POLL_MAX	10000U
#endif

static bool s_Busy = false;

class CryptoLpc546xx : public RngEngine
{
public:
	void Disable() override
	{
		vbValid = false;
	}
	void Reset() override
	{
		(void)Enable();
	}
	bool Enable() override
	{
		vbValid = ROM_API != nullptr && OTP_API != nullptr && OTP_API->rngRead != nullptr;
		return vbValid;
	}

	bool IsSecure() const override
	{
		return vbValid;
	}

	CRYPTO_STATUS Random(uint8_t *pOut, size_t Len) override
	{
		uintptr_t state;
		size_t offset = 0U;
		uint32_t word = 0U;
		size_t count;
		bool ok = true;
		unsigned i;

		if (Len == 0U)
		{
			return CRYPTO_STATUS_OK;
		}
		if (pOut == nullptr)
		{
			return CRYPTO_STATUS_FAIL;
		}
		state = DisableInterrupt();
		if (s_Busy)
		{
			EnableInterrupt(state);
			CryptoSecureWipe(pOut, Len);
			return CRYPTO_STATUS_BUSY;
		}
		s_Busy = true;
		EnableInterrupt(state);
		if (!vbValid)
		{
			ok = false;
		}
		else
		{
			while (offset < Len)
			{
				word = OTP_API->rngRead();
				count = Len - offset < sizeof(word) ? Len - offset : sizeof(word);
				memcpy(pOut + offset, &word, count);
				offset += count;
				// UM10912: discard the next 32 words between returned words.
				for (i = 0U; i < 32U; i++)
				{
					(void)OTP_API->rngRead();
				}
			}
			CryptoSecureWipe(&word, sizeof(word));
		}
		state = DisableInterrupt();
		s_Busy = false;
		EnableInterrupt(state);
		if (!ok)
		{
			CryptoSecureWipe(pOut, Len);
			return CRYPTO_STATUS_FAIL;
		}
		return CRYPTO_STATUS_OK;
	}
};

static CryptoLpc546xx s_Hardware;

#if LPC546XX_SHA_HW

#define LPC546XX_SHA_MAGIC		0x4C534832UL

// Storage of a stream that owns the SHA block. A stream in software has the
// software SHA-256 context in the same storage, the magic tells them apart.
struct Lpc546xxShaCtx
{
	uint32_t Magic;
	uint32_t Used;				// Message bytes in Buf
	uint64_t Total;				// Message bytes so far
	uint8_t Buf[64];
};
static_assert(sizeof(Lpc546xxShaCtx) <= CRYPTO_HASHCTX_MAX, "SHA storage too small");

enum {
	LPC546XX_SHA_UNTESTED,		// Known-answer test not run yet
	LPC546XX_SHA_READY,			// Block in use
	LPC546XX_SHA_OFF,			// Block absent or failed, software only
};

static int s_ShaState = LPC546XX_SHA_UNTESTED;
static const void *s_ShaOwner = nullptr;
static CryptoSoftSha256Hash s_ShaSoft;

// FIPS 180-4 examples: two blocks with the length alone in the second, and
// a block written from the message followed by one with the padding.
static const char s_ShaKatMsg1[] =
	"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
static const uint8_t s_ShaKatDigest1[32] = {
	0x24, 0x8D, 0x6A, 0x61, 0xD2, 0x06, 0x38, 0xB8, 0xE5, 0xC0, 0x26, 0x93,
	0x0C, 0x3E, 0x60, 0x39, 0xA3, 0x3C, 0xE4, 0x59, 0x64, 0xFF, 0x21, 0x67,
	0xF6, 0xEC, 0xED, 0xD4, 0x19, 0xDB, 0x06, 0xC1
};
static const char s_ShaKatMsg2[] =
	"abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
	"hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu";
static const uint8_t s_ShaKatDigest2[32] = {
	0xCF, 0x5B, 0x16, 0xA7, 0x78, 0xAF, 0x83, 0x80, 0x03, 0x6C, 0xE5, 0x9E,
	0x7B, 0x04, 0x92, 0x37, 0x0B, 0x24, 0x9B, 0x11, 0xE8, 0xF0, 0x7A, 0x51,
	0xAF, 0xAC, 0x45, 0x03, 0x7A, 0xFE, 0xE9, 0xD1
};

static size_t ShaCtxSize()
{
	size_t soft = s_ShaSoft.HashCtxSize();

	return soft > sizeof(Lpc546xxShaCtx) ? soft : sizeof(Lpc546xxShaCtx);
}

static size_t ShaCtxAlign()
{
	size_t soft = s_ShaSoft.HashCtxAlign();

	return soft > alignof(Lpc546xxShaCtx) ? soft : alignof(Lpc546xxShaCtx);
}

// The reset clears the running hash and the digest registers
static void ShaResetPulse()
{
	SYSCON->PRESETCTRLSET[2] = SYSCON_PRESETCTRL_SHA_RST_MASK;
	SYSCON->PRESETCTRLCLR[2] = SYSCON_PRESETCTRL_SHA_RST_MASK;
}

// The block failed: software only from now on
static void ShaOff()
{
	uintptr_t state;

	ShaResetPulse();
	SYSCON->AHBCLKCTRLCLR[2] = SYSCON_AHBCLKCTRL_SHA0_MASK;
	state = DisableInterrupt();
	s_ShaState = LPC546XX_SHA_OFF;
	s_ShaOwner = nullptr;
	EnableInterrupt(state);
}

// The block goes to pCtx when no stream owns it. HashInit on the storage of
// the owner restarts that stream.
static bool ShaClaim(const void *pCtx)
{
	uintptr_t state = DisableInterrupt();
	bool claimed = s_ShaState == LPC546XX_SHA_READY &&
				   (s_ShaOwner == nullptr || s_ShaOwner == pCtx);

	if (claimed)
	{
		s_ShaOwner = pCtx;
	}
	EnableInterrupt(state);

	return claimed;
}

static bool ShaOwned(const void *pCtx)
{
	// Only the owner changes the owner from itself
	return s_ShaOwner == pCtx;
}

static bool ShaHwCtx(const void *pCtx)
{
	const Lpc546xxShaCtx *ctx = (const Lpc546xxShaCtx *)pCtx;

	return pCtx != nullptr && (uintptr_t)pCtx % alignof(Lpc546xxShaCtx) == 0U &&
		   ctx->Magic == LPC546XX_SHA_MAGIC && ctx->Used < 64U;
}

// Close a stream: the block, when this stream owns it, and the storage are
// cleared
static void ShaEnd(Lpc546xxShaCtx *pCtx)
{
	uintptr_t state;

	if (ShaOwned(pCtx))
	{
		ShaResetPulse();
		state = DisableInterrupt();
		s_ShaOwner = nullptr;
		EnableInterrupt(state);
	}
	CryptoSecureWipe(pCtx, sizeof(*pCtx));
}

static void ShaFail(Lpc546xxShaCtx *pCtx)
{
	CryptoSecureWipe(pCtx, sizeof(*pCtx));
	ShaOff();
}

static bool ShaWait(uint32_t Mask)
{
	uint32_t polls = LPC546XX_SHA_POLL_MAX;
	uint32_t status;

	do {
		status = SHA0->STATUS;
		if (status & SHA_STATUS_ERROR_MASK)
		{
			return false;
		}
		if (status & Mask)
		{
			return true;
		}
	} while (--polls > 0U);

	return false;
}

// Message bytes are written in order, each word loaded little endian (NXP
// SDK fsl_sha.c)
static uint32_t ShaWord(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
		   ((uint32_t)p[3] << 24);
}

static bool ShaBlock(const uint8_t *pBlk)
{
	unsigned i;

	if (!ShaWait(SHA_STATUS_WAITING_MASK))
	{
		return false;
	}
	SHA0->INDATA = ShaWord(pBlk);
	for (i = 0U; i < 7U; i++)
	{
		SHA0->ALIAS[i] = ShaWord(pBlk + 4U + 4U * i);
	}
	SHA0->INDATA = ShaWord(pBlk + 32U);
	for (i = 0U; i < 7U; i++)
	{
		SHA0->ALIAS[i] = ShaWord(pBlk + 36U + 4U * i);
	}

	return true;
}

static bool ShaDigest(uint8_t *pDigest)
{
	uint32_t d;
	unsigned i;

	if (!ShaWait(SHA_STATUS_DIGEST_MASK))
	{
		return false;
	}
	for (i = 0U; i < 8U; i++)
	{
		d = SHA0->DIGEST[i];
		pDigest[4U * i] = (uint8_t)(d >> 24);
		pDigest[4U * i + 1U] = (uint8_t)(d >> 16);
		pDigest[4U * i + 2U] = (uint8_t)(d >> 8);
		pDigest[4U * i + 3U] = (uint8_t)d;
	}

	return true;
}

static CRYPTO_STATUS ShaInit(CRYPTO_HASH_ALG Alg, void *pCtx)
{
	Lpc546xxShaCtx *ctx = (Lpc546xxShaCtx *)pCtx;

	if (Alg != CRYPTO_HASH_SHA256 || pCtx == nullptr ||
		(uintptr_t)pCtx % ShaCtxAlign() != 0U)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!ShaClaim(pCtx))
	{
		return s_ShaSoft.HashInit(Alg, pCtx);
	}
	memset(ctx, 0, sizeof(*ctx));
	ctx->Magic = LPC546XX_SHA_MAGIC;
	// A stream dropped on this storage may have left a block running
	ShaResetPulse();
	SHA0->CTRL = SHA_CTRL_MODE(2) | SHA_CTRL_NEW(1);

	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS ShaUpdate(void *pCtx, const uint8_t *pMsg, size_t Len)
{
	Lpc546xxShaCtx *ctx = (Lpc546xxShaCtx *)pCtx;
	size_t count;

	if (!ShaHwCtx(pCtx))
	{
		return s_ShaSoft.HashUpdate(pCtx, pMsg, Len);
	}
	if (pMsg == nullptr && Len != 0U)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!ShaOwned(pCtx) || Len > UINT64_MAX / 8U - ctx->Total)
	{
		ShaEnd(ctx);
		return CRYPTO_STATUS_FAIL;
	}
	if (Len == 0U)
	{
		return CRYPTO_STATUS_OK;
	}
	ctx->Total += Len;
	if (ctx->Used > 0U)
	{
		count = 64U - ctx->Used;
		if (count > Len)
		{
			count = Len;
		}
		memcpy(ctx->Buf + ctx->Used, pMsg, count);
		ctx->Used += count;
		pMsg += count;
		Len -= count;
		if (ctx->Used < 64U)
		{
			return CRYPTO_STATUS_OK;
		}
		if (!ShaBlock(ctx->Buf))
		{
			ShaFail(ctx);
			return CRYPTO_STATUS_FAIL;
		}
		ctx->Used = 0U;
	}
	while (Len >= 64U)
	{
		if (!ShaBlock(pMsg))
		{
			ShaFail(ctx);
			return CRYPTO_STATUS_FAIL;
		}
		pMsg += 64U;
		Len -= 64U;
	}
	if (Len > 0U)
	{
		memcpy(ctx->Buf, pMsg, Len);
		ctx->Used = Len;
	}

	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS ShaFinal(void *pCtx, uint8_t *pDigest)
{
	Lpc546xxShaCtx *ctx = (Lpc546xxShaCtx *)pCtx;
	uint64_t bits;
	unsigned i;
	bool ok = true;

	if (!ShaHwCtx(pCtx))
	{
		return s_ShaSoft.HashFinal(pCtx, pDigest);
	}
	if (pDigest == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!ShaOwned(pCtx))
	{
		ShaEnd(ctx);
		CryptoSecureWipe(pDigest, 32U);
		return CRYPTO_STATUS_FAIL;
	}
	bits = ctx->Total * 8U;
	ctx->Buf[ctx->Used++] = 0x80U;
	if (ctx->Used > 56U)
	{
		memset(ctx->Buf + ctx->Used, 0, 64U - ctx->Used);
		ok = ShaBlock(ctx->Buf);
		ctx->Used = 0U;
	}
	if (ok)
	{
		memset(ctx->Buf + ctx->Used, 0, 56U - ctx->Used);
		for (i = 0U; i < 8U; i++)
		{
			ctx->Buf[56U + i] = (uint8_t)(bits >> (56U - 8U * i));
		}
		ok = ShaBlock(ctx->Buf) && ShaDigest(pDigest);
	}
	if (!ok)
	{
		CryptoSecureWipe(pDigest, 32U);
		ShaFail(ctx);
		return CRYPTO_STATUS_FAIL;
	}
	ShaEnd(ctx);

	return CRYPTO_STATUS_OK;
}

static CRYPTO_STATUS ShaHash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len,
							 uint8_t *pDigest)
{
	alignas(CRYPTO_HASHCTX_ALIGN_MAX) uint8_t ctx[CRYPTO_HASHCTX_MAX];
	CRYPTO_STATUS status;

	if (Alg != CRYPTO_HASH_SHA256 || pDigest == nullptr ||
		(pMsg == nullptr && Len != 0U))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	status = ShaInit(Alg, ctx);
	if (status == CRYPTO_STATUS_OK)
	{
		status = ShaUpdate(ctx, pMsg, Len);
	}
	if (status == CRYPTO_STATUS_OK)
	{
		status = ShaFinal(ctx, pDigest);
	}
	if (status != CRYPTO_STATUS_OK)
	{
		CryptoSecureWipe(pDigest, 32U);
	}
	CryptoSecureWipe(ctx, sizeof(ctx));

	return status;
}

static bool ShaKat(const char *pMsg, size_t Len, const uint8_t *pExpect)
{
	alignas(CRYPTO_HASHCTX_ALIGN_MAX) uint8_t ctx[CRYPTO_HASHCTX_MAX];
	uint8_t digest[32];
	bool ok;

	// The stream has to run on the block, not in software
	ok = ShaInit(CRYPTO_HASH_SHA256, ctx) == CRYPTO_STATUS_OK && ShaHwCtx(ctx) &&
		 ShaUpdate(ctx, (const uint8_t *)pMsg, Len) == CRYPTO_STATUS_OK &&
		 ShaFinal(ctx, digest) == CRYPTO_STATUS_OK &&
		 memcmp(digest, pExpect, sizeof(digest)) == 0;
	CryptoSecureWipe(ctx, sizeof(ctx));

	return ok;
}

// Once after reset: the block is used when it passes the known-answer test.
// It runs before the first Crypto object is valid, so no other stream can
// take the block meanwhile.
static void ShaStart()
{
	uintptr_t state;

	if (s_ShaState != LPC546XX_SHA_UNTESTED)
	{
		return;
	}
	// HashInit resets the block before each stream
	SYSCON->AHBCLKCTRLSET[2] = SYSCON_AHBCLKCTRL_SHA0_MASK;
	state = DisableInterrupt();
	s_ShaState = LPC546XX_SHA_READY;
	EnableInterrupt(state);
	if (!ShaKat(s_ShaKatMsg1, sizeof(s_ShaKatMsg1) - 1U, s_ShaKatDigest1) ||
		!ShaKat(s_ShaKatMsg2, sizeof(s_ShaKatMsg2) - 1U, s_ShaKatDigest2))
	{
		ShaOff();
	}
}

#endif

bool Crypto::Init()
{

	if (vbValid)
	{
		return true;
	}
	if (!s_Hardware.Enable())
	{
		return false;
	}
#if LPC546XX_SHA_HW
	ShaStart();
#endif
	vbValid = true;
	return true;
}

CRYPTO_STATUS Crypto::Random(uint8_t *pOut, size_t Len)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Hardware.Random(pOut, Len);
}

bool Crypto::IsSecure() const
{
	if (!vbValid)
	{
		return false;
	}
	return s_Hardware.IsSecure();
}

#if LPC546XX_SHA_HW

CRYPTO_STATUS Crypto::Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len, uint8_t *pDigest)
{
	if (!vbValid)
	{
		if (pDigest != nullptr && Alg == CRYPTO_HASH_SHA256)
		{
			CryptoSecureWipe(pDigest, 32U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return ShaHash(Alg, pMsg, Len, pDigest);
}

size_t Crypto::HashCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	return ShaCtxSize();
}

size_t Crypto::HashCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	return ShaCtxAlign();
}

CRYPTO_STATUS Crypto::HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return ShaInit(Alg, pHashCtx);
}

CRYPTO_STATUS Crypto::HashUpdate(void *pHashCtx, const uint8_t *pMsg, size_t Len)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return ShaUpdate(pHashCtx, pMsg, Len);
}

CRYPTO_STATUS Crypto::HashFinal(void *pHashCtx, uint8_t *pDigest)
{
	if (!vbValid)
	{
		if (pDigest != nullptr)
		{
			CryptoSecureWipe(pDigest, 32U);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return ShaFinal(pHashCtx, pDigest);
}

#endif
