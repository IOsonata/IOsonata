/**-------------------------------------------------------------------------
@file	crypto_esp32.cpp
@brief	ESP32-C3/C6 AES overrides of the generic crypto implementation.

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
#include "esp32xx.h"
#include "coredev/interrupt.h"
#include "crypto/crypto.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"

#if !defined(ESP32C3) && !defined(ESP32C6)
#error "Crypto register mapping is verified only for ESP32-C3/C6"
#endif

#ifndef ESP32_CRYPTO_REG
#define ESP32_CRYPTO_REG(addr) (*(volatile uint32_t *)(uintptr_t)(addr))
#endif
#ifndef ESP32_CRYPTO_POLL_MAX
#define ESP32_CRYPTO_POLL_MAX 1000000U
#endif

// Register offsets follow the C3/C6 TRMs and ESP-IDF v5.4.2 aes_ll.h.
#define AES_REG(offset) ESP32_CRYPTO_REG(ESP32_AES_BASE + (offset))
#define SHA_REG(offset) ESP32_CRYPTO_REG(ESP32_SHA_BASE + (offset))
#define ESP_SHA_MAGIC 0x53484145UL
struct EspShaCtx
{
	uint32_t Magic;
	uint32_t H[8];
	uint64_t Total;
	uint8_t Buf[64];
	size_t Used;
	bool Started;
};
static_assert(sizeof(EspShaCtx) <= CRYPTO_HASHCTX_MAX, "SHA storage too small");
static bool s_ShaBusy;
static bool s_AesBusy = false;

class CryptoEsp32 : public CryptoAesModes, public HashEngine
{
public:
	CRYPTO_STATUS Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg, size_t Len, uint8_t *pDigest) override;
	size_t HashCtxSize() const override;
	size_t HashCtxAlign() const override;
	CRYPTO_STATUS HashInit(CRYPTO_HASH_ALG Alg, void *pCtx) override;
	CRYPTO_STATUS HashUpdate(void *pCtx, const uint8_t *pMsg, size_t Len) override;
	CRYPTO_STATUS HashFinal(void *pCtx, uint8_t *pDigest) override;
	bool Enable() override;
	CRYPTO_STATUS Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
		const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len,
		uint8_t *pOut) override;
protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
		uint8_t Out[16]) override;
private:
	bool Block(const uint8_t *pKey, size_t KeyLen, bool Encrypt,
		const uint8_t In[16], uint8_t Out[16]);
};

static CryptoEsp32 s_Hardware;

static void AesReset()
{
	uintptr_t state = DisableInterrupt();

#if defined(ESP32C3)
	ESP32_CRYPTO_REG(ESP32_SYSTEM_BASE + 0x14U) |= 2U;
	ESP32_CRYPTO_REG(ESP32_SYSTEM_BASE + 0x1CU) |= 2U;
	// DS reset also holds AES in reset. Preserve SHA, RSA and HMAC.
	ESP32_CRYPTO_REG(ESP32_SYSTEM_BASE + 0x1CU) &= ~18U;
#else
	ESP32_CRYPTO_REG(ESP32_PCR_BASE + 0xC8U) |= 3U;
	ESP32_CRYPTO_REG(ESP32_PCR_BASE + 0xC8U) &= ~2U;
	ESP32_CRYPTO_REG(ESP32_PCR_BASE + 0xE0U) &= ~2U;
#endif
	EnableInterrupt(state);
}

bool CryptoEsp32::Enable()
{
	// Clock/reset is performed only after acquiring the shared peripheral.
	vbValid = true;
	return true;
}

bool CryptoEsp32::AesOpBegin()
{
	uintptr_t state = DisableInterrupt();
	bool ok = !s_AesBusy;

	if (ok)
	{
		s_AesBusy = true;
	}
	EnableInterrupt(state);
	if (ok)
	{
		AesReset();
	}
	return ok;
}

void CryptoEsp32::AesOpEnd()
{
	uintptr_t state;
	unsigned i;

	AesReset();
	for (i = 0U; i < 16U; i++)
	{
		AES_REG(i * 4U) = 0U;
	}
	state = DisableInterrupt();
	s_AesBusy = false;
	EnableInterrupt(state);
}

bool CryptoEsp32::Block(const uint8_t *pKey, size_t KeyLen, bool Encrypt,
	const uint8_t In[16], uint8_t Out[16])
{
	uint32_t word = 0U;
	uint32_t polls = ESP32_CRYPTO_POLL_MAX;
	unsigned i;

	AES_REG(0x90U) = 0U; // CPU block access, no DMA.
	AES_REG(0xB0U) = 0U;
	AES_REG(0x44U) = 0U;
	AES_REG(0x40U) = (Encrypt ? 0U : 4U) | (KeyLen == 32U ? 2U : 0U);
	for (i = 0U; i < KeyLen; i += 4U)
	{
		memcpy(&word, pKey + i, sizeof(word));
		AES_REG(i) = word;
	}
	for (i = 0U; i < 16U; i += 4U)
	{
		memcpy(&word, In + i, sizeof(word));
		AES_REG(0x20U + i) = word;
	}
	AES_REG(0x48U) = 1U;
	while (AES_REG(0x4CU) != 0U && polls > 0U)
	{
		polls--;
	}
	if (AES_REG(0x4CU) != 0U)
	{
		CryptoSecureWipe(&word, sizeof(word));
		CryptoSecureWipe(Out, 16U);
		return false;
	}
	for (i = 0U; i < 16U; i += 4U)
	{
		word = AES_REG(0x30U + i);
		memcpy(Out + i, &word, sizeof(word));
	}
	CryptoSecureWipe(&word, sizeof(word));
	return true;
}

bool CryptoEsp32::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
	uint8_t Out[16])
{
	return Block(Key, 16U, true, In, Out);
}

CRYPTO_STATUS CryptoEsp32::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
	const CryptoKey &Key, const uint8_t *pIv, size_t IvLen,
	const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	uint8_t chain[16] = {};
	uint8_t input[16] = {};
	uint8_t output[16] = {};
	size_t off;
	size_t i;
	size_t count;
	int digit;
	bool ok = true;
	bool encrypt = bEncrypt != 0;
	size_t keyLen = Key.Type == CRYPTO_KEY_AES_256 ? 32U : 16U;
	uint32_t usage = encrypt ? CRYPTO_KEY_USE_ENCRYPT : CRYPTO_KEY_USE_DECRYPT;

	if ((Key.Type != CRYPTO_KEY_AES_128 && Key.Type != CRYPTO_KEY_AES_256) ||
		Key.Loc != CRYPTO_KEY_LOC_PLAIN || Key.Plain.pData == nullptr ||
		Key.Plain.Len != keyLen || (Key.Usage & usage) == 0U ||
		pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (Alg != CRYPTO_CIPHER_ECB && Alg != CRYPTO_CIPHER_CBC && Alg != CRYPTO_CIPHER_CTR)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if ((Alg != CRYPTO_CIPHER_CTR && (Len & 15U) != 0U) ||
		(Alg != CRYPTO_CIPHER_ECB && (pIv == nullptr || IvLen != 16U)))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!AesOpBegin())
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_BUSY;
	}
	if (Alg != CRYPTO_CIPHER_ECB)
	{
		memcpy(chain, pIv, sizeof(chain));
	}
	for (off = 0U; off < Len; off += count)
	{
		count = Len - off < 16U ? Len - off : 16U;
		memcpy(input, pIn + off, count);
		if (Alg == CRYPTO_CIPHER_CBC && encrypt)
		{
			for (i = 0U; i < 16U; i++)
			{
				input[i] ^= chain[i];
			}
		}
		ok = Block(Key.Plain.pData, keyLen, encrypt || Alg == CRYPTO_CIPHER_CTR,
			Alg == CRYPTO_CIPHER_CTR ? chain : input, output);
		if (!ok)
		{
			break;
		}
		if (Alg == CRYPTO_CIPHER_CTR)
		{
			for (i = 0U; i < count; i++)
			{
				output[i] ^= input[i];
			}
			for (digit = 15; digit >= 0; digit--)
			{
				if (++chain[digit] != 0U)
				{
					break;
				}
			}
		}
		else if (Alg == CRYPTO_CIPHER_CBC)
		{
			if (!encrypt)
			{
				for (i = 0U; i < 16U; i++)
				{
					output[i] ^= chain[i];
				}
			}
			memcpy(chain, encrypt ? output : input, sizeof(chain));
		}
		memcpy(pOut + off, output, count);
	}
	AesOpEnd();
	CryptoSecureWipe(chain, sizeof(chain));
	CryptoSecureWipe(input, sizeof(input));
	CryptoSecureWipe(output, sizeof(output));
	if (!ok)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

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
	vbValid = true;
	return true;
}

// Caller-owned digest state allows interleaved streams. Peripheral ownership
// lasts one compression block, never the lifetime of a caller's hash context.
static void ShaReset()
{
	uintptr_t state = DisableInterrupt();

#if defined(ESP32C3)
	ESP32_CRYPTO_REG(ESP32_SYSTEM_BASE + 0x14U) |= 4U;
	ESP32_CRYPTO_REG(ESP32_SYSTEM_BASE + 0x1CU) |= 4U;
	ESP32_CRYPTO_REG(ESP32_SYSTEM_BASE + 0x1CU) &= ~52U;
#else
	ESP32_CRYPTO_REG(ESP32_PCR_BASE + 0xCCU) |= 3U;
	ESP32_CRYPTO_REG(ESP32_PCR_BASE + 0xCCU) &= ~2U;
	ESP32_CRYPTO_REG(ESP32_PCR_BASE + 0xE0U) &= ~2U;
	ESP32_CRYPTO_REG(ESP32_PCR_BASE + 0xE4U) &= ~2U;
#endif
	EnableInterrupt(state);
}

static CRYPTO_STATUS ShaBlock(EspShaCtx *pCtx)
{
	uintptr_t state = DisableInterrupt();
	unsigned i;
	uint32_t word;
	uint32_t nonzero = 0U;
	uint32_t polls = ESP32_CRYPTO_POLL_MAX;
	bool ok;

	if (s_ShaBusy)
	{
		EnableInterrupt(state);
		return CRYPTO_STATUS_BUSY;
	}
	s_ShaBusy = true;
	EnableInterrupt(state);
	ShaReset();
	for (i = 0U; i < 8U; i++)
	{
		SHA_REG(0x40U + 4U * i) = pCtx->H[i];
	}
	for (i = 0U; i < 64U; i += 4U)
	{
		memcpy(&word, pCtx->Buf + i, sizeof(word));
		SHA_REG(0x80U + i) = word;
	}
	SHA_REG(0x00U) = 2U;
	SHA_REG(pCtx->Started ? 0x14U : 0x10U) = 1U;
	while (SHA_REG(0x18U) != 0U && polls > 0U)
	{
		polls--;
	}
	ok = SHA_REG(0x18U) == 0U;
	if (ok)
	{
		for (i = 0U; i < 8U; i++)
		{
			pCtx->H[i] = SHA_REG(0x40U + 4U * i);
			nonzero |= pCtx->H[i];
		}
		ok = nonzero != 0U;
	}
	ShaReset();
	for (i = 0U; i < 8U; i++)
	{
		SHA_REG(0x40U + 4U * i) = 0U;
	}
	for (i = 0U; i < 16U; i++)
	{
		SHA_REG(0x80U + 4U * i) = 0U;
	}
	CryptoSecureWipe(&word, sizeof(word));
	state = DisableInterrupt();
	s_ShaBusy = false;
	EnableInterrupt(state);
	pCtx->Started = true;
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

static bool ShaLive(const void *pCtx)
{
	return pCtx != nullptr && (uintptr_t)pCtx % alignof(EspShaCtx) == 0U &&
		((const EspShaCtx *)pCtx)->Magic == ESP_SHA_MAGIC &&
		((const EspShaCtx *)pCtx)->Used < 64U;
}

size_t CryptoEsp32::HashCtxSize() const
{
	return sizeof(EspShaCtx);
}
size_t CryptoEsp32::HashCtxAlign() const
{
	return alignof(EspShaCtx);
}

CRYPTO_STATUS CryptoEsp32::HashInit(CRYPTO_HASH_ALG Alg, void *pCtx)
{
	if (Alg != CRYPTO_HASH_SHA256 || pCtx == nullptr ||
		(uintptr_t)pCtx % alignof(EspShaCtx) != 0U)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	memset(pCtx, 0, sizeof(EspShaCtx));
	((EspShaCtx *)pCtx)->Magic = ESP_SHA_MAGIC;
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoEsp32::HashUpdate(void *pCtx, const uint8_t *pMsg, size_t Len)
{
	EspShaCtx *ctx = (EspShaCtx *)pCtx;
	size_t count;
	CRYPTO_STATUS status;

	if (!ShaLive(pCtx) || (pMsg == nullptr && Len != 0U))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (Len > UINT64_MAX / 8U - ctx->Total)
	{
		CryptoSecureWipe(ctx, sizeof(*ctx));
		return CRYPTO_STATUS_FAIL;
	}
	ctx->Total += Len;
	while (Len > 0U)
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
		if (ctx->Used == 64U)
		{
			status = ShaBlock(ctx);
			if (status != CRYPTO_STATUS_OK)
			{
				CryptoSecureWipe(ctx, sizeof(*ctx));
				return status;
			}
			ctx->Used = 0U;
		}
	}
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoEsp32::HashFinal(void *pCtx, uint8_t *pDigest)
{
	EspShaCtx *ctx = (EspShaCtx *)pCtx;
	uint64_t bits;
	unsigned i;
	CRYPTO_STATUS status = CRYPTO_STATUS_OK;

	if (!ShaLive(pCtx) || pDigest == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	bits = ctx->Total * 8U;
	ctx->Buf[ctx->Used++] = 0x80U;
	if (ctx->Used > 56U)
	{
		memset(ctx->Buf + ctx->Used, 0, 64U - ctx->Used);
		status = ShaBlock(ctx);
		ctx->Used = 0U;
	}
	if (status == CRYPTO_STATUS_OK)
	{
		memset(ctx->Buf + ctx->Used, 0, 56U - ctx->Used);
		for (i = 0U; i < 8U; i++)
		{
			ctx->Buf[56U + i] = (uint8_t)(bits >> (56U - 8U * i));
		}
		status = ShaBlock(ctx);
	}
	if (status == CRYPTO_STATUS_OK)
	{
		memcpy(pDigest, ctx->H, 32U);
	}
	else
	{
		CryptoSecureWipe(pDigest, 32U);
	}
	CryptoSecureWipe(ctx, sizeof(*ctx));
	return status;
}

CRYPTO_STATUS CryptoEsp32::Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg,
	size_t Len, uint8_t *pDigest)
{
	EspShaCtx ctx;
	CRYPTO_STATUS status;

	if (Alg != CRYPTO_HASH_SHA256 || pDigest == nullptr ||
		(pMsg == nullptr && Len != 0U))
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	status = HashInit(Alg, &ctx);
	if (status == CRYPTO_STATUS_OK)
	{
		status = HashUpdate(&ctx, pMsg, Len);
	}
	if (status == CRYPTO_STATUS_OK)
	{
		status = HashFinal(&ctx, pDigest);
	}
	if (status != CRYPTO_STATUS_OK)
	{
		CryptoSecureWipe(pDigest, 32U);
	}
	CryptoSecureWipe(&ctx, sizeof(ctx));
	return status;
}

CRYPTO_STATUS Crypto::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt, const CryptoKey &Key,
			const uint8_t *pIv, size_t IvLen, const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Hardware.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
}

CRYPTO_STATUS Crypto::Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		if (pTag != nullptr)
		{
			CryptoSecureWipe(pTag, TagLen);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Hardware.Seal(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
}

CRYPTO_STATUS Crypto::Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
			const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
			const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen)
{
	if (!vbValid)
	{
		if (pOut != nullptr)
		{
			CryptoSecureWipe(pOut, Len);
		}
		return CRYPTO_STATUS_FAIL;
	}
	return s_Hardware.Open(Alg, Key, pNonce, NonceLen, pAad, AadLen, pMsg, Len, pOut, pTag, TagLen);
}

CRYPTO_STATUS Crypto::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
			const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen)
{
	if (!vbValid)
	{
		if (pMac != nullptr)
		{
			CryptoSecureWipe(pMac, MacLen);
		}
		return CRYPTO_STATUS_FAIL;
	}
	if (Alg == CRYPTO_MAC_HMAC)
	{
		return CryptoHmacSha256(this, Alg, Key, pMsg, Len, pMac, MacLen);
	}
	return s_Hardware.Mac(Alg, Key, pMsg, Len, pMac, MacLen);
}

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
	return s_Hardware.Hash(Alg, pMsg, Len, pDigest);
}

size_t Crypto::HashCtxSize() const
{
	if (!vbValid)
	{
		return 0U;
	}
	return s_Hardware.HashCtxSize();
}

size_t Crypto::HashCtxAlign() const
{
	if (!vbValid)
	{
		return 1U;
	}
	return s_Hardware.HashCtxAlign();
}

CRYPTO_STATUS Crypto::HashInit(CRYPTO_HASH_ALG Alg, void *pHashCtx)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return s_Hardware.HashInit(Alg, pHashCtx);
}

CRYPTO_STATUS Crypto::HashUpdate(void *pHashCtx, const uint8_t *pMsg, size_t Len)
{
	if (!vbValid)
	{
		return CRYPTO_STATUS_FAIL;
	}
	return s_Hardware.HashUpdate(pHashCtx, pMsg, Len);
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
	return s_Hardware.HashFinal(pHashCtx, pDigest);
}

