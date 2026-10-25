/**-------------------------------------------------------------------------
@file	crypto_cc3xx_sym.cpp
@brief	CryptoCell AES and streaming SHA-256 overrides.

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
#include "crypto_cc3xx_sym.h"

#ifndef CC3XX_SYM_POLL_MAX
#define CC3XX_SYM_POLL_MAX 100000U
#endif
#define CC_SHA_MAGIC 0x53484343UL
enum : uint32_t {
	REG_AES_KEY0 = 0x400U,
	REG_AES_CONTROL = 0x4C0U,
	REG_AES_REMAINING = 0x4BCU,
	REG_AES_CLK = 0x810U,
	REG_HASH_CLK = 0x818U,
	REG_DMA_CLK = 0x820U,
	REG_CRYPTO_BUSY = 0x910U,
	REG_HOST_ENDIAN = 0xA0CU,
	REG_HOST_ICR = 0xA08U,
	REG_HOST_IMR = 0xA04U,
	REG_DIN_RESET = 0xC58U,
	REG_HASH_H0 = 0x640U,
	REG_CRYPTO_CTL = 0x900U,
	REG_DIN_EMPTY = 0xC50U,
	REG_DIN = 0xC00U,
	REG_DIN_SIZE = 0xC48U,
	REG_DOUT_EMPTY = 0xD50U,
#if defined(NRF52840_XXAA) || defined(NRF91_SERIES)
	// CC310 (nRF52840 and nRF91) shares the CPU DIN/DOUT buffer at 0xC00.
	REG_DOUT = 0xC00U,
#else
	// CC312 has a separate DOUT buffer window.
	REG_DOUT = 0xD00U,
#endif
	REG_HASH_SEL_AES_MAC = 0x6A4U,
	REG_HASH_CONTROL = 0x7C0U,
	REG_HASH_PAD_ENABLE = 0x7C4U,
	REG_HASH_AUTO_PAD = 0x684U,
	REG_HASH_PAD_CFG = 0x7C8U,
	REG_HASH_BUSY = 0x91CU,
};

struct CcShaCtx {
	uint32_t Magic;
	uint32_t H[8];
	uint64_t Total;
	uint8_t Buf[64];
	size_t Used;
};
static const uint32_t s_ClearWords[8] = {};
static const uint32_t s_InitialHash[8] = {
	0x6A09E667U, 0xBB67AE85U, 0x3C6EF372U, 0xA54FF53AU,
	0x510E527FU, 0x9B05688CU, 0x1F83D9ABU, 0x5BE0CD19U
};
static_assert(sizeof(CcShaCtx) <= CRYPTO_HASHCTX_MAX, "Hash storage too small");

// Register layout and CPU FIFO operation follow the Arm CC312 runtime
// register description and its AES/hash external-DMA setup. No memory DMA
// is started: only the CPU accesses DIN/DOUT, with bounded ready waits.
uint32_t CryptoCc3xxSym::RegRead(uint32_t Offset)
{
	uint32_t value = 0U;
	uint8_t address[4] = {(uint8_t)Offset, (uint8_t)(Offset >> 8),
		(uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24)};
	if (Read(address, 4, (uint8_t *)&value, 4) != 4)
	{
		vFault = true;
	}
	return value;
}

void CryptoCc3xxSym::RegWrite(uint32_t Offset, uint32_t Value)
{
	RegWriteBlock(Offset, &Value, sizeof(Value));
}

void CryptoCc3xxSym::RegWriteBlock(uint32_t Offset, const void *pData, size_t Len)
{
	uint8_t address[4] = {(uint8_t)Offset, (uint8_t)(Offset >> 8),
		(uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24)};
	if (Write(address, 4, (const uint8_t *)pData, Len) != (int)Len)
	{
		vFault = true;
	}
}

bool CryptoCc3xxSym::Wait(uint32_t Offset, uint32_t Expected)
{
	uint32_t i;
	for (i = 0U; i < CC3XX_SYM_POLL_MAX && !vFault; i++)
	{
		if (RegRead(Offset) == Expected && !vFault)
		{
			return true;
		}
	}
	vFault = true;
	return false;
}

bool CryptoCc3xxSym::Init(Cc3xxIntrf *pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	Interface(pIntrf);
	DeviceAddress(CC3XX_ADDR_REG);
	return Enable();
}

bool CryptoCc3xxSym::Enable()
{
	if (vbValid)
	{
		return true;
	}
	if (Interface() == nullptr)
	{
		return false;
	}
	if (!vEnabled)
	{
		Interface()->Enable();
		vEnabled = true;
	}
	vbValid = Cc3xxEnable();
	return vbValid;
}

void CryptoCc3xxSym::Disable()
{
	if (vEnabled)
	{
		Interface()->Disable();
		vEnabled = false;
	}
	vbValid = false;
}

void CryptoCc3xxSym::Reset()
{
	Cc3xxIntrf *intrf = (Cc3xxIntrf *)Interface();
	if (intrf == nullptr || !intrf->OpHold(this))
	{
		return;
	}
	Cc3xxDisable();
	vbValid = Cc3xxEnable();
	vFault = !vbValid;
	intrf->OpRelease(this);
}

CRYPTO_STATUS CryptoCc3xxSym::Begin()
{
	Cc3xxIntrf *intrf = (Cc3xxIntrf *)Interface();
	if (!vbValid || intrf == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (!intrf->OpHold(this))
	{
		return CRYPTO_STATUS_BUSY;
	}
	vFault = !Cc3xxEnable();
	vAesReady = false;
	RegWrite(REG_AES_CLK, 1U);
	RegWrite(REG_HASH_CLK, 1U);
	RegWrite(REG_DMA_CLK, 1U);
	if (!Wait(REG_CRYPTO_BUSY, 0U))
	{
		End();
		return CRYPTO_STATUS_FAIL;
	}
	RegWrite(REG_HOST_ENDIAN, 0U);
	RegWrite(REG_HOST_ICR, 0xBF0U);
	RegWrite(REG_HOST_IMR, RegRead(REG_HOST_IMR) | 0xBF0U);
	RegWrite(REG_DIN_RESET, 0U);
	if (vFault)
	{
		End();
		return CRYPTO_STATUS_FAIL;
	}
	return CRYPTO_STATUS_OK;
}

void CryptoCc3xxSym::End()
{
	RegWriteBlock(REG_AES_KEY0, s_ClearWords, sizeof(s_ClearWords));
	RegWriteBlock(REG_HASH_H0, s_ClearWords, sizeof(s_ClearWords));
	RegWrite(REG_CRYPTO_CTL, 0U);
	RegWrite(REG_DIN_RESET, 0U);
	RegWrite(REG_AES_CLK, 0U);
	RegWrite(REG_HASH_CLK, 0U);
	RegWrite(REG_DMA_CLK, 0U);
	if (vFault)
	{
		// No DMA addresses were exposed. Power-down discards partial FIFO state.
		Cc3xxDisable();
	}
	((Cc3xxIntrf *)Interface())->OpRelease(this);
}

bool CryptoCc3xxSym::AesOpBegin() { return Begin() == CRYPTO_STATUS_OK; }
void CryptoCc3xxSym::AesOpEnd() { End(); }

bool CryptoCc3xxSym::Feed(const uint8_t *pIn, size_t Len)
{
	uint32_t word = 0U;
	size_t off;
	// Feed four words at a time so AES can consume a complete block.
	for (off = 0U; off < Len; off += 4U)
	{
		if ((off & 15U) == 0U && !Wait(REG_DIN_EMPTY, 1U))
		{
			break;
		}
		memcpy(&word, pIn + off, 4U);
		RegWrite(REG_DIN, word);
	}
	CryptoSecureWipe(&word, sizeof(word));
	return !vFault;
}

bool CryptoCc3xxSym::Block(const uint8_t *pKey, size_t KeyLen, bool Encrypt,
	const uint8_t In[16], uint8_t Out[16])
{
	uint32_t word = 0U;
	unsigned i;
	// Cipher and the common CMAC/AEAD loops use one key and direction
	// for the whole operation. End wipes it before releasing the interface.
	if (!vAesReady)
	{
		RegWrite(REG_CRYPTO_CTL, 1U);
		RegWrite(REG_AES_CONTROL, (Encrypt ? 0U : 1U) | (KeyLen == 32U ? 0x2000U : 0U));
		RegWriteBlock(REG_AES_KEY0, pKey, KeyLen);
		vAesReady = !vFault;
	}
	RegWrite(REG_AES_REMAINING, 16U);
	RegWrite(REG_DIN_SIZE, 16U);
	if (Feed(In, 16U))
	{
		for (i = 0U; i < 16U; i += 4U)
		{
			if (!Wait(REG_DOUT_EMPTY, 0U))
			{
				break;
			}
			word = RegRead(REG_DOUT);
			memcpy(Out + i, &word, 4U);
		}
	}
	CryptoSecureWipe(&word, sizeof(word));
	if (vFault)
	{
		CryptoSecureWipe(Out, 16U);
	}
	return !vFault;
}

bool CryptoCc3xxSym::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
	uint8_t Out[16])
{
	return Block(Key, 16U, true, In, Out);
}

CRYPTO_STATUS CryptoCc3xxSym::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
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
	CRYPTO_STATUS status;
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
#if defined(NRF52840_XXAA) || defined(NRF91_SERIES)
	// CC310 provides AES-128 only; preserve AES-256 through the generic
	// software implementation rather than passing an unsupported key size.
	if (Key.Type == CRYPTO_KEY_AES_256)
	{
		CryptoSoftAes soft;
		soft.Enable();
		return soft.Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}
#endif
	status = Begin();
	if (status != CRYPTO_STATUS_OK)
	{
		CryptoSecureWipe(pOut, Len);
		return status;
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
	ok = ok && !vFault;
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

// The generic AES end hook has no return value. Propagate a failed register
// cleanup through these complete operations as well as through Cipher.
CRYPTO_STATUS CryptoCc3xxSym::Mac(CRYPTO_MAC_ALG Alg, const CryptoKey &Key,
	const uint8_t *pMsg, size_t Len, uint8_t *pMac, size_t MacLen)
{
	CRYPTO_STATUS status = (Alg == CRYPTO_MAC_HMAC ? CryptoHmacSha256(this, Alg, Key, pMsg, Len, pMac, MacLen) : CryptoAesModes::Mac(Alg, Key, pMsg, Len, pMac, MacLen));
	if (status == CRYPTO_STATUS_OK && vFault)
	{
		CryptoSecureWipe(pMac, MacLen);
		return CRYPTO_STATUS_FAIL;
	}
	return status;
}

CRYPTO_STATUS CryptoCc3xxSym::Seal(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
	const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
	const uint8_t *pMsg, size_t Len, uint8_t *pOut, uint8_t *pTag, size_t TagLen)
{
	CRYPTO_STATUS status = CryptoAesModes::Seal(Alg, Key, pNonce, NonceLen, pAad,
		AadLen, pMsg, Len, pOut, pTag, TagLen);
	if (status == CRYPTO_STATUS_OK && vFault)
	{
		CryptoSecureWipe(pOut, Len);
		CryptoSecureWipe(pTag, TagLen);
		return CRYPTO_STATUS_FAIL;
	}
	return status;
}

CRYPTO_STATUS CryptoCc3xxSym::Open(CRYPTO_AEAD_ALG Alg, const CryptoKey &Key,
	const uint8_t *pNonce, size_t NonceLen, const uint8_t *pAad, size_t AadLen,
	const uint8_t *pMsg, size_t Len, uint8_t *pOut, const uint8_t *pTag, size_t TagLen)
{
	CRYPTO_STATUS status = CryptoAesModes::Open(Alg, Key, pNonce, NonceLen, pAad,
		AadLen, pMsg, Len, pOut, pTag, TagLen);
	if (status == CRYPTO_STATUS_OK && vFault)
	{
		CryptoSecureWipe(pOut, Len);
		return CRYPTO_STATUS_FAIL;
	}
	return status;
}

CRYPTO_STATUS CryptoCc3xxSym::HashBlock(void *pCtx)
{
	CcShaCtx *ctx = (CcShaCtx *)pCtx;
	int i;
	bool ok;
	CRYPTO_STATUS status = Begin();
	if (status != CRYPTO_STATUS_OK)
	{
		return status;
	}
	RegWrite(REG_CRYPTO_CTL, 7U);
	RegWrite(REG_HASH_SEL_AES_MAC, 0U);
	RegWrite(REG_HASH_CONTROL, 2U);
	RegWrite(REG_HASH_PAD_ENABLE, 0U);
	RegWrite(REG_HASH_AUTO_PAD, 0U);
	RegWrite(REG_HASH_PAD_CFG, 0U);
	for (i = 7; i >= 0; i--)
	{
		RegWrite(REG_HASH_H0 + 4U * i, ctx->H[i]);
	}
	RegWrite(REG_DIN_SIZE, 64U);
	ok = Feed(ctx->Buf, 64U) && Wait(REG_DIN_EMPTY, 1U) && Wait(REG_HASH_BUSY, 0U);
	if (ok)
	{
		for (i = 7; i >= 0; i--)
		{
			ctx->H[i] = RegRead(REG_HASH_H0 + 4U * i);
		}
	}
	End();
	return ok && !vFault ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

static bool ShaLive(const void *pCtx)
{
	return pCtx != nullptr && (uintptr_t)pCtx % alignof(CcShaCtx) == 0U &&
		((const CcShaCtx *)pCtx)->Magic == CC_SHA_MAGIC &&
		((const CcShaCtx *)pCtx)->Used < 64U;
}

size_t CryptoCc3xxSym::HashCtxSize() const { return sizeof(CcShaCtx); }
size_t CryptoCc3xxSym::HashCtxAlign() const { return alignof(CcShaCtx); }

CRYPTO_STATUS CryptoCc3xxSym::HashInit(CRYPTO_HASH_ALG Alg, void *pCtx)
{
	if (Alg != CRYPTO_HASH_SHA256 || pCtx == nullptr ||
		(uintptr_t)pCtx % alignof(CcShaCtx) != 0U)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	memset(pCtx, 0, sizeof(CcShaCtx));
	((CcShaCtx *)pCtx)->Magic = CC_SHA_MAGIC;
	memcpy(((CcShaCtx *)pCtx)->H, s_InitialHash, sizeof(s_InitialHash));
	return CRYPTO_STATUS_OK;
}

CRYPTO_STATUS CryptoCc3xxSym::HashUpdate(void *pCtx, const uint8_t *pMsg, size_t Len)
{
	CcShaCtx *ctx = (CcShaCtx *)pCtx;
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
			status = HashBlock(ctx);
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

CRYPTO_STATUS CryptoCc3xxSym::HashFinal(void *pCtx, uint8_t *pDigest)
{
	CcShaCtx *ctx = (CcShaCtx *)pCtx;
	uint64_t bits;
	unsigned i;
	uint32_t word;
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
		status = HashBlock(ctx);
		ctx->Used = 0U;
	}
	if (status == CRYPTO_STATUS_OK)
	{
		memset(ctx->Buf + ctx->Used, 0, 56U - ctx->Used);
		for (i = 0U; i < 8U; i++)
		{
			ctx->Buf[56U + i] = (uint8_t)(bits >> (56U - 8U * i));
		}
		status = HashBlock(ctx);
	}
	if (status == CRYPTO_STATUS_OK)
	{
		for (i = 0U; i < 8U; i++)
		{
			word = __builtin_bswap32(ctx->H[i]);
			memcpy(pDigest + 4U * i, &word, 4U);
		}
	}
	else
	{
		CryptoSecureWipe(pDigest, 32U);
	}
	CryptoSecureWipe(ctx, sizeof(*ctx));
	return status;
}

CRYPTO_STATUS CryptoCc3xxSym::Hash(CRYPTO_HASH_ALG Alg, const uint8_t *pMsg,
	size_t Len, uint8_t *pDigest)
{
	CcShaCtx ctx;
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
