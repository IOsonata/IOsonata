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
static bool s_AesBusy = false;

class CryptoEsp32 : public Crypto {
public:
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
	static CryptoEsp32 hardware;

	if (vbValid)
	{
		return true;
	}
	if (!hardware.Enable())
	{
		return false;
	}
	vpHardware = &hardware;
	vbValid = true;
	return true;
}
