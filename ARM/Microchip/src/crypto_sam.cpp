/**-------------------------------------------------------------------------
@file	crypto_sam.cpp
@brief	Default crypto engines selected by the target library.

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
#include "crypto_sam.h"

bool CryptoSam::AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
	uint8_t Out[16])
{
	return Block(Key, 16U, true, In, Out);
}

CRYPTO_STATUS CryptoSam::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
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

	if (Key.Type == CRYPTO_KEY_AES_256 && !WideKey())
	{
		return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}
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
	static CryptoSam hardware;

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
