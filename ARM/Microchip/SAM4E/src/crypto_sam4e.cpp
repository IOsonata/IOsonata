/**-------------------------------------------------------------------------
@file	crypto_sam4e.cpp
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
#include "sam4e.h"
#include "coredev/interrupt.h"
#include "crypto_sam.h"

static bool s_AesBusy = false;

bool CryptoSam::Enable()
{
	if (vbValid)
	{
		return true;
	}
	PMC->PMC_PCER1 = 1UL << (ID_AES - 32U);
	(void)PMC->PMC_PCSR1;
	vbValid = true;
	return true;
}

bool CryptoSam::WideKey() const
{
	return true;
}

bool CryptoSam::AesOpBegin()
{
	uintptr_t state = DisableInterrupt();
	bool ok = !s_AesBusy;

	if (ok)
	{
		s_AesBusy = true;
	}
	EnableInterrupt(state);
	return ok;
}

void CryptoSam::AesOpEnd()
{
	uintptr_t state;
	unsigned i;

	AES->AES_CR = AES_CR_SWRST;
	for (i = 0U; i < 8U; i++)
	{
		AES->AES_KEYWR[i] = 0U;
	}
	state = DisableInterrupt();
	s_AesBusy = false;
	EnableInterrupt(state);
}

bool CryptoSam::Block(const uint8_t *pKey, size_t KeyLen, bool Encrypt,
	const uint8_t In[16], uint8_t Out[16])
{
	uint32_t word;
	uint32_t polls = 1000000U;
	unsigned i;

	AES->AES_CR = AES_CR_SWRST;
	AES->AES_MR = AES_MR_CKEY_PASSWD | (Encrypt ? AES_MR_CIPHER : 0U) |
		(KeyLen == 32U ? AES_MR_KEYSIZE_AES256 : AES_MR_KEYSIZE_AES128);
	for (i = 0U; i < KeyLen / 4U; i++)
	{
		memcpy(&word, pKey + i * 4U, sizeof(word));
		AES->AES_KEYWR[i] = word;
	}
	for (i = 0U; i < 4U; i++)
	{
		memcpy(&word, In + i * 4U, sizeof(word));
		AES->AES_IDATAR[i] = word;
	}
	AES->AES_CR = AES_CR_START;
	while ((AES->AES_ISR & (AES_ISR_DATRDY | AES_ISR_URAD)) == 0U && polls > 0U)
	{
		polls--;
	}
	if ((AES->AES_ISR & (AES_ISR_DATRDY | AES_ISR_URAD)) != AES_ISR_DATRDY)
	{
		CryptoSecureWipe(&word, sizeof(word));
		CryptoSecureWipe(Out, 16U);
		return false;
	}
	for (i = 0U; i < 4U; i++)
	{
		word = AES->AES_ODATAR[i];
		memcpy(Out + i * 4U, &word, sizeof(word));
	}
	CryptoSecureWipe(&word, sizeof(word));
	return true;
}

CRYPTO_STATUS CryptoSam::Random(uint8_t *pOut, size_t Len)
{
	if (pOut != nullptr)
	{
		CryptoSecureWipe(pOut, Len);
	}
	return CRYPTO_STATUS_UNSUPPORTED;
}

bool CryptoSam::IsSecure() const
{
	return false;
}

CRYPTO_STATUS CryptoSam::NarrowCipher(CRYPTO_CIPHER_ALG, int, const CryptoKey &,
	const uint8_t *, size_t, const uint8_t *, size_t, uint8_t *)
{
	return CRYPTO_STATUS_UNSUPPORTED;
}
