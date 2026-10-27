/**-------------------------------------------------------------------------
@file	crypto_default_r9a02.cpp
@brief	R9A02G021 TRNG with inherited software crypto.

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
#include "crypto/crypto.h"
#include "crypto/crypto_softaes.h"
#include "crypto/crypto_softsha256.h"
#include "coredev/interrupt.h"
#include "r9a02g021.h"
#include "system_r9a02.h"

static bool s_Busy = false;

class CryptoR9a02 : public RngEngine
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
		R9a02ModuleStart(R9A02_MSTPCR_C, R9A02_MSTPCRC_TRNG);
		vbValid = R9a02ModuleRunning(R9A02_MSTPCR_C, R9A02_MSTPCRC_TRNG);
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
		uint32_t polls;
		uint8_t value;
		bool ok = true;
		unsigned i;
		const size_t fullWords = Len & ~(size_t)3U;

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
			R9A02_TRNG->TRNGSCR1 = 0x10U;
			R9A02_TRNG->TRNGSCR0 = R9A02_TRNG_EN;
			while (offset < Len && ok)
			{
				R9A02_TRNG->TRNGSCR0 = R9A02_TRNG_EN | R9A02_TRNG_ST;
				polls = 1000000U;
				while ((R9A02_TRNG->TRNGSCR0 & R9A02_TRNG_RDY) == 0U && polls > 0U)
				{
					polls--;
				}
				if ((R9A02_TRNG->TRNGSCR0 & R9A02_TRNG_RDY) == 0U)
				{
					ok = false;
					break;
				}
				// A generated seed is always four byte-wide reads. Avoid
				// branching after each read in the common aligned path.
				if (offset < fullWords)
				{
					pOut[offset++] = R9A02_TRNG->TRNGSDR;
					pOut[offset++] = R9A02_TRNG->TRNGSDR;
					pOut[offset++] = R9A02_TRNG->TRNGSDR;
					pOut[offset++] = R9A02_TRNG->TRNGSDR;
				}
				else
				{
					// Drain all four bytes to clear RDY, even for a short
					// caller buffer, but never write beyond Len.
					for (i = 0U; i < 4U; i++)
					{
						value = R9A02_TRNG->TRNGSDR;
						if (offset < Len)
						{
							pOut[offset++] = value;
						}
					}
				}
			}
			R9A02_TRNG->TRNGSCR0 = 0U;
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

static CryptoR9a02 s_Hardware;

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

