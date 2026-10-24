/**-------------------------------------------------------------------------
@file	crypto_default_lpc546xx.cpp
@brief	LPC546xx ROM RNG with inherited software crypto.

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

static bool s_Busy = false;

class CryptoLpc546xx : public RngEngine {
public:
	void Disable() override { vbValid = false; }
	void Reset() override { (void)Enable(); }
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

