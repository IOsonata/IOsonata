/**-------------------------------------------------------------------------
@file	crypto_default.cpp
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
#include <new>
#include "crypto/crypto.h"

static_assert(sizeof(Crypto) <= CRYPTO_MEMSIZE, "Crypto arena too small");
static_assert(alignof(Crypto) <= CRYPTO_MEMALIGN, "Crypto arena alignment");

Crypto *CryptoCreate(void *pMem, size_t MemSize)
{
	Crypto *pCrypto;

	if (pMem == nullptr || MemSize < sizeof(Crypto) ||
		(uintptr_t)pMem % alignof(Crypto) != 0U)
	{
		return nullptr;
	}
	pCrypto = new (pMem) Crypto();
	if (!pCrypto->Enable())
	{
		pCrypto->Disable();
		pCrypto->~Crypto();
		return nullptr;
	}
	return pCrypto;
}
