/**-------------------------------------------------------------------------
@file	crypto_ecb_nrf53.h

@brief	IOsonata crypto ecb nrf53.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

// Compatibility for existing direct users; MCU libraries use the common Nordic driver.
#ifndef __CRYPTO_ECB_NRF53_H__
#define __CRYPTO_ECB_NRF53_H__
#include "../../include/crypto_ecb_nrf.h"
using CryptoEcbNrf53 = CryptoEcbNrf;
#define CRYPTO_ECB_NRF53_MEMSIZE sizeof(CryptoEcbNrf)
inline CryptoEcbNrf53 *CryptoEcbNrf53Create(void *pMem, size_t MemSize)
{
	return CryptoEcbNrfCreate(pMem, MemSize);
}
#endif
