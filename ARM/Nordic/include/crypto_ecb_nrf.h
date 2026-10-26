/**-------------------------------------------------------------------------
@file	crypto_ecb_nrf.h

@brief	IOsonata crypto ecb nrf.

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

#ifndef __CRYPTO_ECB_NRF_H__
#define __CRYPTO_ECB_NRF_H__

#include <atomic>
#include <stddef.h>
#include <stdint.h>
#include "crypto/crypto_softaes.h"

// Shared nRF52 / nRF5340 network-core ECB accelerator.
// CMAC and CCM use the inherited
// CryptoSoftAes algorithms over the hardware ECB block primitive.
class CryptoEcbNrf : public CryptoSoftAes
{
public:
	CryptoEcbNrf()
	{
	}
	bool Enable() override
	{
		return CryptoSoftAes::Enable();
	}
	void Disable() override
	{
		CryptoSoftAes::Disable();
	}
	void Reset() override
	{
	}
protected:
	bool AesOpBegin() override;
	void AesOpEnd() override;
	bool AesEcbEncrypt(const uint8_t Key[16], const uint8_t In[16],
					   uint8_t Out[16]) override;
private:
	struct alignas(4) EcbBlock
	{
		uint8_t Key[16];
		uint8_t Input[16];
		uint8_t Output[16];
	};
	// Shared DMA storage remains valid even if a caller destroys its engine.
	static EcbBlock s_Block;
	static std::atomic_flag s_Busy;
	static bool s_DmaUnresolved;
};

#define CRYPTO_ECB_NRF_MEMSIZE sizeof(CryptoEcbNrf)
CryptoEcbNrf *CryptoEcbNrfCreate(void *pMem, size_t MemSize);

#endif
