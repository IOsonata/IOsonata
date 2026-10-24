#include <atomic>
#include <new>
#include <string.h>
#include "nrf.h"
#include "crypto_ecb_nrf53.h"
#include "crypto/icrypto.h"

#if !defined(NRF5340_XXAA_NETWORK)
#error "CryptoEcbNrf53 is only available on the nRF5340 network core"
#endif

#ifndef CRYPTO_ECB_NRF53_TIMEOUT
#define CRYPTO_ECB_NRF53_TIMEOUT 1000000U
#endif

CryptoEcbNrf53::EcbBlock CryptoEcbNrf53::s_Block = {};
std::atomic_flag CryptoEcbNrf53::s_Busy = ATOMIC_FLAG_INIT;
bool CryptoEcbNrf53::s_DmaUnresolved = false;

bool CryptoEcbNrf53::AesOpBegin()
{
	return !s_Busy.test_and_set(std::memory_order_acquire);
}

void CryptoEcbNrf53::AesOpEnd()
{
	// A timeout without a terminal event leaves DMA ownership unresolved.
	// Retain exclusive ownership until a system reset, not just a new object.
	if (!s_DmaUnresolved)
	{
		s_Busy.clear(std::memory_order_release);
	}
}

bool CryptoEcbNrf53::AesEcbEncrypt(const uint8_t Key[16],
								  const uint8_t In[16], uint8_t Out[16])
{
	bool ok = false;
	bool stopped = false;
	NRF_ECB_Type *reg = NRF_ECB_NS;

	memcpy(s_Block.Key, Key, 16);
	memcpy(s_Block.Input, In, 16);
	memset(s_Block.Output, 0, 16);
	reg->EVENTS_ENDECB = 0;
	reg->EVENTS_ERRORECB = 0;
	reg->ECBDATAPTR = reinterpret_cast<uintptr_t>(&s_Block);
	__DMB();
	reg->TASKS_STARTECB = 1;

	for (uint32_t n = 0; n < CRYPTO_ECB_NRF53_TIMEOUT; n++)
	{
		if (reg->EVENTS_ERRORECB != 0)
		{
			break;
		}
		if (reg->EVENTS_ENDECB != 0)
		{
			ok = true;
			break;
		}
	}
	if (!ok)
	{
		stopped = reg->EVENTS_ERRORECB != 0 || reg->EVENTS_ENDECB != 0;
		reg->TASKS_STOPECB = 1;
		for (uint32_t n = 0; n < CRYPTO_ECB_NRF53_TIMEOUT; n++)
		{
			if (reg->EVENTS_ERRORECB != 0 || reg->EVENTS_ENDECB != 0)
			{
				stopped = true;
				break;
			}
		}
	}
	__DMB();
	if (ok)
	{
		memcpy(Out, s_Block.Output, 16);
	}
	else
	{
		memset(Out, 0, 16);
	}
	if (ok || stopped)
	{
		reg->EVENTS_ENDECB = 0;
		reg->EVENTS_ERRORECB = 0;
		CryptoSecureWipe(&s_Block, sizeof(s_Block));
	}
	else
	{
		// The peripheral may still access its DMA block. Do not erase or
		// reuse the block, and prevent any later ECB operation this boot.
		s_DmaUnresolved = true;
	}
	return ok;
}

CRYPTO_STATUS CryptoEcbNrf53::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
									const CryptoKey &Key,
									const uint8_t *pIv, size_t IvLen,
									const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	if (Alg != CRYPTO_CIPHER_ECB || bEncrypt == 0)
	{
		return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen, pIn, Len, pOut);
	}
	if (Key.Type != CRYPTO_KEY_AES_128 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		Key.Plain.pData == nullptr || Key.Plain.Len != 16 ||
		(Key.Usage & CRYPTO_KEY_USE_ENCRYPT) == 0 || pIn == nullptr ||
		pOut == nullptr || (Len & 15U) != 0)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (!AesOpBegin())
	{
		memset(pOut, 0, Len);
		return CRYPTO_STATUS_BUSY;
	}
	CRYPTO_STATUS status = CRYPTO_STATUS_OK;
	for (size_t off = 0; off < Len; off += 16)
	{
		if (!AesEcbEncrypt(Key.Plain.pData, pIn + off, pOut + off))
		{
			status = CRYPTO_STATUS_FAIL;
			break;
		}
	}
	AesOpEnd();
	if (status != CRYPTO_STATUS_OK)
	{
		memset(pOut, 0, Len);
	}
	return status;
}

CryptoEcbNrf53 *CryptoEcbNrf53Create(void *pMem, size_t MemSize)
{
	if (pMem == nullptr || MemSize < sizeof(CryptoEcbNrf53) ||
		(reinterpret_cast<uintptr_t>(pMem) & (alignof(CryptoEcbNrf53) - 1U)) != 0)
	{
		return nullptr;
	}
	CryptoEcbNrf53 *p = new (pMem) CryptoEcbNrf53();
	return p->Enable() ? p : nullptr;
}
