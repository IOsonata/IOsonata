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

std::atomic_flag CryptoEcbNrf53::s_Busy = ATOMIC_FLAG_INIT;

bool CryptoEcbNrf53::AesOpBegin()
{
	return !s_Busy.test_and_set(std::memory_order_acquire);
}

void CryptoEcbNrf53::AesOpEnd()
{
	s_Busy.clear(std::memory_order_release);
}

bool CryptoEcbNrf53::AesEcbEncrypt(const uint8_t Key[16],
								  const uint8_t In[16], uint8_t Out[16])
{
	bool ok = false;
	NRF_ECB_Type *reg = NRF_ECB_NS;

	memcpy(vBlock.Key, Key, 16);
	memcpy(vBlock.Input, In, 16);
	memset(vBlock.Output, 0, 16);
	reg->EVENTS_ENDECB = 0;
	reg->EVENTS_ERRORECB = 0;
	reg->ECBDATAPTR = reinterpret_cast<uintptr_t>(&vBlock);
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
		reg->TASKS_STOPECB = 1;
		for (uint32_t n = 0; n < CRYPTO_ECB_NRF53_TIMEOUT; n++)
		{
			if (reg->EVENTS_ERRORECB != 0 || reg->EVENTS_ENDECB != 0)
			{
				break;
			}
		}
	}
	__DMB();
	if (ok)
	{
		memcpy(Out, vBlock.Output, 16);
	}
	else
	{
		memset(Out, 0, 16);
	}
	reg->EVENTS_ENDECB = 0;
	reg->EVENTS_ERRORECB = 0;
	CryptoSecureWipe(&vBlock, sizeof(vBlock));
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
