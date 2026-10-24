#include <atomic>
#include <new>
#include <string.h>
#include "nrf.h"
#include "crypto_ecb_nrf53.h"
#include "crypto/icrypto.h"
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
#include "nrf_soc.h"
#include "nrf_sdm.h"
#endif

#if !defined(NRF5340_XXAA_NETWORK) && !defined(NRF52_SERIES) && \
	!defined(NRF52805_XXAA) && !defined(NRF52810_XXAA) && \
	!defined(NRF52832_XXAA) && !defined(NRF52840_XXAA)
#error "This ECB driver requires nRF52 or the nRF5340 network core"
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
#if defined(NRF5340_XXAA_NETWORK)
	NRF_ECB_Type *reg = NRF_ECB_NS;
#else
	NRF_ECB_Type *reg = NRF_ECB;
#endif
#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT
	uint8_t enabled = 0U;
	uint32_t status;
	nrf_ecb_hal_data_t data;

	if (sd_softdevice_is_enabled(&enabled) != NRF_SUCCESS)
	{
		CryptoSecureWipe(Out, 16U);
		return false;
	}
	if (enabled != 0U)
	{
		memcpy(data.key, Key, 16U);
		memcpy(data.cleartext, In, 16U);
		status = sd_ecb_block_encrypt(&data);
		if (status == NRF_SUCCESS)
		{
			memcpy(Out, data.ciphertext, 16U);
		}
		else
		{
			CryptoSecureWipe(Out, 16U);
		}
		CryptoSecureWipe(&data, sizeof(data));
		return status == NRF_SUCCESS;
	}
#endif

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
