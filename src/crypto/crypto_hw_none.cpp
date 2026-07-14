/**-------------------------------------------------------------------------
@file	crypto_hw_none.cpp

@brief	Default hardware crypto provider slot.

		The nRF54 build uses this project slot for the direct CRACEN AES provider.
		Other targets that select this file retain the fail-closed null provider.
		Targets with another hardware engine link their own CryptoHwInit source
		instead, as before.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include "crypto/crypto.h"

#if defined(NRF54L15_XXAA) || defined(NRF54H20_XXAA)

#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "hal/nrf_cracen.h"
#include "hal/nrf_cracen_cm.h"
#include "cracen_bm.h"

#if !defined(NRF_CRACEN_HAS_CRYPTOMASTER) || !(NRF_CRACEN_HAS_CRYPTOMASTER)
#error "This nRF54 target does not provide CRACEN CryptoMaster"
#endif

#define CRACEN_AES_BLOCK_SIZE		16U
#define CRACEN_AES_POLL_LIMIT		100000U

// CryptoMaster scatter/gather descriptor. The hardware consumes four 32-bit
// words in this order. The list terminator is address 1, not nullptr.
typedef struct __CracenDmaDesc CracenDmaDesc_t;
struct __CracenDmaDesc {
	uint8_t         *pAddr;
	CracenDmaDesc_t *pNext;
	uint32_t         Size;
	uint32_t         Tag;
};

static_assert(sizeof(CracenDmaDesc_t) == 16,
			  "CryptoMaster DMA descriptor requires a 32-bit target");

#define CRACEN_DMA_DESC_STOP		((CracenDmaDesc_t *)1)
#define CRACEN_DMA_REALIGN			(1UL << 29)

// CryptoMaster data tags. BA411 is the AES engine. A configuration descriptor
// sets bit 4 and places the BA411 register offset in bits 15:8.
#define CRACEN_DMA_TAG_BA411		1U
#define CRACEN_DMA_TAG_CONFIG(Offset)	((1U << 4) | ((uint32_t)(Offset) << 8))
#define CRACEN_DMA_TAG_LAST			(1U << 5)

#define CRACEN_BA411_CFG_OFFSET		0x00U
#define CRACEN_BA411_KEY_OFFSET		0x08U
#define CRACEN_BA411_MODE_ECB		(1U << 8)
#define CRACEN_BA411_AES128_ENCRYPT	CRACEN_BA411_MODE_ECB

#define CRACEN_CM_ALL_INTERRUPTS \
	(NRF_CRACEN_CM_INT_FETCH_BLOCK_END_MASK | \
	 NRF_CRACEN_CM_INT_FETCH_STOPPED_MASK | \
	 NRF_CRACEN_CM_INT_FETCH_ERROR_MASK | \
	 NRF_CRACEN_CM_INT_PUSH_BLOCK_END_MASK | \
	 NRF_CRACEN_CM_INT_PUSH_STOPPED_MASK | \
	 NRF_CRACEN_CM_INT_PUSH_ERROR_MASK)

static bool CracenCmReset(void)
{
	nrf_cracen_cm_softreset(NRF_CRACENCORE);

	for (uint32_t i = 0; i < CRACEN_AES_POLL_LIMIT; i++)
	{
		if (nrf_cracen_cm_status_get(NRF_CRACENCORE,
				NRF_CRACEN_CM_STATUS_SOFTRESET_BUSY_MASK) == 0)
		{
			nrf_cracen_cm_int_clear(NRF_CRACENCORE, CRACEN_CM_ALL_INTERRUPTS);
			return true;
		}
	}

	nrf_cracen_cm_int_clear(NRF_CRACENCORE, CRACEN_CM_ALL_INTERRUPTS);
	return false;
}

static bool CracenCmWait(void)
{
	for (uint32_t i = 0; i < CRACEN_AES_POLL_LIMIT; i++)
	{
		uint32_t pending = nrf_cracen_cm_int_pending_get(NRF_CRACENCORE);
		if ((pending & (NRF_CRACEN_CM_INT_FETCH_ERROR_MASK |
						NRF_CRACEN_CM_INT_PUSH_ERROR_MASK)) != 0)
		{
			return false;
		}

		uint32_t busy = nrf_cracen_cm_status_get(
			NRF_CRACENCORE,
			NRF_CRACEN_CM_STATUS_BUSY_FETCH_MASK |
			NRF_CRACEN_CM_STATUS_BUSY_PUSH_MASK |
			NRF_CRACEN_CM_STATUS_PUSH_WAITING_MASK);

		if (busy == 0)
		{
			return true;
		}
	}

	return false;
}

static CRYPTO_STATUS CracenAes128Ecb(CryptoDev_t * const pDev,
									 const uint8_t Key[16], const uint8_t In[16],
									 uint8_t Out[16], void *pCtx)
{
	(void)pDev;
	(void)pCtx;

	if (Key == nullptr || In == nullptr || Out == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	if (!CracenTryAcquire())
	{
		return CRYPTO_STATUS_FAIL;
	}

	alignas(uint32_t) uint32_t config = CRACEN_BA411_AES128_ENCRYPT;
	alignas(uint32_t) uint8_t key[CRACEN_AES_BLOCK_SIZE];
	alignas(uint32_t) uint8_t input[CRACEN_AES_BLOCK_SIZE];
	alignas(uint32_t) uint8_t output[CRACEN_AES_BLOCK_SIZE];
	alignas(uint32_t) CracenDmaDesc_t inDesc[3];
	alignas(uint32_t) CracenDmaDesc_t outDesc;

	memcpy(key, Key, sizeof(key));
	memcpy(input, In, sizeof(input));
	memset(output, 0, sizeof(output));
	memset(inDesc, 0, sizeof(inDesc));
	memset(&outDesc, 0, sizeof(outDesc));

	inDesc[0].pAddr = (uint8_t *)&config;
	inDesc[0].pNext = &inDesc[1];
	inDesc[0].Size = sizeof(config) | CRACEN_DMA_REALIGN;
	inDesc[0].Tag = CRACEN_DMA_TAG_BA411 |
					CRACEN_DMA_TAG_CONFIG(CRACEN_BA411_CFG_OFFSET);

	inDesc[1].pAddr = key;
	inDesc[1].pNext = &inDesc[2];
	inDesc[1].Size = sizeof(key) | CRACEN_DMA_REALIGN;
	inDesc[1].Tag = CRACEN_DMA_TAG_BA411 |
					CRACEN_DMA_TAG_CONFIG(CRACEN_BA411_KEY_OFFSET);

	inDesc[2].pAddr = input;
	inDesc[2].pNext = CRACEN_DMA_DESC_STOP;
	inDesc[2].Size = sizeof(input) | CRACEN_DMA_REALIGN;
	inDesc[2].Tag = CRACEN_DMA_TAG_BA411 | CRACEN_DMA_TAG_LAST;

	outDesc.pAddr = output;
	outDesc.pNext = CRACEN_DMA_DESC_STOP;
	outDesc.Size = sizeof(output) | CRACEN_DMA_REALIGN;
	outDesc.Tag = CRACEN_DMA_TAG_LAST;

	nrf_cracen_module_enable(NRF_CRACEN, NRF_CRACEN_MODULE_CRYPTOMASTER_MASK);
	bool ok = CracenCmReset();

	if (ok)
	{
		nrf_cracen_cm_fetch_addr_set(NRF_CRACENCORE, inDesc);
		nrf_cracen_cm_push_addr_set(NRF_CRACENCORE, &outDesc);
		nrf_cracen_cm_config_indirect_set(
			NRF_CRACENCORE,
			(nrf_cracen_cm_config_indirect_mask_t)
			(NRF_CRACEN_CM_CONFIG_INDIRECT_FETCH_MASK |
			 NRF_CRACEN_CM_CONFIG_INDIRECT_PUSH_MASK));

		__DMB();
		nrf_cracen_cm_start(NRF_CRACENCORE);
		ok = CracenCmWait();
		__DMB();
	}

	if (ok)
	{
		memcpy(Out, output, sizeof(output));
	}

	(void)CracenCmReset();
	nrf_cracen_module_disable(NRF_CRACEN, NRF_CRACEN_MODULE_CRYPTOMASTER_MASK);

	CryptoSecureWipe(key, sizeof(key));
	CryptoSecureWipe(input, sizeof(input));
	CryptoSecureWipe(output, sizeof(output));
	CryptoSecureWipe(inDesc, sizeof(inDesc));
	CryptoSecureWipe(&outDesc, sizeof(outDesc));
	config = 0;
	CracenRelease();

	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

static int CracenSelfTest(CryptoDev_t * const pDev)
{
	static const uint8_t key[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
	};
	static const uint8_t plain[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff
	};
	static const uint8_t expected[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a
	};

	uint8_t out[16];
	CRYPTO_STATUS st = CracenAes128Ecb(pDev, key, plain, out, nullptr);
	int rc = (st == CRYPTO_STATUS_OK &&
			  memcmp(out, expected, sizeof(out)) == 0) ? 0 : -1;
	CryptoSecureWipe(out, sizeof(out));
	return rc;
}

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	const uint32_t effective = CRYPTO_CAP_AES128_ECB |
							   CRYPTO_CAP_AES_CMAC |
							   CRYPTO_CAP_AES_CCM |
							   CRYPTO_CAP_AES_GCM |
							   CRYPTO_CAP_SHA256 |
							   CRYPTO_CAP_HMAC_SHA256;

	if (pDev == nullptr || pCfg == nullptr || pCfg->DevNo != 0 ||
		(pCfg->ReqCaps & ~effective) != 0)
	{
		return false;
	}

	memset(pDev, 0, sizeof(*pDev));
	pDev->pName      = "cracen-aes";
	pDev->Cap        = CRYPTO_CAP_AES128_ECB;
	pDev->Props      = CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->KeyCtxSize = 0;
	pDev->EvtCB      = pCfg->EvtCB;
	pDev->Aes128Ecb  = CracenAes128Ecb;
	pDev->SelfTest   = CracenSelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) != 0 && CracenSelfTest(pDev) != 0)
	{
		memset(pDev, 0, sizeof(*pDev));
		return false;
	}

	return true;
}

#else

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	(void)pDev;
	(void)pCfg;
	return false;
}

#endif
