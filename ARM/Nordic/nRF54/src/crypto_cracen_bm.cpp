/**-------------------------------------------------------------------------
@file	crypto_cracen_bm.cpp

@brief	Direct nRF54 CRACEN hardware crypto provider.

		Implements the IOsonata hardware crypto slot using the CRACEN
		CryptoMaster/BA411 AES engine and BA414e public-key engine. AES-128
		ECB and P-256 ECDH are native; CMAC, CCM and GCM are derived by the
		generic crypto layer.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "hal/nrf_cracen.h"
#include "hal/nrf_cracen_cm.h"

#include "crypto/crypto.h"
#include "crypto/crypto_p256.h"
#include "cracen_bm.h"
#include "cracen_ba414e_ucode.h"

#if !defined(NRF54L15_XXAA) && !defined(NRF54H20_XXAA)
#error "crypto_cracen_bm.cpp is only for nRF54 targets"
#endif

#if !defined(NRF_CRACEN_HAS_CRYPTOMASTER) || !(NRF_CRACEN_HAS_CRYPTOMASTER)
#error "This nRF54 target does not provide CRACEN CryptoMaster"
#endif

#if !defined(NRF_CRACEN_HAS_PKEIKG) || !(NRF_CRACEN_HAS_PKEIKG)
#error "This nRF54 target does not provide CRACEN PKE"
#endif

#define CRACEN_ACQUIRE_SPINS		100000U
#define CRACEN_AES_BLOCK_SIZE		16U
#define CRACEN_AES_POLL_LIMIT		100000U
#define CRACEN_PKE_POLL_LIMIT		10000000U
#define CRACEN_PKE_RETRY_COUNT		10U

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

// BA414e register interface and CryptoRAM layout. nRF54L15 maps the register
// block at NRF_CRACENCORE->PK, operand RAM at core + 0x8000 and code RAM at
// core + 0xC000. P-256 operands are big-endian and occupy the last 32 bytes of
// a 0x200-byte slot.
#define PKE_REG_CONFIG			0x00U
#define PKE_REG_COMMAND			0x04U
#define PKE_REG_CONTROL			0x08U
#define PKE_REG_STATUS			0x0CU
#define PKE_REG_HWCONFIG		0x18U

#define PKE_CONTROL_START		(1U << 0)
#define PKE_CONTROL_CLEAR_IRQ	(1U << 1)
#define PKE_STATUS_POINT_ERROR	(1U << 4)
#define PKE_STATUS_NOT_INVERTIBLE (1U << 11)
#define PKE_STATUS_BUSY			(1U << 16)
#define PKE_STATUS_ERROR_MASK	0x0001FFF0U

#define PKE_COMMAND_BIG_ENDIAN	(1U << 28)
#define PKE_COMMAND_RAND_SCALAR	(1U << 24)
#define PKE_COMMAND_RAND_PROJ	(1U << 25)
#define PKE_COMMAND_P256		(1U << 20)
#define PKE_COMMAND_POINT_MULT	0x22U
#define PKE_COMMAND_P256_MULT \
	(PKE_COMMAND_POINT_MULT | PKE_COMMAND_BIG_ENDIAN | PKE_COMMAND_P256 | \
	 PKE_COMMAND_RAND_SCALAR | PKE_COMMAND_RAND_PROJ | ((P256_BYTES - 1U) << 8))

#define PKE_CONFIG_PTRS(A, B, C)	((A) | ((B) << 8) | ((C) << 16))
#define PKE_CONFIG_POINT_MULT	PKE_CONFIG_PTRS(12U, 8U, 10U)

#define PKE_SLOT_SIZE			0x200U
#define PKE_SLOT_SCALAR			8U
#define PKE_SLOT_RESULT_X		10U
#define PKE_SLOT_RESULT_Y		11U
#define PKE_SLOT_POINT_X		12U
#define PKE_SLOT_POINT_Y		13U
#define PKE_SLOT_BLIND			15U
#define PKE_DATA_OFFSET			0x8000U
#define PKE_CODE_OFFSET			0xC000U

static const uint8_t s_P256Generator[64] = {
	0x6B,0x17,0xD1,0xF2,0xE1,0x2C,0x42,0x47,0xF8,0xBC,0xE6,0xE5,0x63,0xA4,0x40,0xF2,
	0x77,0x03,0x7D,0x81,0x2D,0xEB,0x33,0xA0,0xF4,0xA1,0x39,0x45,0xD8,0x98,0xC2,0x96,
	0x4F,0xE3,0x42,0xE2,0xFE,0x1A,0x7F,0x9B,0x8E,0xE7,0xEB,0x4A,0x7C,0x0F,0x9E,0x16,
	0x2B,0xCE,0x33,0x57,0x6B,0x31,0x5E,0xCE,0xCB,0xB6,0x40,0x68,0x37,0xBF,0x51,0xF5,
};

struct alignas(uint32_t) CryptoCracenData {
	uint8_t PrivKey[P256_BYTES];
	bool bKeyValid;
};

static_assert(sizeof(CryptoCracenData) <= CRYPTO_MEMSIZE_HW,
			  "CRYPTO_MEMSIZE_HW too small for CryptoCracenData");

static bool CracenAcquire(void)
{
	uint32_t attempts = (__get_IPSR() == 0U) ? CRACEN_ACQUIRE_SPINS : 1U;
	while (attempts-- > 0U)
	{
		if (CracenTryAcquire())
		{
			return true;
		}
		__asm volatile("" ::: "memory");
	}
	return false;
}

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

	if (Key == nullptr || In == nullptr || Out == nullptr || !CracenAcquire())
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

static volatile uint32_t *PkeRegs(void)
{
	return (volatile uint32_t *)&NRF_CRACENCORE->PK;
}

static volatile uint8_t *PkeData(void)
{
	return (volatile uint8_t *)((uintptr_t)NRF_CRACENCORE + PKE_DATA_OFFSET);
}

static volatile uint32_t *PkeCode(void)
{
	return (volatile uint32_t *)((uintptr_t)NRF_CRACENCORE + PKE_CODE_OFFSET);
}

static volatile uint8_t *PkeOperand(uint32_t Slot, uint32_t Size)
{
	return PkeData() + Slot * PKE_SLOT_SIZE + PKE_SLOT_SIZE - Size;
}

static void PkeWrite(volatile uint8_t *pDst, const uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[i];
	}
}

static void PkeRead(uint8_t *pDst, const volatile uint8_t *pSrc, size_t Len)
{
	for (size_t i = 0; i < Len; i++)
	{
		pDst[i] = pSrc[i];
	}
}

static void PkeClearOperand(uint32_t Slot, uint32_t Size)
{
	volatile uint8_t *p = PkeOperand(Slot, Size);
	for (uint32_t i = 0; i < Size; i++)
	{
		p[i] = 0;
	}
}

static bool PkeWaitIdle(uint32_t *pStatus)
{
	volatile uint32_t *reg = PkeRegs();
	for (uint32_t i = 0; i < CRACEN_PKE_POLL_LIMIT; i++)
	{
		uint32_t status = reg[PKE_REG_STATUS / sizeof(uint32_t)];
		if ((status & PKE_STATUS_BUSY) == 0U)
		{
			*pStatus = status & PKE_STATUS_ERROR_MASK;
			return true;
		}
	}
	*pStatus = PKE_STATUS_BUSY;
	return false;
}

static bool PkeLoadMicrocode(void)
{
	volatile uint32_t *pCode = PkeCode();
	const size_t words = CRACEN_BA414E_UCODE_WORDS;

	// Reload when code RAM is empty or was replaced by another firmware layer.
	if (pCode[0] == s_CracenBa414eUcode[0] &&
		pCode[words - 1U] == s_CracenBa414eUcode[words - 1U])
	{
		return true;
	}

	for (size_t i = 0; i < words; i++)
	{
		pCode[i] = s_CracenBa414eUcode[i];
	}
	__DSB();

	for (size_t i = 0; i < words; i++)
	{
		if (pCode[i] != s_CracenBa414eUcode[i])
		{
			return false;
		}
	}
	return true;
}

static bool PkePrepare(void)
{
	nrf_cracen_module_enable(NRF_CRACEN, NRF_CRACEN_MODULE_PKE_IKG_MASK);

	volatile uint32_t *reg = PkeRegs();
	uint32_t status;
	if (!PkeWaitIdle(&status) || status != 0U || !PkeLoadMicrocode())
	{
		return false;
	}

	uint32_t hw = reg[PKE_REG_HWCONFIG / sizeof(uint32_t)];
	const bool p256 = (hw & 0xFFFU) >= P256_BYTES &&
					  (hw & (1U << 16)) != 0U &&
					  (hw & PKE_COMMAND_P256) != 0U;
	return p256;
}

static void PkeCleanup(void)
{
	PkeClearOperand(PKE_SLOT_SCALAR, P256_BYTES);
	PkeClearOperand(PKE_SLOT_RESULT_X, P256_BYTES);
	PkeClearOperand(PKE_SLOT_RESULT_Y, P256_BYTES);
	PkeClearOperand(PKE_SLOT_POINT_X, P256_BYTES);
	PkeClearOperand(PKE_SLOT_POINT_Y, P256_BYTES);
	PkeClearOperand(PKE_SLOT_BLIND, 8U);
	__DMB();

	volatile uint32_t *reg = PkeRegs();
	reg[PKE_REG_CONTROL / sizeof(uint32_t)] = PKE_CONTROL_CLEAR_IRQ;
	nrf_cracen_event_clear(NRF_CRACEN, NRF_CRACEN_EVENT_PKE_IKG);
	nrf_cracen_module_disable(NRF_CRACEN, NRF_CRACEN_MODULE_PKE_IKG_MASK);
}

static bool PkePointMultiply(const uint8_t Point[64], const uint8_t Scalar[32],
							 uint8_t Result[64])
{
	if (Point == nullptr || Scalar == nullptr || Result == nullptr ||
		!P256ScalarInRange(Scalar))
	{
		return false;
	}

	memset(Result, 0, 64U);
	for (uint32_t attempt = 0; attempt < CRACEN_PKE_RETRY_COUNT; attempt++)
	{
		uint8_t blind[8];
		if (!RngGet(blind, sizeof(blind)))
		{
			CryptoSecureWipe(blind, sizeof(blind));
			return false;
		}

		// BA414e countermeasure factor requirements for big-endian mode.
		blind[0] = (uint8_t)((blind[0] & 0x3FU) | 0x20U);
		blind[7] |= 1U;

		if (!CracenAcquire())
		{
			CryptoSecureWipe(blind, sizeof(blind));
			return false;
		}

		bool prepared = PkePrepare();
		uint32_t status = PKE_STATUS_BUSY;
		if (prepared)
		{
			volatile uint32_t *reg = PkeRegs();
			reg[PKE_REG_CONFIG / sizeof(uint32_t)] = PKE_CONFIG_POINT_MULT;
			reg[PKE_REG_COMMAND / sizeof(uint32_t)] = PKE_COMMAND_P256_MULT;

			PkeWrite(PkeOperand(PKE_SLOT_SCALAR, P256_BYTES), Scalar, P256_BYTES);
			PkeWrite(PkeOperand(PKE_SLOT_POINT_X, P256_BYTES), &Point[0], P256_BYTES);
			PkeWrite(PkeOperand(PKE_SLOT_POINT_Y, P256_BYTES), &Point[32], P256_BYTES);
			PkeWrite(PkeOperand(PKE_SLOT_BLIND, sizeof(blind)), blind, sizeof(blind));

			__DMB();
			reg[PKE_REG_CONTROL / sizeof(uint32_t)] =
				PKE_CONTROL_START | PKE_CONTROL_CLEAR_IRQ;
			(void)PkeWaitIdle(&status);
			__DMB();

			if (status == 0U)
			{
				PkeRead(&Result[0], PkeOperand(PKE_SLOT_RESULT_X, P256_BYTES), P256_BYTES);
				PkeRead(&Result[32], PkeOperand(PKE_SLOT_RESULT_Y, P256_BYTES), P256_BYTES);
			}
		}

		PkeCleanup();
		CracenRelease();
		CryptoSecureWipe(blind, sizeof(blind));

		if (prepared && status == 0U &&
			!P256IsZero(&Result[0], P256_BYTES) &&
			!P256IsZero(&Result[32], P256_BYTES))
		{
			return true;
		}
		memset(Result, 0, 64U);

		if (!prepared || status != PKE_STATUS_NOT_INVERTIBLE)
		{
			return false;
		}
	}
	return false;
}

static CryptoCracenData *CracenData(CryptoDev_t * const pDev, void *pKeyCtx)
{
	return (CryptoCracenData *)CryptoResolveKeyCtx(pDev, pKeyCtx,
											  alignof(CryptoCracenData));
}

static void CracenKeyReset(CryptoCracenData *pData)
{
	CryptoSecureWipe(pData->PrivKey, sizeof(pData->PrivKey));
	pData->bKeyValid = false;
}

static CRYPTO_STATUS CracenEcdhKeyGen(CryptoDev_t * const pDev, void *pKeyCtx,
									 uint8_t pPubKey[64], void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCracenData *pData = CracenData(pDev, pKeyCtx);
	if (pData == nullptr || pPubKey == nullptr)
	{
		return CRYPTO_STATUS_FAIL;
	}

	CracenKeyReset(pData);
	if (P256RandomScalar(pData->PrivKey) &&
		PkePointMultiply(s_P256Generator, pData->PrivKey, pPubKey))
	{
		pData->bKeyValid = true;
		return CRYPTO_STATUS_OK;
	}

	CracenKeyReset(pData);
	memset(pPubKey, 0, 64U);
	return CRYPTO_STATUS_FAIL;
}

static CRYPTO_STATUS CracenEcdh(CryptoDev_t * const pDev, void *pKeyCtx,
								const uint8_t pPeerPubKey[64], uint8_t pDhKey[32],
								void *pOpCtx)
{
	(void)pOpCtx;
	CryptoCracenData *pData = CracenData(pDev, pKeyCtx);
	if (pData == nullptr || pPeerPubKey == nullptr || pDhKey == nullptr ||
		!pData->bKeyValid)
	{
		return CRYPTO_STATUS_FAIL;
	}

	uint8_t point[64];
	const bool ok = PkePointMultiply(pPeerPubKey, pData->PrivKey, point);
	CracenKeyReset(pData);
	if (ok)
	{
		memcpy(pDhKey, point, P256_BYTES);
	}
	else
	{
		memset(pDhKey, 0, P256_BYTES);
	}
	CryptoSecureWipe(point, sizeof(point));
	return ok ? CRYPTO_STATUS_OK : CRYPTO_STATUS_FAIL;
}

static int CracenSelfTest(CryptoDev_t * const pDev)
{
	static const uint8_t aesKey[16] = {
		0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
		0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f
	};
	static const uint8_t aesPlain[16] = {
		0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
		0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff
	};
	static const uint8_t aesExpected[16] = {
		0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
		0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a
	};
	static const uint8_t priv[32] = {
		0x3F,0x49,0xF6,0xD4,0xA3,0xC5,0x5F,0x38,0x74,0xC9,0xB3,0xE3,0xD2,0x10,0x3F,0x50,
		0x4A,0xFF,0x60,0x7B,0xEB,0x40,0xB7,0x99,0x58,0x99,0xB8,0xA6,0xCD,0x3C,0x1A,0xBD,
	};
	static const uint8_t pub[64] = {
		0x1E,0xA1,0xF0,0xF0,0x1F,0xAF,0x1D,0x96,0x09,0x59,0x22,0x84,0xF1,0x9E,0x4C,0x00,
		0x47,0xB5,0x8A,0xFD,0x86,0x15,0xA6,0x9F,0x55,0x90,0x77,0xB2,0x2F,0xAA,0xA1,0x90,
		0x4C,0x55,0xF3,0x3E,0x42,0x9D,0xAD,0x37,0x73,0x56,0x70,0x3A,0x9A,0xB8,0x51,0x60,
		0x47,0x2D,0x11,0x30,0xE2,0x8E,0x36,0x76,0x5F,0x89,0xAF,0xF9,0x15,0xB1,0x21,0x4A,
	};
	static const uint8_t dhExpected[32] = {
		0xEC,0x02,0x34,0xA3,0x57,0xC8,0xAD,0x05,0x34,0x10,0x10,0xA6,0x0A,0x39,0x7D,0x9B,
		0x99,0x79,0x6B,0x13,0xB4,0xF8,0x66,0xF1,0x86,0x8D,0x34,0xF3,0x73,0xBF,0xA6,0x98,
	};

	uint8_t aesOut[16];
	uint8_t point[64];
	const bool aesOk = CracenAes128Ecb(pDev, aesKey, aesPlain, aesOut, nullptr) ==
					   CRYPTO_STATUS_OK &&
					   memcmp(aesOut, aesExpected, sizeof(aesOut)) == 0;
	const bool pkeOk = PkePointMultiply(pub, priv, point) &&
					   memcmp(point, dhExpected, sizeof(dhExpected)) == 0;
	CryptoSecureWipe(aesOut, sizeof(aesOut));
	CryptoSecureWipe(point, sizeof(point));
	return (aesOk && pkeOk) ? 0 : -1;
}

bool CryptoHwInit(CryptoDev_t * const pDev, const CryptoCfg_t *pCfg)
{
	const uint32_t effective = CRYPTO_CAP_AES128_ECB |
							   CRYPTO_CAP_ECDH_P256 |
							   CRYPTO_CAP_AES_CMAC |
							   CRYPTO_CAP_AES_CCM |
							   CRYPTO_CAP_AES_GCM |
							   CRYPTO_CAP_SHA256 |
							   CRYPTO_CAP_HMAC_SHA256;

	if (!CryptoCfgValidate(pDev, pCfg, sizeof(CryptoCracenData), effective) ||
		pCfg->DevNo != 0 ||
		((uintptr_t)pCfg->pMem & (alignof(CryptoCracenData) - 1U)) != 0U)
	{
		return false;
	}

	memset(pDev, 0, sizeof(*pDev));
	memset(pCfg->pMem, 0, sizeof(CryptoCracenData));
	pDev->pDevData       = pCfg->pMem;
	pDev->pName          = "cracen-hw";
	pDev->Cap            = CRYPTO_CAP_AES128_ECB | CRYPTO_CAP_ECDH_P256;
	pDev->Props          = CRYPTO_PROP_PLAIN_KEYCTX |
						   CRYPTO_PROP_HARDWARE | CRYPTO_PROP_SYNC;
	pDev->KeyCtxSize     = sizeof(CryptoCracenData);
	pDev->EvtCB          = pCfg->EvtCB;
	pDev->Aes128Ecb      = CracenAes128Ecb;
	pDev->EcdhP256KeyGen = CracenEcdhKeyGen;
	pDev->EcdhP256       = CracenEcdh;
	pDev->SelfTest       = CracenSelfTest;

	if ((pCfg->Flags & CRYPTO_FLAG_SELFTEST) != 0U && CracenSelfTest(pDev) != 0)
	{
		memset(pDev, 0, sizeof(*pDev));
		CryptoSecureWipe(pCfg->pMem, sizeof(CryptoCracenData));
		return false;
	}
	return true;
}
