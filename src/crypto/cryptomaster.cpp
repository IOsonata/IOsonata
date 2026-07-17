/**-------------------------------------------------------------------------
@file	cryptomaster.cpp

@brief	Silex CryptoMaster hardware AES engine.

		The CRACEN CryptoMaster module is held once for the complete one-shot
		cipher call. This prevents another CRACEN user from interleaving between
		blocks and avoids releasing and reacquiring the shared core for every block.

@author	Hoang Nguyen Hoan
@date	Jul 2026

@license MIT, (c) 2026 I-SYST. See bt_smp.h for full text.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>

#include "cracen_intrf.h"
#include "crypto/cryptomaster.h"

#ifndef __DMB
#define __DMB()		((void)0)
#endif

#define AES_BLOCK		16U
#define CM_POLL_LIMIT	100000U

static void CmWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0U)
	{
		*p++ = 0U;
	}
}

static uint32_t CmRegRead(Device *pDev, uint32_t Offset)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CRACEN_ADDR_REG);
	return pDev->Read32(off, 4);
}

static void CmRegWrite(Device *pDev, uint32_t Offset, uint32_t Value)
{
	uint8_t off[4] = { (uint8_t)Offset, (uint8_t)(Offset >> 8),
					   (uint8_t)(Offset >> 16), (uint8_t)(Offset >> 24) };
	pDev->DeviceAddress(CRACEN_ADDR_REG);
	pDev->Write32(off, 4, Value);
}

static bool CmReset(Device *pDev)
{
	CmRegWrite(pDev, CRYPTOMASTER_CONFIG, CRYPTOMASTER_SOFTRESET_ENABLE);
	for (volatile uint32_t delay = 0; delay < 64U; delay++) { }
	CmRegWrite(pDev, CRYPTOMASTER_CONFIG, 0U);

	for (uint32_t i = 0; i < CM_POLL_LIMIT; i++)
	{
		if ((CmRegRead(pDev, CRYPTOMASTER_STATUS) &
			 CRYPTOMASTER_STATUS_RESET_BUSY) == 0U)
		{
			CmRegWrite(pDev, CRYPTOMASTER_INT_STATCLR, 0xFFFFFFFFU);
			return true;
		}
	}
	CmRegWrite(pDev, CRYPTOMASTER_INT_STATCLR, 0xFFFFFFFFU);
	return false;
}

static bool CmWait(Device *pDev)
{
	for (uint32_t i = 0; i < CM_POLL_LIMIT; i++)
	{
		if ((CmRegRead(pDev, CRYPTOMASTER_INT_STATRAW) &
			 CRYPTOMASTER_INT_ERROR) != 0U)
		{
			return false;
		}
		if ((CmRegRead(pDev, CRYPTOMASTER_STATUS) &
			 CRYPTOMASTER_STATUS_BUSY) == 0U)
		{
			return true;
		}
	}
	return false;
}

// The caller owns the CRACEN module hold.
static bool CmAesBlock(Device *pDev, const uint8_t Key[16],
						const uint8_t In[16], uint8_t Out[16])
{
	alignas(uint32_t) uint32_t config = CRYPTOMASTER_BA411_MODE_ECB;
	alignas(uint32_t) uint8_t key[AES_BLOCK];
	alignas(uint32_t) uint8_t input[AES_BLOCK];
	alignas(uint32_t) uint8_t output[AES_BLOCK];
	alignas(uint32_t) CryptoMasterDesc_t inDesc[3];
	alignas(uint32_t) CryptoMasterDesc_t outDesc;

	memcpy(key, Key, sizeof(key));
	memcpy(input, In, sizeof(input));
	memset(output, 0, sizeof(output));
	memset(inDesc, 0, sizeof(inDesc));
	memset(&outDesc, 0, sizeof(outDesc));

	inDesc[0].pAddr = (uint8_t *)&config;
	inDesc[0].pNext = &inDesc[1];
	inDesc[0].Size = sizeof(config) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[0].Tag = CRYPTOMASTER_TAG_ENGINE_AES |
		CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_CFG_OFFSET);

	inDesc[1].pAddr = key;
	inDesc[1].pNext = &inDesc[2];
	inDesc[1].Size = sizeof(key) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[1].Tag = CRYPTOMASTER_TAG_ENGINE_AES |
		CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_KEY_OFFSET);

	inDesc[2].pAddr = input;
	inDesc[2].pNext = CRYPTOMASTER_DMA_STOP;
	inDesc[2].Size = sizeof(input) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[2].Tag = CRYPTOMASTER_TAG_ENGINE_AES | CRYPTOMASTER_TAG_LAST;

	outDesc.pAddr = output;
	outDesc.pNext = CRYPTOMASTER_DMA_STOP;
	outDesc.Size = sizeof(output) | CRYPTOMASTER_DMA_REALIGN;
	outDesc.Tag = CRYPTOMASTER_TAG_LAST;

	bool ok = CmReset(pDev);
	if (ok)
	{
		CmRegWrite(pDev, CRYPTOMASTER_FETCH_ADDR, (uint32_t)(uintptr_t)inDesc);
		CmRegWrite(pDev, CRYPTOMASTER_PUSH_ADDR, (uint32_t)(uintptr_t)&outDesc);
		CmRegWrite(pDev, CRYPTOMASTER_CONFIG,
				   CRYPTOMASTER_CONFIG_SCATTERGATHER);
		__DMB();
		CmRegWrite(pDev, CRYPTOMASTER_START, CRYPTOMASTER_START_BOTH);
		ok = CmWait(pDev);
		__DMB();
	}
	if (ok)
	{
		memcpy(Out, output, sizeof(output));
	}
	(void)CmReset(pDev);

	CmWipe(key, sizeof(key));
	CmWipe(input, sizeof(input));
	CmWipe(output, sizeof(output));
	CmWipe(inDesc, sizeof(inDesc));
	CmWipe(&outDesc, sizeof(outDesc));
	config = 0U;
	return ok;
}

bool CryptoMaster::Init(CracenIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	vpCracen = pIntrf;
	Interface(pIntrf);
	return Enable();
}

bool CryptoMaster::Enable()
{
	if (vpCracen == nullptr ||
		!vpCracen->ModuleHold(CRACEN_MODULE_CRYPTOMASTER))
	{
		vbValid = false;
		return false;
	}
	uint32_t present = CmRegRead(this, CRYPTOMASTER_HW_PRESENCE);
	vpCracen->ModuleRelease();
	vbValid = (present & CRYPTOMASTER_PRESENT_AES) != 0U;
	return vbValid;
}

CRYPTO_STATUS CryptoMaster::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
								   const CryptoKey &Key,
								   const uint8_t *pIv, size_t IvLen,
								   const uint8_t *pIn, size_t Len,
								   uint8_t *pOut)
{
	uint32_t required = bEncrypt != 0 ? CRYPTO_KEY_USE_ENCRYPT :
									 CRYPTO_KEY_USE_DECRYPT;
	if (Key.Type != CRYPTO_KEY_AES_128 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		Key.Plain.pData == nullptr || Key.Plain.Len != AES_BLOCK ||
		(Key.Usage & required) == 0U || pIn == nullptr || pOut == nullptr)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}

	// ECB and CBC decryption need the inverse cipher, which this hardware path
	// does not implement. CTR uses the forward block operation for both directions.
	if (bEncrypt == 0 && Alg != CRYPTO_CIPHER_CTR)
	{
		return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen,
								 pIn, Len, pOut);
	}

	if (Alg == CRYPTO_CIPHER_ECB && (Len & (AES_BLOCK - 1U)) != 0U)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if ((Alg == CRYPTO_CIPHER_CTR || Alg == CRYPTO_CIPHER_CBC) &&
		(pIv == nullptr || IvLen != AES_BLOCK))
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (Alg == CRYPTO_CIPHER_CBC && (Len & (AES_BLOCK - 1U)) != 0U)
	{
		return CRYPTO_STATUS_FAIL;
	}
	if (Alg != CRYPTO_CIPHER_ECB && Alg != CRYPTO_CIPHER_CTR &&
		Alg != CRYPTO_CIPHER_CBC)
	{
		return CRYPTO_STATUS_UNSUPPORTED;
	}
	if (vpCracen == nullptr ||
		!vpCracen->ModuleHold(CRACEN_MODULE_CRYPTOMASTER))
	{
		return CRYPTO_STATUS_FAIL;
	}

	const uint8_t *key = Key.Plain.pData;
	CRYPTO_STATUS status = CRYPTO_STATUS_OK;

	if (Alg == CRYPTO_CIPHER_ECB)
	{
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			if (!CmAesBlock(this, key, pIn + off, pOut + off))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
		}
	}
	else if (Alg == CRYPTO_CIPHER_CTR)
	{
		uint8_t ctr[AES_BLOCK], stream[AES_BLOCK];
		memcpy(ctr, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; )
		{
			if (!CmAesBlock(this, key, ctr, stream))
			{
				status = CRYPTO_STATUS_FAIL;
				break;
			}
			size_t count = Len - off < AES_BLOCK ? Len - off : AES_BLOCK;
			for (size_t i = 0; i < count; i++)
			{
				pOut[off + i] = pIn[off + i] ^ stream[i];
			}
			for (int i = AES_BLOCK - 1; i >= 0; i--)
			{
				if (++ctr[i] != 0U) break;
			}
			off += count;
		}
		CmWipe(ctr, sizeof(ctr));
		CmWipe(stream, sizeof(stream));
	}
	else
	{
		uint8_t chain[AES_BLOCK];
		memcpy(chain, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len; off += AES_BLOCK)
		{
			uint8_t block[AES_BLOCK];
			for (size_t i = 0; i < AES_BLOCK; i++)
			{
				block[i] = pIn[off + i] ^ chain[i];
			}
			if (!CmAesBlock(this, key, block, pOut + off))
			{
				status = CRYPTO_STATUS_FAIL;
				CmWipe(block, sizeof(block));
				break;
			}
			memcpy(chain, pOut + off, AES_BLOCK);
			CmWipe(block, sizeof(block));
		}
		CmWipe(chain, sizeof(chain));
	}

	vpCracen->ModuleRelease();
	if (status != CRYPTO_STATUS_OK && Len > 0U)
	{
		memset(pOut, 0, Len);
	}
	return status;
}
