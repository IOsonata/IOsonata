/**-------------------------------------------------------------------------
@file	cryptomaster.cpp

@brief	Silex CryptoMaster hardware AES engine on the OO engine tree.

		Overrides CipherEngine::Cipher with the BA411e hardware AES. All access
		is raw register read/write at the offsets in cryptomaster.h, reached
		through the injected base and the vendor hooks; no vendor HAL and no
		architecture register name here. ECB, CTR and CBC are built on one
		hardware single-block primitive. The inherited CMAC (CryptoSoftAes) runs
		over this hardware AES through the virtual Cipher call.

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

#define AES_BLOCK			16U
#define CM_POLL_LIMIT		100000U

static void CmWipe(void *pData, size_t Len)
{
	volatile uint8_t *p = (volatile uint8_t *)pData;
	while (Len-- > 0)
	{
		*p++ = 0;
	}
}

// Soft reset the DMA and clear pending interrupt status. The reset is a pulse:
// assert the enable bit, hold briefly, then clear it, and only then wait for the
// reset-busy bit to drop. Clearing the enable is required; without it the block
// stays held in reset.
// Register access through the inherited Device Read / Write, with DevAddr
// selecting the symmetric engine register base. The module is held for the
// operation by the caller.
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

	// Brief hold so the reset takes on all MCUs, then clear it.
	for (volatile uint32_t d = 0; d < 64; d++) { }
	CmRegWrite(pDev, CRYPTOMASTER_CONFIG, 0);

	for (uint32_t i = 0; i < CM_POLL_LIMIT; i++)
	{
		if ((CmRegRead(pDev, CRYPTOMASTER_STATUS) & CRYPTOMASTER_STATUS_RESET_BUSY) == 0U)
		{
			CmRegWrite(pDev, CRYPTOMASTER_INT_STATCLR, 0xFFFFFFFFU);
			return true;
		}
	}
	CmRegWrite(pDev, CRYPTOMASTER_INT_STATCLR, 0xFFFFFFFFU);
	return false;
}

// Wait for both DMA channels; false on a bus error or timeout.
static bool CmWait(Device *pDev)
{
	for (uint32_t i = 0; i < CM_POLL_LIMIT; i++)
	{
		if ((CmRegRead(pDev, CRYPTOMASTER_INT_STATRAW) & CRYPTOMASTER_INT_ERROR) != 0U)
		{
			return false;
		}
		if ((CmRegRead(pDev, CRYPTOMASTER_STATUS) & CRYPTOMASTER_STATUS_BUSY) == 0U)
		{
			return true;
		}
	}
	return false;
}

// One AES-128-ECB block in hardware: config + key + input fetch chain, output
// push. The caller holds the lock and has enabled the module.
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
	inDesc[0].Size  = sizeof(config) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[0].Tag   = CRYPTOMASTER_TAG_ENGINE_AES |
					  CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_CFG_OFFSET);

	inDesc[1].pAddr = key;
	inDesc[1].pNext = &inDesc[2];
	inDesc[1].Size  = sizeof(key) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[1].Tag   = CRYPTOMASTER_TAG_ENGINE_AES |
					  CRYPTOMASTER_TAG_CONFIG(CRYPTOMASTER_BA411_KEY_OFFSET);

	inDesc[2].pAddr = input;
	inDesc[2].pNext = CRYPTOMASTER_DMA_STOP;
	inDesc[2].Size  = sizeof(input) | CRYPTOMASTER_DMA_REALIGN;
	inDesc[2].Tag   = CRYPTOMASTER_TAG_ENGINE_AES | CRYPTOMASTER_TAG_LAST;

	outDesc.pAddr = output;
	outDesc.pNext = CRYPTOMASTER_DMA_STOP;
	outDesc.Size  = sizeof(output) | CRYPTOMASTER_DMA_REALIGN;
	outDesc.Tag   = CRYPTOMASTER_TAG_LAST;

	bool ok = CmReset(pDev);
	if (ok)
	{
		CmRegWrite(pDev, CRYPTOMASTER_FETCH_ADDR, (uint32_t)(uintptr_t)inDesc);
		CmRegWrite(pDev, CRYPTOMASTER_PUSH_ADDR, (uint32_t)(uintptr_t)&outDesc);
		CmRegWrite(pDev, CRYPTOMASTER_CONFIG, CRYPTOMASTER_CONFIG_SCATTERGATHER);
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
	config = 0;
	return ok;
}

// Acquire the lock, power the module, run one hardware block, release.
static bool CmAesBlockLocked(Device *pDev, CracenIntrf *pIntrf,
							 const uint8_t Key[16], const uint8_t In[16],
							 uint8_t Out[16])
{
	// Hold the symmetric module for the block, then release. Matches the
	// reference driver: enable once, run one block, disable once.
	if (!pIntrf->ModuleHold(CRACEN_MODULE_CRYPTOMASTER))
	{
		return false;
	}
	bool ok = CmAesBlock(pDev, Key, In, Out);
	pIntrf->ModuleRelease();
	return ok;
}

bool CryptoMaster::Init(DeviceIntrf * const pIntrf)
{
	if (pIntrf == nullptr)
	{
		return false;
	}
	Interface(pIntrf);
	return Enable();
}

bool CryptoMaster::Enable()
{
	// Probe the AES presence bit; the block must be powered to read it.
	CracenIntrf *pIntrf = (CracenIntrf *)Interface();
	if (!pIntrf->ModuleHold(CRACEN_MODULE_CRYPTOMASTER))
	{
		return false;
	}
	uint32_t present = CmRegRead(this, CRYPTOMASTER_HW_PRESENCE);
	pIntrf->ModuleRelease();

	vbValid = (present & CRYPTOMASTER_PRESENT_AES) != 0U;
	return vbValid;
}

CRYPTO_STATUS CryptoMaster::Cipher(CRYPTO_CIPHER_ALG Alg, int bEncrypt,
								   const CryptoKey &Key,
								   const uint8_t *pIv, size_t IvLen,
								   const uint8_t *pIn, size_t Len, uint8_t *pOut)
{
	if (Key.Type != CRYPTO_KEY_AES_128 || Key.Loc != CRYPTO_KEY_LOC_PLAIN ||
		Key.Plain.pData == nullptr || Key.Plain.Len != AES_BLOCK ||
		pIn == nullptr || pOut == nullptr || bEncrypt == 0)
	{
		// Decrypt and non-AES-128 are not on the hardware forward path; defer to
		// the software base, which declines what it cannot do.
		return CryptoSoftAes::Cipher(Alg, bEncrypt, Key, pIv, IvLen,
									 pIn, Len, pOut);
	}

	const uint8_t *k = Key.Plain.pData;
	CRYPTO_STATUS st = CRYPTO_STATUS_OK;

	if (Alg == CRYPTO_CIPHER_ECB)
	{
		if ((Len & (AES_BLOCK - 1)) != 0)
		{
			return CRYPTO_STATUS_FAIL;
		}
		for (size_t off = 0; off < Len && st == CRYPTO_STATUS_OK; off += AES_BLOCK)
		{
			if (!CmAesBlockLocked(this, (CracenIntrf *)Interface(), k, pIn + off, pOut + off))
			{
				st = CRYPTO_STATUS_FAIL;
			}
		}
	}
	else if (Alg == CRYPTO_CIPHER_CTR)
	{
		if (pIv == nullptr || IvLen != AES_BLOCK)
		{
			return CRYPTO_STATUS_FAIL;
		}
		uint8_t ctr[AES_BLOCK], ks[AES_BLOCK];
		memcpy(ctr, pIv, AES_BLOCK);
		size_t off = 0;
		while (off < Len && st == CRYPTO_STATUS_OK)
		{
			if (!CmAesBlockLocked(this, (CracenIntrf *)Interface(), k, ctr, ks))
			{
				st = CRYPTO_STATUS_FAIL;
				break;
			}
			size_t n = (Len - off < AES_BLOCK) ? (Len - off) : AES_BLOCK;
			for (size_t j = 0; j < n; j++)
			{
				pOut[off + j] = (uint8_t)(pIn[off + j] ^ ks[j]);
			}
			for (int j = AES_BLOCK - 1; j >= 0; j--)
			{
				if (++ctr[j] != 0) break;
			}
			off += n;
		}
		CmWipe(ctr, sizeof(ctr));
		CmWipe(ks, sizeof(ks));
	}
	else if (Alg == CRYPTO_CIPHER_CBC)
	{
		if (pIv == nullptr || IvLen != AES_BLOCK || (Len & (AES_BLOCK - 1)) != 0)
		{
			return CRYPTO_STATUS_FAIL;
		}
		uint8_t chain[AES_BLOCK];
		memcpy(chain, pIv, AES_BLOCK);
		for (size_t off = 0; off < Len && st == CRYPTO_STATUS_OK; off += AES_BLOCK)
		{
			uint8_t blk[AES_BLOCK];
			for (int j = 0; j < AES_BLOCK; j++)
			{
				blk[j] = (uint8_t)(pIn[off + j] ^ chain[j]);
			}
			if (!CmAesBlockLocked(this, (CracenIntrf *)Interface(), k, blk, pOut + off))
			{
				st = CRYPTO_STATUS_FAIL;
			}
			else
			{
				memcpy(chain, pOut + off, AES_BLOCK);
			}
			CmWipe(blk, sizeof(blk));
		}
		CmWipe(chain, sizeof(chain));
	}
	else
	{
		st = CRYPTO_STATUS_UNSUPPORTED;
	}

	return st;
}
